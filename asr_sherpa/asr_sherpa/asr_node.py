#!/usr/bin/env python3
"""Offline ASR node based on sherpa-onnx.

- Uses Silero VAD to detect speech.
- A speech segment is finalized after the user pauses for a configurable
  period of time (default 0.5 s).
- Each finalized segment is decoded with an offline (non-streaming)
  SenseVoice model and the resulting text is published on
  ``/asr/final_text`` (std_msgs/msg/String).
- Both float32 and int8 model files are kept on disk; float32 is used
  by default. Set the parameter ``use_int8:=true`` to use int8.
"""

import os
import queue
import threading
from pathlib import Path

import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SAMPLE_RATE = 16000
MODEL_SUBDIR = 'sense-voice-zh-en-ja-ko-yue-2025-09-09'
VAD_SUBDIR = 'vad'


def resolve_models_root(explicit_path: str):
    """Locate the directory that contains the downloaded models."""
    if explicit_path:
        p = Path(explicit_path).expanduser()
        if p.is_dir():
            return p
    env_path = os.environ.get('ASR_MODELS_DIR')
    if env_path:
        p = Path(env_path).expanduser()
        if p.is_dir():
            return p
    # Walk up from this file and look for <dir>/src/models.
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / 'src' / 'models'
        if candidate.is_dir():
            return candidate
    return None


class AsrSherpaNode(Node):

    def __init__(self):
        super().__init__('asr_sherpa')

        # ---- Model selection parameters ----
        self.declare_parameter('models_dir', '')
        self.declare_parameter('use_int8', True)
        self.declare_parameter('language', 'auto')
        self.declare_parameter('use_itn', True)
        self.declare_parameter('num_threads', 4)

        # ---- VAD / segmentation parameters ----
        self.declare_parameter('silence_duration', 0.5)
        self.declare_parameter('vad_threshold', 0.5)
        self.declare_parameter('min_speech_duration', 0.2)
        self.declare_parameter('max_speech_duration', 10.0)

        # ---- Microphone parameters ----
        # None -> type NOT_SET so users can pass an index (int) or a name (str)
        self.declare_parameter('input_device', None)
        self.declare_parameter('mic_sample_rate', SAMPLE_RATE)

        models_root = resolve_models_root(
            self.get_parameter('models_dir').value)
        if models_root is None:
            raise RuntimeError(
                'Could not find the models directory. Please download the models '
                'to <workspace>/src/models or set the parameter models_dir / '
                'environment variable ASR_MODELS_DIR.')

        model_dir = models_root / MODEL_SUBDIR
        vad_path = models_root / VAD_SUBDIR / 'silero_vad.onnx'

        use_int8 = bool(self.get_parameter('use_int8').value)
        model_file = 'model.int8.onnx' if use_int8 else 'model.onnx'
        model_path = model_dir / model_file
        tokens_path = model_dir / 'tokens.txt'

        for path in (vad_path, model_path, tokens_path):
            if not path.is_file():
                raise RuntimeError(f'Required model file not found: {path}')

        self._sample_rate = SAMPLE_RATE

        # ---- Offline recognizer ----
        import sherpa_onnx
        self._sherpa_onnx = sherpa_onnx
        self.get_logger().info(
            f'Loading offline SenseVoice model: {model_path} '
            f'({"int8" if use_int8 else "float32"})')
        self.recognizer = sherpa_onnx.OfflineRecognizer.from_sense_voice(
            tokens=str(tokens_path),
            model=str(model_path),
            num_threads=int(self.get_parameter('num_threads').value),
            language=str(self.get_parameter('language').value),
            use_itn=bool(self.get_parameter('use_itn').value),
        )

        # ---- VAD ----
        vad_config = sherpa_onnx.VadModelConfig()
        vad_config.silero_vad.model = str(vad_path)
        vad_config.silero_vad.threshold = float(
            self.get_parameter('vad_threshold').value)
        vad_config.silero_vad.min_speech_duration = float(
            self.get_parameter('min_speech_duration').value)
        # User pauses for this long -> the segment is separated.
        vad_config.silero_vad.min_silence_duration = float(
            self.get_parameter('silence_duration').value)
        vad_config.silero_vad.max_speech_duration = float(
            self.get_parameter('max_speech_duration').value)
        vad_config.silero_vad.window_size = 512
        vad_config.sample_rate = self._sample_rate
        self.vad = sherpa_onnx.VoiceActivityDetector(
            vad_config, buffer_size_in_seconds=100)

        # ---- Publisher ----
        self.publisher = self.create_publisher(String, '/asr/final_text', 10)

        # ---- Audio input ----
        self._audio_queue: 'queue.Queue[np.ndarray]' = queue.Queue()
        input_device = self._resolve_input_device(
            self.get_parameter('input_device').value)
        mic_rate = int(self.get_parameter('mic_sample_rate').value)
        self._mic_rate = mic_rate

        self._stream = sd.InputStream(
            channels=1,
            dtype='float32',
            samplerate=mic_rate,
            blocksize=512 if mic_rate == self._sample_rate else None,
            device=input_device,
            callback=self._audio_callback,
        )
        self._stream.start()
        dev_name = sd.query_devices(self._stream.device)['name']
        self.get_logger().info(
            f'Listening on device: {dev_name} ({mic_rate} Hz). '
            f'Segmentation silence: '
            f'{vad_config.silero_vad.min_silence_duration} s')

        self._running = True
        self._worker = threading.Thread(target=self._process_loop, daemon=True)
        self._worker.start()

    @staticmethod
    def _resolve_input_device(value):
        """Resolve the input_device parameter to a PortAudio device or None.

        Supported values:
          - negative int / empty string  -> system default input device
          - non-negative int             -> PortAudio device index
          - string                       -> substring matched against the
                                           device name (case-insensitive),
                                           e.g. "Yundea", "hw:2,0", "USB Audio"
        """
        if value is None or value == '' or value == '-1':
            return None
        if isinstance(value, bool):
            return None
        if isinstance(value, int):
            return value if value >= 0 else None
        if isinstance(value, float):
            return int(value) if int(value) >= 0 else None
        # string: try integer first, then name substring match
        s = str(value).strip()
        if s.lstrip('-').isdigit():
            idx = int(s)
            return idx if idx >= 0 else None
        devices = sd.query_devices()
        matches = []
        for idx, d in enumerate(devices):
            if d.get('max_input_channels', 0) <= 0:
                continue
            if s.lower() in d.get('name', '').lower():
                matches.append((idx, d['name']))
        if not matches:
            raise RuntimeError(
                f'No input device matched name "{s}". Available input devices: '
                + ', '.join(f'{i}:{d["name"]}'
                            for i, d in enumerate(devices)
                            if d.get('max_input_channels', 0) > 0))
        if len(matches) > 1:
            raise RuntimeError(
                f'Device name "{s}" matched multiple devices: '
                + ', '.join(f'{i}:{n}' for i, n in matches)
                + '. Use a more specific name or the device index.')
        idx, name = matches[0]
        return idx

    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f'Audio stream status: {status}',
                                   throttle_duration_sec=2.0)
        self._audio_queue.put(indata.reshape(-1).copy())

    def _process_loop(self):
        while self._running and rclpy.ok():
            try:
                samples = self._audio_queue.get(timeout=0.2)
            except queue.Empty:
                samples = None
            if samples is not None and samples.size > 0:
                if self._mic_rate != self._sample_rate:
                    samples = self._resample(samples, self._mic_rate,
                                            self._sample_rate)
                self.vad.accept_waveform(samples.astype(np.float32))
            self._decode_segments()

        # Flush any remaining speech on shutdown.
        try:
            self.vad.flush()
            self._decode_segments()
        except Exception as err:  # noqa: BLE001
            self.get_logger().warn(f'Error while flushing VAD: {err}')

    @staticmethod
    def _resample(samples: np.ndarray, orig_rate: int,
                  target_rate: int) -> np.ndarray:
        """Linear resampling (used only when the mic rate is not 16 kHz)."""
        if orig_rate == target_rate:
            return samples
        duration = samples.shape[0] / orig_rate
        new_length = max(1, int(round(duration * target_rate)))
        old_pos = np.arange(samples.shape[0])
        new_pos = np.linspace(0, samples.shape[0] - 1, new_length)
        return np.interp(new_pos, old_pos, samples).astype(np.float32)

    def _decode_segments(self):
        while not self.vad.empty():
            speech = self.vad.front.samples
            duration = len(speech) / self._sample_rate
            stream = self.recognizer.create_stream()
            stream.accept_waveform(self._sample_rate, speech)
            self.recognizer.decode_stream(stream)
            text = stream.result.text.strip()
            self.vad.pop()

            if not text:
                self.get_logger().info(
                    f'Segment of {duration:.2f} s produced no text.')
                continue

            self.get_logger().info(
                f'Recognized ({duration:.2f} s): {text}')
            msg = String()
            msg.data = text
            self.publisher.publish(msg)

    def destroy_node(self):
        self._running = False
        try:
            self._stream.stop()
            self._stream.close()
        except Exception:  # noqa: BLE001
            pass
        if self._worker.is_alive():
            self._worker.join(timeout=1.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AsrSherpaNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
