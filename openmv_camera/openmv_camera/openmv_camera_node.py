#!/usr/bin/env python3
"""OpenMV camera frame grabber node.

Publishes ``sensor_msgs/Image`` on ``/openmv/image_raw`` at a configurable rate
by driving the OpenMV board through its USB CDC virtual serial port using the
MicroPython raw-REPL protocol.  No ``pyopenmv`` dependency is required.

Protocol (empirically confirmed on this hardware):
  * DTR must be asserted (``dtr=True``) or the board answers with 0 bytes.
  * Enter raw REPL: send ``\\x03\\x03`` (Ctrl-C x2) then ``\\x01`` (Ctrl-A);
    the board replies with ``raw REPL; CTRL-B to exit\\r\\n>``.
  * Execute a code block: write the code, then ``\\x04`` (Ctrl-D).
  * Response format:  ``b'OK' + stdout + b'\\x04' + stderr + b'\\x04>'``.
    So stdout = everything between the leading ``OK`` and the first ``\\x04``.

The sensor is initialised ONCE at startup (reset + warm-up) so the sensor
keeps streaming; each timer tick only does snapshot/compress/base64, which is
fast enough to easily sustain the default 5 Hz.
"""

import base64
import time

import cv2
import numpy as np
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# framesize string -> OpenMV sensor constant name
FRAMESIZE_MAP = {
    'qqvga': 'sensor.QQVGA',
    'qcif':  'sensor.QCIF',
    'qvga':  'sensor.QVGA',
    'cif':   'sensor.CIF',
    'vga':   'sensor.VGA',
}

PIXFORMAT_MAP = {
    'rgb565': 'sensor.RGB565',
    'grayscale': 'sensor.GRAYSCALE',
}


class OpenMVCameraNode(Node):

    def __init__(self):
        super().__init__('openmv_camera')

        self.declare_parameter('serial_port', '/dev/openmv')
        self.declare_parameter('frame_rate', 5.0)
        self.declare_parameter('frame_id', 'openmv_camera')
        self.declare_parameter('framesize', 'qvga')
        self.declare_parameter('pixformat', 'rgb565')
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('warmup_ms', 2000)

        self.serial_port = self.get_parameter('serial_port').value
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.framesize = self.get_parameter('framesize').value.lower()
        self.pixformat = self.get_parameter('pixformat').value.lower()
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.warmup_ms = int(self.get_parameter('warmup_ms').value)

        if self.framesize not in FRAMESIZE_MAP:
            self.get_logger().warn(
                f"unknown framesize '{self.framesize}', falling back to qvga")
            self.framesize = 'qvga'
        if self.pixformat not in PIXFORMAT_MAP:
            self.get_logger().warn(
                f"unknown pixformat '{self.pixformat}', falling back to rgb565")
            self.pixformat = 'rgb565'

        self._bridge = CvBridge()
        self._pub = self.create_publisher(
            Image, '/openmv/image_raw', qos_profile_sensor_data)

        self._ser = None
        self._fail_count = 0
        self._last_connect_attempt = 0.0
        self._connect()   # may fail; timer will keep retrying

        period = 1.0 / self.frame_rate if self.frame_rate > 0 else 0.2
        self._timer = self.create_timer(period, self._timer_cb)
        self.get_logger().info(
            f'openmv_camera started: {self.serial_port} @ {self.frame_rate:.1f}Hz '
            f'({self.framesize}/{self.pixformat}) -> /openmv/image_raw')

    # ------------------------------------------------------------------ serial
    def _connect(self):
        """Open the serial port, enter raw REPL, and initialise the sensor."""
        self._last_connect_attempt = time.time()
        # close any stale handle
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        try:
            # NOTE: do NOT rely on a long blocking timeout + read(N);
            # read(N) blocks until N bytes or the timeout elapses, which
            # adds ~timeout of dead air per frame.  We use in_waiting
            # (non-blocking drain) in _drain(), so keep timeout small.
            ser = serial.Serial(
                self.serial_port, baudrate=115200, timeout=0.05,
                write_timeout=2.0, inter_byte_timeout=None)
        except serial.SerialException as exc:
            self.get_logger().error(
                f'cannot open {self.serial_port}: {exc}. '
                f'Check udev/permissions (e.g. "sudo chmod 666 {self.serial_port}").')
            return
        ser.dtr = True   # REQUIRED for OpenMV USB CDC to reply
        ser.rts = True
        time.sleep(0.5)
        self._ser = ser

        if not self._enter_raw_repl():
            self.get_logger().error('failed to enter raw REPL; will retry')
            return
        if not self._init_sensor():
            self.get_logger().error('failed to initialise sensor; will retry')
            return

        self._fail_count = 0
        self.get_logger().info('OpenMV connected and sensor initialised')

    def _drain(self, marker: bytes, timeout: float) -> bytes:
        """Non-blocking drain: read whatever is available (in_waiting) in a
        tight loop until ``marker`` is seen in the tail or ``timeout`` elapses.
        This avoids the dead-air that ser.read(N) adds when it blocks waiting
        to fill its buffer.  Returns all bytes read."""
        data = bytearray()
        t0 = time.time()
        mlen = len(marker)
        while time.time() - t0 < timeout:
            n = self._ser.in_waiting
            if n:
                data.extend(self._ser.read(n))
                if marker in data[-(mlen + 4):]:
                    break
            else:
                time.sleep(0.001)
        return bytes(data)

    def _enter_raw_repl(self) -> bool:
        ser = self._ser
        ser.reset_input_buffer()
        ser.write(b'\x03\x03')     # Ctrl-C x2: interrupt any running script
        ser.flush()
        time.sleep(0.4)
        ser.reset_input_buffer()
        ser.write(b'\x01')         # Ctrl-A: enter raw REPL
        ser.flush()
        data = self._drain(b'raw REPL', 3.0)
        ser.reset_input_buffer()
        return b'raw REPL' in data

    def _exec_raw(self, code: str, timeout: float = 8.0):
        """Execute a code block in raw REPL. Returns (stdout, stderr, raw)."""
        ser = self._ser
        ser.reset_input_buffer()
        ser.write(code.encode())
        ser.write(b'\x04')         # Ctrl-D: run
        ser.flush()
        raw = self._drain(b'\x04>', timeout)
        body = raw[2:] if raw.startswith(b'OK') else raw
        if body.endswith(b'\x04>'):
            body = body[:-2]
        if b'\x04' in body:
            stdout, stderr = body.split(b'\x04', 1)
        else:
            stdout, stderr = body, b''
        return stdout, stderr, raw

    def _init_sensor(self) -> bool:
        fs = FRAMESIZE_MAP[self.framesize]
        pf = PIXFORMAT_MAP[self.pixformat]
        code = (
            'import sensor, image, ubinascii, sys\n'
            'sensor.reset()\n'
            f'sensor.set_pixformat({pf})\n'
            f'sensor.set_framesize({fs})\n'
            'sensor.set_auto_gain(True)\n'
            'sensor.set_auto_exposure(True)\n'
            'sensor.set_auto_whitebal(True)\n'
            f'sensor.skip_frames(time={self.warmup_ms})\n'
        )
        stdout, stderr, raw = self._exec_raw(code, timeout=10.0)
        if b'Traceback' in stdout or b'Traceback' in stderr:
            self.get_logger().error(
                'sensor init failed:\n' +
                (stdout + b'\n' + stderr).decode('utf-8', 'replace'))
            return False
        return True

    # -------------------------------------------------------------- frame grab
    def _grab_one(self):
        """Grab one frame. Returns a decoded BGR np.ndarray or None on failure."""
        q = self.jpeg_quality
        code = (
            'img = sensor.snapshot()\n'
            f'img.compress(quality={q})\n'
            'buf = img.bytearray()\n'
            "sys.stdout.write('FRAME %d %d %d\\r\\n' % (len(buf), img.width(), img.height()))\n"
            'sys.stdout.write(ubinascii.b2a_base64(buf).decode())\n'
        )
        stdout, stderr, raw = self._exec_raw(code, timeout=5.0)
        if not stdout:
            if b'Traceback' in stderr:
                self.get_logger().error(
                    'grab script error: ' +
                    stderr.decode('utf-8', 'replace').strip())
            return None
        if b'Traceback' in stdout:
            self.get_logger().error(
                'grab script error: ' + stdout.decode('utf-8', 'replace').strip())
            return None
        try:
            text = stdout.decode('utf-8', 'replace')
            lines = [ln for ln in text.replace('\r', '\n').split('\n') if ln.strip()]
            header = next((ln for ln in lines if ln.startswith('FRAME ')), None)
            if header is None:
                self.get_logger().error(f'no FRAME header in: {text[:200]!r}')
                return None
            parts = header.split()
            flen, w, h = int(parts[1]), int(parts[2]), int(parts[3])
            # base64 = all non-empty lines after the header
            b64 = ''
            collecting = False
            for ln in text.replace('\r', '\n').split('\n'):
                if ln.startswith('FRAME '):
                    collecting = True
                    continue
                if collecting and ln.strip():
                    b64 += ln.strip()
            jpg = base64.b64decode(b64)
        except Exception as exc:
            self.get_logger().error(f'frame decode error: {exc}')
            return None

        arr = np.frombuffer(jpg, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().error('JPEG decode failed')
            return None
        if (w, h) != (img.shape[1], img.shape[0]):
            self.get_logger().warn(
                f'declared {w}x{h} != decoded {img.shape[1]}x{img.shape[0]}')
        return img

    # ------------------------------------------------------------------- timer
    def _timer_cb(self):
        if self._ser is None:
            # throttle reconnect attempts
            if time.time() - self._last_connect_attempt > 2.0:
                self.get_logger().info('attempting to (re)connect OpenMV...')
                self._connect()
            return

        img = self._grab_one()
        if img is None:
            self._fail_count += 1
            if self._fail_count >= 3:
                self.get_logger().warn(
                    f'{self._fail_count} consecutive failures; reconnecting')
                self._connect()
            return

        self._fail_count = 0
        try:
            msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self._pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f'publish failed: {exc}')

    def destroy_node(self):
        try:
            if self._ser is not None:
                # leave raw REPL so the board can run user code again on reset
                self._ser.write(b'\x02')
                self._ser.flush()
                self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenMVCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
