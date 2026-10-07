# asr_sherpa

基于 [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) 的 **离线（非流式）语音识别** ROS 2 功能包。

使用 Silero VAD 检测麦克风中的语音，用户停顿指定时间（默认 0.5 s）后将该段语音切分并送入离线 SenseVoice 模型识别，识别得到的文字发布到 `/asr/final_text` 话题。

- **完全离线**：模型与识别均在本地 CPU 运行，不需要联网，不使用流式接口。
- **中英日韩粤多语言**：SenseVoice 模型支持普通话、英语、日语、韩语、粤语，自动判别。
- **保留两种精度模型**：float32 与 int8 同时存放，可通过参数切换，默认使用 int8。

---

## 1. 工作逻辑

节点启动后的数据流：

```
麦克风(sounddevice/PortAudio)
        │  16 kHz, 单声道, float32
        ▼
Silero VAD  ──检测语音/静音，按停顿切分语音段
        │
        ▼
离线 SenseVoice 识别(OfflineRecognizer)
        │
        ▼
发布 /asr/final_text (std_msgs/msg/String)
```

1. **音频采集**：通过 `sounddevice`（PortAudio）打开输入设备，在独立回调线程中以 512 样本（32 ms）为块读取音频并放入队列；若麦克风原生采样率不是 16 kHz，则在处理线程中线性重采样到 16 kHz。
2. **VAD 检测与分段**：处理线程把音频送入 `VoiceActivityDetector`。
   - 检测到语音后开始累计一段；
   - 当连续静音时长达到 `silence_duration`（默认 **0.5 s**）时，该段语音被判定为一句并放入完成队列；
   - 单段超过 `max_speech_duration`（默认 10 s）会被强制切分；短于 `min_speech_duration`（默认 0.2 s）的声音会被当作噪声忽略。
3. **离线识别**：对每段完成的语音创建 `OfflineStream`，调用 `OfflineRecognizer.decode_stream()` 做整段识别（非流式），默认开启逆文本归一化（ITN，输出标点、数字规范化）。
4. **发布结果**：识别文本去除空白后，以 `std_msgs/String` 发布到 `/asr/final_text`；空结果不发布，仅记录日志。
5. **退出处理**：节点关闭时调用 `vad.flush()` 把缓冲区中最后一段语音取出识别，避免句尾丢失。

---

## 2. 话题

| 方向 | 话题名 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/asr/final_text` | `std_msgs/msg/String` | 每段语音识别完成后的最终文本 |

订阅示例：

```bash
ros2 topic echo /asr/final_text
```

---

## 3. 模型文件

模型统一存放在工作空间的 `src/models` 目录（本功能包本身不包含模型）：

```
src/models/
├── sense-voice-zh-en-ja-ko-yue-2025-09-09/
│   ├── model.onnx          # float32 模型 (约 887 MB)
│   ├── model.int8.onnx     # int8 模型    (约 227 MB)
│   ├── tokens.txt          # 词表
│   └── test_wavs/          # 测试音频 (zh.wav / en.wav)
└── vad/
    └── silero_vad.onnx     # Silero VAD 模型 (约 632 KB)
```

模型版本：**SenseVoice 2025-09-09**，来源为 sherpa-onnx 官方发布（GitHub Releases / HuggingFace）。

模型目录的查找优先级：

1. 参数 `models_dir` 指定的路径；
2. 环境变量 `ASR_MODELS_DIR`；
3. 自动从节点位置向上查找 `<workspace>/src/models`。

如需自行下载：

```bash
# float32（GitHub）
wget https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2025-09-09.tar.bz2
# int8（HuggingFace 国内镜像）
wget https://hf-mirror.com/csukuangfj/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2025-09-09/resolve/main/model.int8.onnx
# VAD
wget https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.onnx
```

---

## 4. 环境依赖

- ROS 2（开发环境为 **Jazzy**，使用系统 Python，**不要使用 conda 环境**）
- Python 3.12（系统环境）
- sherpa-onnx、sounddevice、numpy

安装依赖：

```bash
# PortAudio 运行库
sudo apt update
sudo apt install -y portaudio19-dev

# Python 依赖装入系统环境
pip3 install --break-system-packages sherpa-onnx sounddevice numpy
```

音频权限（如无法打开麦克风）：

```bash
sudo usermod -aG audio $USER   # 重新登录后生效
```

查看可用录音设备：

```bash
arecord -l                 # ALSA 声卡列表（card/device）
python3 -m sounddevice     # sounddevice 设备索引列表
```

---

## 5. 编译

```bash
cd ~/wheeltec/wheeltec_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select asr_sherpa
source install/setup.bash
```

---

## 6. 使用方法

### 6.1 通过 launch 启动（推荐）

```bash
# 使用 launch 中配置的默认参数
ros2 launch asr_sherpa asr_sherpa.launch.py

# 使用 float32 模型
ros2 launch asr_sherpa asr_sherpa.launch.py use_int8:=false

# 指定麦克风
ros2 launch asr_sherpa asr_sherpa.launch.py input_device:="Yundea"
```

launch 文件暴露的参数：`use_int8`、`silence_duration`、`language`、`input_device`、`models_dir`。

### 6.2 直接运行节点

```bash
# 默认参数（int8 / 4 线程 / 系统默认麦克风）
ros2 run asr_sherpa asr_node

# 指定麦克风、float32 模型
ros2 run asr_sherpa asr_node --ros-args \
  -p use_int8:=false \
  -p input_device:="Yundea"

# 指定模型目录
ros2 run asr_sherpa asr_node --ros-args -p models_dir:=/opt/models
```

### 6.3 指定麦克风（input_device）

参数同时支持**整数索引**与**设备名字符串**（子串匹配，不区分大小写）；不指定时使用系统默认输入设备。

| 写法 | 含义 |
|---|---|
| `input_device:=6` | sounddevice 索引 6 |
| `input_device:="Yundea"` | 名称中含 "Yundea" 的设备 |
| `input_device:="hw:2,0"` | 对应 ALSA card 2, device 0 |
| `input_device:="USB Audio"` | 名称中含 "USB Audio" 的设备 |
| 不指定 / `-1` / `''` | 系统默认输入设备 |

> 建议使用设备名（如 `"Yundea"` 或 `"hw:2,0"`），重启后设备索引可能变化，名称更稳定。节点启动时会打印 `Listening on device: ...`，可据此确认所选设备。名称若匹配到 0 个或多个设备会报错并列出可用设备。

### 6.4 查看识别结果

```bash
ros2 topic echo /asr/final_text
```

---

## 7. 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `use_int8` | bool | `true` | `true` 使用 int8 模型；`false` 使用 float32 |
| `num_threads` | int | `4` | ONNX Runtime 推理线程数 |
| `language` | string | `auto` | 语言：`auto`/`zh`/`en`/`ja`/`ko`/`yue` |
| `use_itn` | bool | `true` | 逆文本归一化（标点、数字规范化） |
| `silence_duration` | double | `0.5` | 用户停顿多少秒后切分一句（秒） |
| `vad_threshold` | double | `0.5` | VAD 语音判定阈值，越低越灵敏 |
| `min_speech_duration` | double | `0.2` | 短于该时长的声音被忽略（秒） |
| `max_speech_duration` | double | `10.0` | 单段最长时长，超过强制切分（秒） |
| `input_device` | int/string | 默认设备 | 麦克风索引或设备名，见 6.3 |
| `mic_sample_rate` | int | `16000` | 麦克风采样率；非 16k 时自动重采样 |
| `models_dir` | string | `''` | 模型根目录，为空时自动查找 |

> 注意：launch 文件目前只暴露了部分参数；`num_threads`、`use_itn`、`vad_threshold`、`min_speech_duration`、`max_speech_duration`、`mic_sample_rate` 需用 `ros2 run ... --ros-args -p` 方式设置。

调参建议：

- **安静环境**：`vad_threshold:=0.5`、`silence_duration:=0.5`；
- **嘈杂环境**：`vad_threshold:=0.6`（更严格）、`silence_duration:=0.8`（避免句中停顿被误切）；
- **只说中文**：`language:=zh`；
- **追求速度 / 嵌入式部署**：`use_int8:=true`；追求最高精度：`use_int8:=false`。

---

## 8. 功能包结构

```
src/asr_sherpa/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│   └── asr_sherpa
├── asr_sherpa/
│   ├── __init__.py
│   └── asr_node.py            # 核心识别节点
└── launch/
    └── asr_sherpa.launch.py
```

---

## 9. 部署到 NUC11 i5 等 x86 主机

x86 主机可直接使用本功能包，建议配置：

```bash
ros2 launch asr_sherpa asr_sherpa.launch.py \
  use_int8:=true \
  input_device:="Yundea"
```

- int8 模型体积小、推理快，Intel 11 代 CPU 支持 VNNI 指令，int8 有硬件加速；
- `num_threads` 建议设为物理核数（i5-1135G7 可设 4）；
- 部署时需在目标机安装第 4 节的依赖，并拷贝 `src/models` 整个目录；
- 目标机声卡不同时，先用 `arecord -l` / `python3 -m sounddevice` 确认设备，再用 `input_device` 指定。

---

## 10. 常见问题

1. **启动报 `Required model file not found`**：模型未下载或路径不对，确认 `src/models` 目录结构，或用 `models_dir` / `ASR_MODELS_DIR` 指定。
2. **没有识别结果**：
   - 确认麦克风设备（`arecord -l`），用 `input_device` 显式指定；
   - 适当调低 `vad_threshold`（如 0.3）提高灵敏度；
   - 说话后停顿至少 `silence_duration`（0.5 s）才会触发识别。
3. **频繁误切 / 噪声触发**：调高 `vad_threshold`、`silence_duration` 或 `min_speech_duration`。
4. **`ModuleNotFoundError: sherpa_onnx`**：确认在系统 Python 中安装（第 4 节），不要切换到 conda 环境。
5. **麦克风采样率不支持 16 kHz**：设置 `mic_sample_rate` 为设备原生采样率（如 48000），节点会自动重采样。
