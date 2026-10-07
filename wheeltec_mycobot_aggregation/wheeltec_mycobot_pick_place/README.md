# wheeltec_mycobot_pick_place

基于语音指令的苹果抓取运送系统，适用于 **WHEELTEC R680 移动底盘 + myCobot 280 机械臂**
聚合机器人。

> 收到语音指令"把苹果拿过来"后，系统自动完成：
> 识别苹果 → 小车靠近苹果 → 机械臂抓取 → 小车运送 → 放置 → 返回起点。

---

## 一、系统架构

### 1.1 节点一览

| 节点 | 功能 |
|------|------|
| `voice_command_node` | 监听 `/asr_command`，按指令表匹配（"把苹果拿过来"等）后触发流程，支持多指令热重载 |
| `apple_detection_node` | YOLO26（ONN X 端侧部署）检测苹果，结合深度图输出 3D 位姿 |
| `navigation_node` | Nav2 导航（靠近苹果 / 运送 / 返回），动态避障 |
| `grasp_node` | MoveIt2 抓取与放置（MTC 风格分阶段） |
| `coordinator_node` | 状态机，串联整个流水线 |

### 1.2 整体数据流

```
┌──────────────────────────────────────────────────────────────────────┐
│                         语音指令触发                                  │
│  麦克风 → asr_sherpa → /asr/final_text → asr_keywords → /asr_command │
│                                  │                                   │
│                          voice_command_node                          │
│                                  ▼                                   │
│                         /pick_place/trigger                          │
└──────────────────────────────┬───────────────────────────────────────┘
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                       coordinator_node (状态机)                       │
│                                                                      │
│  IDLE → DETECTING → NAV_TO_APPLE → GRASPING → NAV_TO_DELIVERY        │
│        → PLACING → NAV_HOME → DONE                                   │
└──────┬──────────┬──────────────┬─────────────┬───────────────┬──────┘
       │          │              │             │               │
       ▼          ▼              ▼             ▼               ▼
  detect_apple  navigate_near   pick         navigate       place
       │          │              │             │               │
       ▼          ▼              ▼             ▼               ▼
  apple_detection  navigation  grasp_node  navigation    grasp_node
  _node           _node                      _node         (release)
```

### 1.3 各阶段详细数据流

```
【识别阶段】
  /camera/color/image_raw ─┐
  /camera/depth/image_raw ─┼─→ apple_detection_node (YOLO26 ONNX + 深度)
  /camera/depth/camera_info┘                    │
                                                 ▼
                                    /pick_place/apple_pose (base_link)

【靠近苹果阶段】
  /pick_place/apple_pose → coordinator → /pick_place/nav_object
                                              │
                                              ▼
                              navigation_node._compute_approach_pose()
                              (计算车头朝向苹果、距离 0.35m 的目标位姿)
                                              │
                                              ▼
                                    Nav2 BasicNavigator.goToPose()
                                    (局部代价地图融合激光雷达+点云 → 动态避障)

【抓取阶段】
  /pick_place/apple_pose → grasp_node
  MoveIt2 PlanningComponent 分阶段:
    open gripper → pre-grasp → approach → close gripper
    → attach object → lift

【运送阶段】
  delivery_pose = start_pose + (0.5, 0.5)
  → /pick_place/nav_goal → navigation_node → Nav2 导航

【放置阶段】
  /pick_place/place_pose → grasp_node
    pre-place → lower → open gripper → detach → retreat

【返回阶段】
  /pick_place/go_home → navigation_node → Nav2 返回起点
```

---

## 二、安装与依赖

### 2.1 系统依赖

| 依赖 | 版本/说明 | 安装方式 |
|------|-----------|----------|
| ROS 2 | Jazzy | 系统已安装 |
| MoveIt2 | 随 ROS2 Jazzy | `apt install ros-jazzy-moveit` |
| Nav2 | 随 ROS2 Jazzy | `apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup` |
| ultralytics (YOLO, 仅 ultralytics 后端调试用) | 8.4.26 | `pip install ultralytics` |
| onnxruntime (YOLO26 端侧推理) | 1.26.0 | `pip install onnxruntime`（GPU 用 `onnxruntime-gpu`） |
| torch (仅 YOLO26 训练，conda env: yolov26) | 2.12.1+cu130 | 见 `~/YOLO/yolov26/使用说明.md` |
| opencv-python | 4.6.0 | `pip install opencv-python` |
| pypinyin | 可选（ASR拼音匹配） | `pip install pypinyin` |
| cv_bridge | ROS2 | `apt install ros-jazzy-cv-bridge` |
| tf2_geometry_msgs | ROS2 | `apt install ros-jazzy-tf2-geometry-msgs` |

### 2.2 工作空间依赖包

本包依赖以下已有工作空间包（均在 `wheeltec_ws/src` 中）：

- `wheeltec_mycobot_bringup` — 聚合机器人设备启动
- `wheeltec_mycobot_description` — 聚合机器人 URDF
- `mycobot_moveit_config` — 机械臂 MoveIt2 配置（复用）
- `mycobot280_moveit2_control` — 机械臂 ros2_control 硬件接口
- `speech_controller`、`asr_sherpa`、`asr_keywords` — 语音识别栈
- `wheeltec_nav2` — Nav2 导航配置
- `astra_camera`、`lslidar_driver` — 相机与激光雷达驱动
- `turn_on_wheeltec_robot` — 底盘驱动

### 2.3 模型文件

YOLO26 检测模型（ONNX，端侧部署用）：
```
wheeltec_wheeltec_ws/src/models/yolo26n.onnx
```
由 `~/YOLO/yolov26` 源码（conda env `yolov26`）导出，导出命令见
[02_apple_detection.md](docs/02_apple_detection.md)。

ASR 模型位于：
```
wheeltec_wheeltec_ws/src/models/sense-voice-zh-en-ja-ko-yue-2025-09-09/
wheeltec_wheeltec_ws/src/models/vad/
```

### 2.4 构建

```bash
cd ~/wheeltec/wheeltec_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wheeltec_mycobot_pick_place --symlink-install
source install/setup.bash
```

---

## 三、使用方法

### 3.1 模拟模式（无需任何硬件，推荐先测试）

```bash
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py sim_mode:=true
```

触发流程（任选其一）：
```bash
# 方法1：手动触发服务
ros2 service call /pick_place/trigger_manual std_srvs/srv/Trigger

# 方法2：发布模拟语音指令
ros2 topic pub --once /asr_command std_msgs/msg/String "{data: '把苹果拿过来'}"
```

监控状态：
```bash
ros2 topic echo /pick_place/state
```

预期状态序列：
`IDLE → DETECTING → NAV_TO_APPLE → GRASPING → NAV_TO_DELIVERY → PLACING → NAV_HOME → DONE`

### 3.2 真实模式

**前置条件：**
- 插入 WHEELTEC 底盘 USB、myCobot 机械臂 USB（默认 `/dev/ttyACM0`）
- 接入 Astra 相机、LSLidar 激光雷达
- 连接麦克风

```bash
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py sim_mode:=false use_rviz:=true
```

启动后：
1. 等待所有设备节点就绪（`all.launch.py` 统一启动底盘、相机、雷达、机械臂、MoveIt2）
2. 语音栈自动启动（`speech_control_all.launch.py`）
3. 对着麦克风说 **"把苹果拿过来"** 即可触发完整流程

> 若某设备未接入，对应节点会报错但不影响其他节点启动，系统会提示需要插入的设备。

---

## 四、模拟/真实模式切换

所有节点通过全局参数 `sim_mode` 控制：

| `sim_mode` | 行为 |
|------------|------|
| `true` | 各节点使用模拟数据：固定苹果位姿、导航延时模拟成功、抓取延时模拟成功。无需任何硬件。 |
| `false` | 订阅真实传感器话题，调用 Nav2 / MoveIt2 真实执行。需硬件设备。 |

参数在 `config/pick_place_params.yaml` 中配置，也可通过 launch 参数 `sim_mode:=true/false` 覆盖。

---

## 五、各节点详细文档

- [01_voice_command.md](docs/01_voice_command.md) — 语音指令节点（含多指令框架）
- [02_apple_detection.md](docs/02_apple_detection.md) — 苹果识别节点（YOLO26 端侧部署）
- [03_navigation.md](docs/03_navigation.md) — 导航节点
- [04_grasp.md](docs/04_grasp.md) — 抓取节点
- [05_coordinator.md](docs/05_coordinator.md) — 协调节点

---

## 六、如何添加新的语音指令（代码框架说明）

语音指令采用 **指令表 + 动作分发** 框架（`config/voice_commands.yaml`），
添加新指令是纯配置任务，无需修改任何代码：

```yaml
commands:
  - name: pick_banana                 # 1. 指令唯一名
    patterns: ["把香蕉拿过来"]         # 2. 触发文本（可多个别名）
    match_mode: exact                 # 3. exact=精确 | contains=模糊包含
    enabled: true
    actions:                          # 4. 触发后依次执行的动作
      - type: topic                   #    动作A：发布话题（动态消息类型）
        topic: /pick_place/trigger
        msg_type: std_msgs/Bool
        data: true
      - type: service                 #    动作B（可选）：调用 Trigger 服务
        service: /pick_place/arm_home
```

生效方式（二选一）：
```bash
# 热重载，无需重启
ros2 service call /pick_place/reload_commands std_srvs/srv/Trigger
# 或重启 launch
```

验证：
```bash
ros2 service call /pick_place/list_commands std_srvs/srv/Trigger
ros2 topic pub --once /asr_command std_msgs/msg/String "{data: '把香蕉拿过来'}"
```

**扩展点**（如需新动作类型，改 [voice_command_node.py](wheeltec_mycobot_pick_place/voice_command_node.py)
的 `_dispatch_action()`）：
- 新话题消息类型：已支持所有含 `data` 字段的消息（动态 import，指令表直接写 `msg_type`）
- 非 Trigger 服务：在 `_dispatch_action()` 增加 srv 分支
- 新动作类型（如 TTS 播报、参数化抓取物体）：增加 `elif` 分支即可，
  指令表格式保持不变

> 注意：若语音栈 asr_keywords 未收录新关键词，还需将其加入
> `speech_controller/config/commands.yaml`（speech 包配置，本包不改）。

---

## 七、配置说明

核心配置文件：
- `config/pick_place_params.yaml` — 节点参数（sim_mode、导航、检测、抓取）
- `config/voice_commands.yaml` — 语音指令表（见第六节）

关键参数：
- `approach_distance`: 小车靠近苹果的距离（默认 0.35m）
- `delivery_offset_x/y`: 送达点相对起点的偏移（默认 0.5m, 0.5m）
- `model_type` / `model_path`: YOLO26 ONNX 模型（端侧部署）
- `confidence_threshold`: YOLO 检测置信度阈值（默认 0.4）
- `sim_apple_pose`: 模拟模式下的苹果位姿

---

## 八、MoveIt2 配置说明

聚合机器人 `wheeltec_mycobot` 的机械臂部分与独立 mycobot280 关节完全一致，
因此直接复用 `mycobot_moveit_config`（arm 组、gripper 组、命名位姿 home/open/half_closed）。
`all.launch.py` 已集成该 move_group 启动，无需额外创建
wheeltec 专用 MoveIt config。

如需为移动底盘+机械臂创建完整的 mobile manipulator MoveIt config（将底盘
虚拟关节纳入规划组），可使用 MoveIt Setup Assistant 基于
`wheeltec_mycobot.urdf.xacro` 生成，但本项目采用 Nav2（底盘）+ MoveIt2（臂）
分离规划的标准方案，无需此配置。

---

## 九、故障排查

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 节点启动报 params file 解析错误 | YAML 格式缺少 `ros__parameters` | 确认 config 以 `/**: ros__parameters:` 开头 |
| 抓取节点报 MoveIt 导入失败 | MTC core 命名空间遮蔽 | 节点已自动 prepend 系统 moveit 路径 |
| 导航失败 `map frame does not exist` | 未启动 Nav2/定位 | 真实模式需启动 `wheeltec_nav2` bringup |
| YOLO 未检测到苹果 | 置信度阈值过高或模型未加载 | 调低 `confidence_threshold`，检查模型路径与 `target_class_id` |
| onnxruntime 加载失败 | 未安装或模型文件缺失 | `pip install onnxruntime`，确认 `src/models/yolo26n.onnx` |
| 语音指令未生效 | 指令未在指令表或未 reload | `list_commands` 查看，`reload_commands` 重载 |
| ASR 无响应 | 麦克风未接入或模型未下载 | 检查 `input_device` 参数，确认 models 目录 |
