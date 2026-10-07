# speech_controller

文字（语音）命令 -> 小车 / 机械臂动作控制的 ROS 2 功能包。

说明，只启动speech_control_all.launch.py不能控制机械臂，而应该以下面的方式启动：
# 方式一（推荐）：直接用 pick_place，它本来就是"整车+语音"的组合
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py use_speech:=true

# 方式二：先起机械臂整车栈（不含 pick_place 业务节点时 gripper action 有、
# 但 arm_stop/arm_home 服务没有），再叠加语音栈且不要重复起底盘
ros2 launch wheeltec_mycobot_bringup all.launch.py use_move_group:=true
ros2 launch speech_controller speech_control_all.launch.py bringup_robot:=false



订阅 `/asr_command` 上的标准命令文本，根据 **YAML 配置**把命令分发到三类执行端：

1. **底盘命令**（如 "小车向前"）：转换为 `geometry_msgs/Twist` 发布到 `/cmd_vel`；
2. **夹爪命令**（"夹爪松开" / "夹爪闭合"）：向 `/gripper_action_controller/gripper_cmd`
   发送 `control_msgs/GripperCommand` action 目标；
3. **机械臂整机命令**（"机械臂停止" / "机械臂回家"）：调用 grasp_node 的
   `std_srvs/Trigger` 服务（`/pick_place/arm_stop`、`/pick_place/arm_home`）。

- 每条命令对应的速度、夹爪位置、服务名**全部由 YAML 文件定义**，改动作只改配置、不改代码；
- 新增命令只需在配置中添加条目（文字须与上游 `asr_keywords` 输出的标准命令一致）；
- 支持使用外部配置文件（调整后无需重新编译）；
- 节点启动时先发一条零速度命令，作为安全保险；
- 夹爪 action / 臂服务尚未就绪时（如只启动了底盘栈），对应命令只告警、不产生动作。

---

## 1. 工作逻辑

```
/asr_command (std_msgs/String)
        │  标准命令文本，如 "小车向前" / "夹爪闭合" / "机械臂回家"
        ▼
从 YAML 加载的三张映射表，按顺序查找
        │
        ├─ 命中 commands          -> 发布 Twist 到 /cmd_vel（底盘）
        ├─ 命中 gripper_commands  -> 发 GripperCommand action 目标（夹爪）
        ├─ 命中 service_commands  -> 异步调用对应 Trigger 服务（机械臂整机）
        └─ 都未命中               -> 告警 Unknown command，不改变当前运动
```

1. 节点启动时从 YAML 读取 `commands` / `gripper_commands` / `service_commands`
   三张映射表，并预先创建发布器、夹爪 action 客户端和各 Trigger 服务客户端；
2. 订阅 `/asr_command`，每收到一条文本就依次查三张表；
3. 底盘命令发布 Twist；夹爪命令异步发 action 目标（结果经回调日志反馈，
   `reached_goal` 与 `stalled`——夹住苹果停滞——均视为成功）；
   机械臂整机命令异步调用 Trigger 服务；
4. 三类动作均不阻塞节点 executor；目标服务/action 不在线时只打印告警。

---

## 2. 接口

### 2.1 话题

| 方向 | 话题名 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/asr_command` | `std_msgs/msg/String` | 输入文本命令 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度命令 |

### 2.2 动作 / 服务（客户端）

| 类型 | 名称 | 接口 | 对应命令 |
|---|---|---|---|
| action | `/gripper_action_controller/gripper_cmd` | `control_msgs/GripperCommand` | 夹爪松开 / 夹爪闭合 |
| service | `/pick_place/arm_stop` | `std_srvs/Trigger` | 机械臂停止（急停） |
| service | `/pick_place/arm_home` | `std_srvs/Trigger` | 机械臂回家（回 home 零位） |

> 夹爪 action 与两个臂服务都由 `pick_place.launch.py` 启动的整车/机械臂栈提供；
> 仅启动底盘语音栈时这些命令不会生效（日志提示 not available）。

---

## 3. 命令配置（YAML）

默认配置文件：[config/commands.yaml](config/commands.yaml)，含三个可选配置段。

### 3.1 `commands`：底盘速度

每个命令下指定 `linear`（线速度，m/s）与 `angular`（角速度，rad/s）的 `x / y / z` 分量，未写的分量按 0 处理：

```yaml
commands:
  小车向前:
    linear:  {x: 0.05, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}

  小车停止:
    linear:  {}      # 全部为 0
    angular: {}
```

### 3.2 `gripper_commands`：夹爪位置（rad）

命令文字 -> 夹爪关节目标位置，经 GripperCommand action 下发。URDF 关节范围
`[-0.7, 0.15] rad`（对应 pymycobot 0..100）：

```yaml
gripper_commands:
  夹爪松开: 0.15    # 最大松开（value 100，夹爪全开）
  夹爪闭合: -0.42   # 闭合到张开约 1/3（value 33，抓取苹果的闭合值）
```

### 3.3 `service_commands`：机械臂整机 Trigger 服务

命令文字 -> 服务名（由 grasp_node 提供）：

```yaml
service_commands:
  机械臂停止: /pick_place/arm_stop   # 急停：取消在途 MoveGroup goal + 固件 stop
  机械臂回家: /pick_place/arm_home   # 6 关节回到 home 零位
```

### 默认命令一览

| 文本命令 | 执行动作 |
|---|---|
| 小车向前 | linear.x = +0.05 m/s |
| 小车向后 | linear.x = -0.05 m/s |
| 小车左转 | angular.z = +0.1 rad/s |
| 小车右转 | angular.z = -0.1 rad/s |
| 小车停止 | 零速度 Twist |
| 夹爪松开 | gripper position = +0.15 rad（value 100，全开） |
| 夹爪闭合 | gripper position = -0.42 rad（value 33，约 1/3 张开） |
| 机械臂停止 | 调 `/pick_place/arm_stop`（立即停止一切臂部运动） |
| 机械臂回家 | 调 `/pick_place/arm_home`（回 home 零位） |

> 语音侧能识别的同义说法（手松开/手张开/手抓住/手抓紧、手回家/手停止/手别动、
> 机器臂、机械手、手臂 等）定义在 `asr_keywords` 包的
> `config/keywords.yaml`，识别后统一输出上表的标准命令文字。
> 多音字/同音字/声调差异由拼音相似度自动模糊匹配。

### 符号约定（ROS 标准）

- `linear.x`：正 = 前进，负 = 后退；
- `angular.z`：正 = 逆时针左转，负 = 顺时针右转。

> 若实测小车转向相反，把配置中左转/右转的 `angular.z` 符号互换（或整体翻转）即可。

### 修改 / 新增命令

- 直接编辑 [config/commands.yaml](config/commands.yaml)，保存后**重新编译**生效（配置会安装到 `install/`）；
- 也可通过 `commands_file` 参数指向**外部 YAML 文件**，改动后无需编译（见第 5、6 节）；
- 新增命令时，文字要与上游 `asr_keywords` 输出的标准命令一致。

---

## 4. 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `commands_file` | string | `''` | 命令 YAML 路径；为空时使用包内 `config/commands.yaml` |
| `command_topic` | string | `/asr_command` | 输入文本命令话题 |
| `cmd_vel_topic` | string | `/cmd_vel` | 输出 Twist 话题 |
| `gripper_action` | string | `/gripper_action_controller/gripper_cmd` | 夹爪 GripperCommand action 名 |
| `gripper_max_effort` | float | `50.0` | 夹爪动作 max_effort |

---

## 5. 环境依赖与编译

- ROS 2（开发环境 **Jazzy**，系统 Python，不使用 conda）
- Python 依赖：`pyyaml`

安装依赖：

```bash
pip3 install --break-system-packages pyyaml
```

编译：

```bash
cd ~/wheeltec/wheeltec_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select speech_controller
source install/setup.bash
```

---

## 6. 使用方法

### 6.1 launch 启动（推荐）

#### 一键启动整条语音控制链路

使用总启动文件，**同时启动小车（turn_on_wheeltec_robot）+ asr_sherpa + asr_keywords + speech_controller**，一条命令完成从底盘上电、麦克风识别到小车运动：

```bash
ros2 launch speech_controller speech_control_all.launch.py
```

可传入全部参数：

```bash
ros2 launch speech_controller speech_control_all.launch.py \
  use_int8:=true \
  input_device:="Yundea" \
  threshold:=0.6
```

总启动文件暴露的参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `carto_slam` | `false` | 转发给小车 launch，为 `true` 时不启动 EKF 节点 |
| `use_int8` | `true` | ASR 使用 int8 / float32 模型 |
| `silence_duration` | `0.5` | 停顿切分秒数 |
| `language` | `auto` | 识别语言 |
| `input_device` | `Yundea` | 麦克风（索引或名称） |
| `models_dir` | `''` | 模型目录 |
| `threshold` | `0.6` | 命令匹配阈值 |
| `keywords_file` | `''` | 外部关键字配置 |
| `commands_file` | `''` | 外部命令速度配置 |
| `gripper_action` | `/gripper_action_controller/gripper_cmd` | 夹爪 action 名 |

> 小车底盘节点需要串口硬件；请在真实小车上运行。若串口未连接，底盘节点会报 `can not open serial port` 并退出（其余语音节点不受影响）。

#### 仅启动本包

```bash
# 使用包内默认配置
ros2 launch speech_controller speech_controller.launch.py

# 使用外部命令配置文件（调整速度无需编译）
ros2 launch speech_controller speech_controller.launch.py \
  commands_file:=/path/to/my_commands.yaml

# 自定义话题
ros2 launch speech_controller speech_controller.launch.py \
  command_topic:=/asr_command cmd_vel_topic:=/cmd_vel
```

### 6.2 直接运行节点

```bash
ros2 run speech_controller speech_controller_node --ros-args \
  -p commands_file:=/path/to/my_commands.yaml
```

### 6.3 在整条语音链路中的位置

```
麦克风
  └─ asr_sherpa        -> /asr/final_text   （语音转文字）
       └─ asr_keywords -> /asr_command      （文字模糊匹配为标准命令）
            └─ speech_controller            <-- 本包，按命令类型分发
                 ├─ /cmd_vel                             （底盘速度）
                 ├─ /gripper_action_controller/gripper_cmd（夹爪 action）
                 └─ /pick_place/arm_stop | arm_home       （机械臂 Trigger 服务）
```

> 与整车栈一起使用时由 `pick_place.launch.py`（`use_speech:=true`）统一拉起；
> 单独用 `speech_control_all.launch.py` 只启动底盘栈，机械臂类命令会提示
> 服务/action 不可用。

手动发命令测试：

```bash
# 底盘
ros2 topic pub --once /asr_command std_msgs/msg/String "{data: '小车向前'}"
ros2 topic echo /cmd_vel
# 夹爪（机械臂栈运行时，会真实驱动夹爪）
ros2 topic pub --once /asr_command std_msgs/msg/String "{data: '夹爪松开'}"
# 机械臂停止 / 回家
ros2 topic pub --once /asr_command std_msgs/msg/String "{data: '机械臂回家'}"
ros2 topic pub --once /asr_command std_msgs/msg/String "{data: '机械臂停止'}"
```

---

## 7. 验证结果

- 5 条底盘命令实测发布的 Twist 正确（前进 +0.050、后退 -0.050、左转 +0.100、右转 -0.100、停止 0.000）；
- 外部配置文件验证通过（如把前进速度改为 0.2，节点正确加载并输出 `linear.x=0.200`）；
- 真机验证：`夹爪松开` -> pymycobot value 100（全开），`夹爪闭合` -> value 33
  （夹爪实际位置 -0.411 rad，约 1/3 张开）；
- 真机验证：`机械臂回家` 调 `/pick_place/arm_home` 臂回零位；`机械臂停止`
  在臂运动中调用可立即取消在途规划并固件急停；
- ASR 模糊匹配离线回归：机械臂/机器臂/机械手/手臂/手 等说法、同音字
  （如"机诫臂停只"）与声调差异均正确命中，且"小车停止"不会被"机械臂停止"
  类说法误触发，"把苹果抓过来"仍命中小车流水线。

---

## 8. 功能包结构

```
src/speech_controller/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/
│   └── speech_controller
├── config/
│   └── commands.yaml                  # 底盘 Twist + 夹爪位置 + 臂服务 三段命令配置
├── launch/
│   ├── speech_controller.launch.py        # 仅启动控制节点
│   └── speech_control_all.launch.py       # 一键启动 小车+asr_sherpa+asr_keywords+speech_controller
└── speech_controller/
    ├── __init__.py
    └── speech_controller_node.py       # 核心节点（Twist / GripperCommand / Trigger 三路分发）
```

核心函数（`speech_controller_node.py`）：

| 函数 | 作用 |
|---|---|
| `load_command_velocities()` | 读取 `commands`，构建命令 -> Twist 映射 |
| `load_gripper_commands()` | 读取 `gripper_commands`，构建命令 -> 夹爪 rad 映射 |
| `load_service_commands()` | 读取 `service_commands`，构建命令 -> Trigger 服务名映射 |
| `_send_gripper_goal()` | 异步发送夹爪 action 目标（回调链不阻塞 executor） |
| `_call_service_command()` | 异步调用机械臂 Trigger 服务 |
| `default_commands_file()` | 返回包内默认配置路径 |

---

## 9. 如何新增一条语音命令

1. 在 `asr_keywords/config/keywords.yaml` 中确定标准命令文字与同义词
   （多音字/模糊音由拼音相似度兜底，注意避免与已有命令的词组冲突，
   可用 `best_match_score` 离线打分回归）；
2. 在本包 `config/commands.yaml` 按执行端选择配置段：
   - 底盘运动 -> `commands`（Twist）；
   - 夹爪开合 -> `gripper_commands`（rad 位置）；
   - 其他机械臂能力（已有 Trigger 服务）-> `service_commands`（服务名）；
3. 重新 `colcon build --packages-select asr_keywords speech_controller`
   （或用 `keywords_file` / `commands_file` 外部配置免编译）。

> 需要全新执行接口（如 FollowJointTrajectory action、自定义服务）时，
> 在节点中增加一个客户端与一个配置段即可，命令分发顺序保持
> 底盘 -> 夹爪 -> 服务 -> Unknown。

---

## 10. 常见问题

1. **小车不动**：确认节点已启动、话题连通（`ros2 topic info /cmd_vel`），并确认命令文字与配置中的键完全一致。
2. **说了夹爪/机械臂命令只打印 not available**：机械臂栈（move_group / ros2_control / grasp_node / mycobot_driver）没有启动，或命令发出时服务尚未就绪；`speech_control_all.launch.py` 单独使用时只带底盘，不含机械臂。
3. **改了 YAML 不生效**：源码配置需重新 `colcon build`；或改用 `commands_file` 指向外部文件。
4. **转向方向相反**：互换配置中左/右转 `angular.z` 的正负号。
5. **报 `Commands file not found`**：检查 `commands_file` 路径；不传该参数使用包内默认配置。
6. **速度太快/太慢**：直接在 YAML 中调整 `linear.x` / `angular.z` 的数值。
7. **语音说了但没匹配到标准命令**：检查 `asr_keywords` 节点日志中识别原文与打分；必要时在 `keywords.yaml` 增补同义词。注意"停止/停"这类裸词只属于小车，停止机械臂请说"机械臂停止/手停止/手别动"。
