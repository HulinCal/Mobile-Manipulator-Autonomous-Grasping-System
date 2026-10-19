# 6. 语音控制小车（"小车前进"等底盘指令）处理流程

## 功能概述

说一句"小车前进"，小车即以 0.05 m/s 向前运动。该功能由 **speech_controller 包**
（`src/speech_controller/`）实现，与 pick_place 的抓取触发指令共用同一条语音识别
链路，但在最后一级**分表分发**：抓取指令走 `voice_command_node`，底盘/夹爪/机械臂
整机指令走 `speech_controller_node`。

## 端到端数据流

```
麦克风
  → asr_sherpa        离线 ASR(SenseVoice)，静音 0.5s 切分语音段
    话题: /asr/final_text        (std_msgs/String, 如 "请小车前进")
  → asr_keywords      拼音+字符模糊匹配，归一化为标准命令
    话题: /asr_command           (std_msgs/String, 标准命令 "小车向前")
  → speech_controller 查 commands 表
    话题: /cmd_vel               (geometry_msgs/Twist, linear.x=+0.05)
  → 底盘驱动链        cmd_vel_to_ackermann_drive → /ackermann_cmd
    → 底盘串口桥 → MCU → 电机
```

同时 `voice_command_node`（pick_place）也在订阅 `/asr_command`，但它的指令表里
只有"把苹果拿过来"等抓取指令，对"小车向前"不匹配、直接忽略——**两个订阅者各查
各的表，互不干扰**。

## 分步流程（以"小车前进"为例）

### 第 1 步：语音识别（asr_sherpa）

- 麦克风采集（默认设备名含 `Yundea`），SenseVoice 模型离线推理；
- 检测到 0.5s（`silence_duration` 参数）静音即认为一句话结束；
- 输出最终识别文本到 `/asr/final_text`，例如 `"请小车前进"`（带不带敬语均可，
  后续匹配对整句做滑窗比较）。

### 第 2 步：关键词匹配与归一化（asr_keywords/keyword_node.py）

**命令组配置**（`asr_keywords/config/keywords.yaml`）：

```yaml
- command: 小车向前                      # 标准命令（发布出去的文字）
  synonyms: [小车前进, 小车往前, 向前, 前进, 往前走, 往前行, 往前开, 一直向前, 直行]
```

注意：用户说"**小车前进**"，发布的标准命令是"**小车向前**"——同义词命中后
统一归一化为组名 `command`，下游只认标准命令。

**匹配算法**（`best_match_score`）：

1. `normalize()`：去除标点和空白（中英文标点全覆盖）；
2. 若短语**完整包含**于识别文本（如"请小车前进"包含"小车前进"），直接得分 1.0；
3. 否则取**字符相似度**（difflib.SequenceMatcher）与**拼音相似度**（pypinyin
   按音节级比较，容错同音/近音字如"前/钱"）两者的最大值；
4. 比较窗口长度限定为短语长度和短语长度+1 的滑动窗口，防止短窗口虚高分数。

**选择规则**：遍历所有命令组的所有短语，取最高分组；**同分时命中短语更长者优先**
（防止裸词"停止"抢占"小车停止"）；阈值 `threshold=0.6`（launch 参数可调），
达标即把标准命令发布到 `/asr_command`，未达标只打日志不发布。

### 第 3 步：命令分发（speech_controller/speech_controller_node.py）

`/asr_command` 回调里按以下顺序查三张表（全部来自 `config/commands.yaml`）：

| 顺序 | 表 | 命中后动作 | 本例 |
|------|-----|-----------|------|
| 1 | `commands`（底盘） | 发布一条 `geometry_msgs/Twist` 到 `/cmd_vel` | ✅ 命中 |
| 2 | `gripper_commands`（夹爪） | 发 GripperCommand action 给 `/gripper_action_controller/gripper_cmd` | — |
| 3 | `service_commands`（整机） | 异步调用 `std_srvs/Trigger` 服务 | — |
| 4 | 均未命中 | warn `Unknown command` | — |

**commands.yaml 中"小车向前"的配置**：

```yaml
commands:
  小车向前:
    linear:  {x: 0.05, y: 0.0, z: 0.0}   # 前进 0.05 m/s
    angular: {x: 0.0, y: 0.0, z: 0.0}    # 不转向
```

底盘指令是**单次发布**一条 Twist（非持续流）。节点启动时会先发一条全零 Twist
保证安全。速度 0.05 m/s 与抓取导航限速一致（狭小测试环境约束）。

**符号约定**（ROS 标准）：`linear.x` 正=前进负=后退；`angular.z` 正=逆时针左转
负=顺时针右转。

### 第 4 步：底盘执行（turn_on_wheeltec_robot 栈）

- `cmd_vel_to_ackermann_drive.py` 订阅 `/cmd_vel`，把 Twist（线速度+角速度）按
  轴距换算为转向角，发布 `AckermannDriveStamped` 到 `/ackermann_cmd`；
- 底盘串口桥节点（`base_serial.launch.py` 启动，话题映射
  `akm_cmd_vel: ackermann_cmd`）经串口下发到 MCU，驱动电机执行。

## 完整指令集（speech_controller 当前配置）

| 语音说法（部分同义词） | 标准命令 | 动作 |
|---|---|---|
| 小车前进/往前/直行… | 小车向前 | `/cmd_vel` linear.x=+0.05 |
| 小车后退/倒车… | 小车向后 | `/cmd_vel` linear.x=-0.05 |
| 小车左转/左拐… | 小车左转 | `/cmd_vel` angular.z=+0.1 |
| 小车右转/右拐… | 小车右转 | `/cmd_vel` angular.z=-0.1 |
| 小车停下/刹车/停… | 小车停止 | `/cmd_vel` 全零 |
| 手松开/张开… | 夹爪松开 | 夹爪 action → 0.15 rad（全开） |
| 手抓住/夹住… | 夹爪闭合 | 夹爪 action → -0.42 rad（张开 1/3） |
| 机械臂停止/手停止… | 机械臂停止 | 调 `/pick_place/arm_stop`（急停） |
| 机械臂回家/手回家… | 机械臂回家 | 调 `/pick_place/arm_home`（回零位） |

## 测试方法

```bash
# 跳过 ASR 直接注入标准命令（验证第 3 步之后链路）：
ros2 topic pub --once /asr_command std_msgs/msg/String "{data: '小车向前'}"

# 观察速度指令：
ros2 topic echo /cmd_vel

# 注入同义词（验证第 2 步匹配，需语音栈在跑）：
ros2 topic pub --once /asr/final_text std_msgs/msg/String "{data: '请小车前进'}"
# 日志应显示: "请小车前进" -> 小车向前 (score=1.00)

# 真机：对麦克风说"小车前进"即可。
```

## 新增一条小车指令的步骤

1. `asr_keywords/config/keywords.yaml`：新增命令组（标准命令 + 同义词列表，
   同义词尽量覆盖口语变体和多音字写法）；
2. `speech_controller/config/commands.yaml`：在对应表（`commands` /
   `gripper_commands` / `service_commands`）加入**标准命令**到动作的映射，
   两边文字必须一致；
3. `colcon build --packages-select asr_keywords speech_controller` 后重启
   语音栈生效（或用 `commands_file`/`keywords_file` 参数指向外部配置免编译）。
