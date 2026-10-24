# 语音引导的移动抓取机器人（wheeltec R680 + MyCobot 280）

> 基于 ROS 2 Jazzy 的端到端苹果抓取系统：说一句"把苹果拿过来"，小车自主导航到目标前，机械臂完成识别、定位、抓取并带回起点。

---

## 目录

- [一、项目概述](#一项目概述)
- [二、硬件配置](#二硬件配置)
- [三、软件架构](#三软件架构)
- [四、端到端执行流程](#四端到端执行流程)
- [五、数据流与话题](#五数据流与话题)
- [六、抓取功能（核心）](#六抓取功能核心)
- [七、包结构](#七包结构)
- [八、启动与使用](#八启动与使用)
- [九、关键参数](#九关键参数)
- [十、经验与踩坑记录](#十经验与踩坑记录)

---

## 一、项目概述

本项目实现了一套**语音指令驱动的移动操作（Mobile Manipulation）系统**。操作者用自然语言下达任务（如"把苹果拿过来"），系统依次完成：

1. **语音识别**：离线 ASR 将语音转文字；
2. **指令理解**：拼音 + 字符双通道模糊匹配，将口语映射为标准命令；
3. **视觉检测**：YOLO 目标检测 + 深度相机解算苹果三维坐标；
4. **自主导航**：视觉伺服 + 激光雷达精确接近目标；
5. **机械臂抓取**：MoveIt 规划，可选"竖直俯抓"或"规划器自动选角度"两种抓取策略；
6. **回程**：抓取成功后收臂、倒退返回起点，任务结束。

整套系统在真实硬件上完成了多轮端到端验证，抓取链路经过反复调参，具备较强的鲁棒性（深度闪烁重试、规划随机性容错、到位判定补发等）。

---

## 二、硬件配置

| 部件 | 型号 | 说明 |
|---|---|---|
| 计算平台 | **Intel NUC11（i5）** | 主控，运行 ROS 2 全部节点，USB 直连相机与机械臂 |
| 移动底盘 | **wheeltec R680** | 四轮驱动底盘（`senior_4wd_bs_robot`），带编码器与 IMU |
| 机械臂 | **Elephant Robotics MyCobot 280** | 6 自由度协作臂 + 自适应夹爪，串口通信 |
| 深度相机 | **奥比中光 Astra Pro** | 结构光 RGB-D，用于苹果检测与三维定位 |
| 激光雷达 | **LSLIDAR** | 2D 激光，导航避障与到位停车 |
| 麦克风 | 麦克风阵列 | 离线语音采集 |

### 硬件接线要点（重要）

- **Astra 相机必须直连 NUC 的 USB 口**，不能经过外置 USB 2.0 Hub——否则供电/带宽不足会导致枚举异常或深度图持续空洞。
- **相机首次使用前必须安装 udev 规则**（astra_camera 包内 `install.sh`，用 `sudo sh` 执行）。
- **机械臂串口已配置 udev 软链** `/dev/mycobot_controller -> ttyACM1`，设备节点权限 666。
- 机械臂 Python 驱动依赖 `pymycobot>=3.7.0`，需用户级安装：
  `python3 -m pip install --user --break-system-packages pymycobot==3.9.9`

---

## 三、软件架构

| 层级 | 技术栈 |
|---|---|
| OS / ROS | Ubuntu 24.04 + **ROS 2 Jazzy**，colcon 构建（66 个包全部编译通过） |
| 运动规划 | **MoveIt 2**（move_group + OMPL），运动学求解 **TRAC-IK**（solve_type=Speed） |
| 控制框架 | ros2_control + JointTrajectoryController + GripperActionController |
| 定位 | robot_localization（EKF 融合编码器 + IMU） |
| 导航 | Nav2（可用时）+ 自研 cmd_vel 视觉伺服（兜底） |
| 视觉 | **YOLO26n（ONNX, onnxruntime）** + 深度直方图聚类 |
| 语音 | sherpa 离线 ASR + **pypinyin** 拼音模糊匹配 |

### 关键设计决策

- **TRAC-IK 运动学**：相比默认 KDL，求解更快更稳定（timeout 0.05s，attempts 3）。
- **苹果位姿用 TRANSIENT_LOCAL（latched）QoS**：所有发布/订阅苹果位姿的节点统一 latched，保证后连接的订阅者也能拿到历史位姿。
- **多线程执行器 + 独立回调组**：检测服务在超时窗口内持续重试，传感器回调用 `ReentrantCallbackGroup`，避免服务回调阻塞时收不到新帧。
- **运动学坐标系遵循 REP-105**：`world -> odom_combined -> base_footprint -> base_link`。URDF 以 `base_footprint` 为根，`world->odom_combined` 用静态变换发布，避免 base_footprint 双父节点导致的 TF 树断裂。

---

## 四、端到端执行流程

### 4.1 语音链路

```
麦克风 → asr_sherpa(/asr/final_text)
       → asr_keywords(拼音+字符模糊匹配, 阈值0.6)
       → /asr_command (标准命令)
       → voice_command_node / speech_controller_node
```

- ASR 输出最终文本后，`asr_keywords` 用**拼音相似度 + 字符相似度取最大值**做模糊匹配，能容错多音字、同音字（如"宋凯"≈"松开"）。
- 匹配阈值 0.6，同分时**优先更长短语**避免误判。
- 识别的标准命令"把苹果拿过来"触发抓取任务；另有"夹爪松开/闭合"、"机械臂停止/回家"等单步指令。

### 4.2 任务状态机（coordinator_node）

```
IDLE → DETECTING → NAV_TO_APPLE → GRASPING → NAV_TO_DELIVERY → DONE
```

| 状态 | 动作 |
|---|---|
| IDLE | 等待 `/pick_place/trigger`、`/pick_place/start` 或 `/pick_place/trigger_manual` |
| DETECTING | 调 `/pick_place/detect_apple`，记录起始位姿 |
| NAV_TO_APPLE | 视觉伺服接近苹果，激光雷达到位停车 |
| GRASPING | 调 `/pick_place/pick`，机械臂抓取 |
| NAV_TO_DELIVERY | 抓取成功后倒退返回起点（保持夹紧） |
| DONE | 到位即结束，苹果由现场人工取走 |

> 设计取舍：抓取成功后**不走送货点、不调用 place、不张开夹爪**，直接收臂回程。回程用 cmd_vel 倒退（不转弯，避免 180° 掉头超时）；Nav2 可用时仍走正常导航。

---

## 五、数据流与话题

### 5.1 主要话题 / 服务

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/camera/color/image_raw` | Image | 相机→检测 | 彩色图（YOLO 输入） |
| `/camera/depth/image_raw` | Image | 相机→检测 | 深度图（已与彩色配准） |
| `/camera/color/camera_info` | CameraInfo | 相机→检测 | **彩色内参**（深度反投影用） |
| `/pick_place/apple_pose` | PoseStamped | 检测→导航/抓取 | 苹果位姿，**latched** |
| `/pick_place/detect_apple` | Trigger | srv | 触发一次检测 |
| `/pick_place/pick` | Trigger | srv | 抓取（超时 120s） |
| `/pick_place/go_home` | Trigger | srv | 回起点 |
| `/pick_place/status` | String | 节点→外部 | 状态广播，latched |
| `/cmd_vel` | Twist | 导航→底盘 | 速度指令 |
| `/joint_states` | JointState | 驱动→全局 | 真实关节角（TF 源） |
| `/gripper_action_controller/gripper_cmd` | GripperCommand | action | 夹爪控制 |
| `/mycobot/cmd_joint_pos` / `cmd_gripper_pos` | — | 桥→驱动 | 50Hz 关节/夹爪指令 |
| `/tf` `/tf_static` | TF | 全局 | 坐标变换 |

### 5.2 抓取数据流（重点）

```
苹果检测(YOLO+深度直方图)
   │  /pick_place/apple_pose (base_link 系)
   ▼
grasp 节点: TF 变换 base_link → link1 (臂基座系)
   │  计算抓取位姿 (含俯仰角/朝向)
   ▼
MoveIt (move_group) 规划关节轨迹
   │  JointTrajectoryController
   ▼
ros2_control → C++ 桥 (50Hz) → /mycobot/cmd_joint_pos
   │
   ▼
mycobot_driver_node (异步 worker + 串口锁) → pymycobot → 机械臂
   │
   ▼ (反向) get_angles → /joint_states → TF → _wait_flange_settled 到位判定
夹爪: grasp 节点 → GripperCommand action → 桥 → /mycobot/cmd_gripper_pos → set_gripper_value
```

---

## 六、抓取功能（核心）

抓取是整个系统打磨最多的部分，提供**两种可切换的抓取节点**，服务/话题接口完全一致，coordinator 无感。

### 6.1 双抓取节点设计

通过 launch 参数 `grasp_impl` 二选一（互斥启动）：

| 取值 | 节点 | 策略 |
|---|---|---|
| `vertical`（**默认**） | `grasp_node` | 竖直向下俯抓，经过多次真机验证，稳定可靠 |
| `angle` | `grasp_angle_node` | **由规划器自动判断最优抓取角度** |

`grasp_angle_node` 继承 `grasp_node`，仅重写核心抓取逻辑，MoveGroup 调用、夹爪控制、到位判定、急停、服务接口全部复用——**最大程度保留经过测试的代码**。

### 6.2 规划器自动选角度（grasp_angle_node）

**动机**：相机装在车头，沿小车前方（base_link +x）水平观察苹果时信息最完整（三维），竖直俯抓时只能看到苹果正面一维信息。因此理论上**平行于小车前方的水平抓取最优，竖直向下最差**。

**实现**：抓取姿态按俯仰角 `pitch` 参数化：

```
趋近方向 a = cos(pitch)·d_xy − sin(pitch)·ẑ
```

- `d_xy` 为臂基座指向苹果的水平单位向量（苹果在正前方时 = link1 +y = 小车前方 +x）；
- `pitch=0°` 为水平前抓，`pitch=90°` 为竖直俯抓；
- 候选按 `grasp_pitch_angles_deg` **从最优到最差排序**，逐个交给规划器判定可达性，**第一个规划成功的角度即最优角度**；
- 非竖直候选的夹爪开合方向（法兰 x）始终**水平且垂直于趋近方向**，保证手指左右环绕苹果，不会上下夹撞台面。

**真机验证的角度选择行为**（苹果距臂基座仅 0.276m 时）：

```
pitch=0°  法兰 y=0.056 → 不可达（太靠近臂基座，自碰撞）
pitch=15° 法兰 y=0.064 → 不可达
pitch=30° 法兰 y=0.086 → 不可达
pitch=45° 法兰 y=0.121 → 可达 ✓ 选中
```

水平抓取需要法兰退到苹果后方 `apple_dist − (tcp_offset + approach_dist)`，当苹果离臂很近时该位置会撞基座，规划器自动回退到更竖直的角度——**这正是"规划器判断最优角度"的价值**。苹果放远（>0.35m）时水平角度才可达。

> 当前默认配置已改为竖直优先 `grasp_pitch_angles_deg: [90, 75, 60, 45, 30, 15, 0]`，兼顾稳定性（竖直抓取在圆苹果上更不易滑脱）。

### 6.3 抓取时序（两节点一致）

```
回 home ∥ 夹爪张到最大(并行)
  → pre-grasp（沿趋近轴后退 0.10m）
  → 缓慢下降/前进 (vel_scale 0.1)
  → _wait_flange_settled 到位判定
  → 闭合夹爪 → 等待 2s
  → 两段提升（先竖直小提，再内收运输位姿）
  → 收臂回 home（保持夹紧）→ 等 2s → pick 返回
```

### 6.4 关键技术点

**① 到位判定 `_wait_flange_settled`**：MoveGroup 报成功不代表物理臂到位（异步驱动合并轨迹点，最后一段固件运动滞后 1~2s）。抓取前用 TF（link1→link6_flange，源自 `/joint_states` 真实关节）判定：①与目标位置误差 <12mm；②连续两次读数位移 <2mm（静止），超时 6s，未到位补发一次目标再等——**确认臂真正停稳才闭合夹爪**。

**② 夹爪指令映射**：URDF 关节范围 `[-0.7, 0.15] rad` 线性对应 pymycobot `0..100`：`0.15 rad = 100 全开`，`-0.7 rad = 0 全闭`。实际抓取用 `gripper_close_value=-0.42 rad`（value 33，张开约 1/3），避免夹到硬限位出现"夹两下"。

**③ 驱动异步化**：`mycobot_driver_node` 中臂和夹爪都改为"回调只存最新目标 + 独立 daemon 工作线程阻塞下发串口"，executor 回调永不阻塞；两条 worker 共用串口锁，臂静止时不持锁 → 夹爪即时下发；关节命令按 0.02° 去重，避免 50Hz 重复指令淹没夹爪。

**④ 两段提升**：直接规划"升高+内收"目标时，OMPL 关节空间自由路径起始段会先小幅下沉（"夹住后先下降"）。改为阶段 A 纯竖直小提（让苹果离地），阶段 B 再内收到运输位姿。

**⑤ 规划提速**：TRAC-IK + 候选试错用短时限 `explore_planning_time=1.5s`（可达目标 TRAC-IK 通常 <0.5s 出解），全失败后才对内收位姿做一次 5s 完整兜底；候选排序把上次成功朝向排首位，减少试错。

### 6.5 抓取坐标补偿

- 苹果位姿 x 方向存在约 9cm 系统偏移，最终位姿 x 方向加 9cm 补偿；
- 检测测的是苹果**边界**距离，真实中心需在 x 方向加上苹果半径（当前 0.02m）；
- 苹果高度（相对 base_link）由用户标定固定，途中不再用远距离检测值更新（避免可达位置误判）。

---

## 七、包结构

```
src/
├── wheeltec_mycobot_aggregation/
│   └── wheeltec_mycobot_pick_place/      # 抓取主包
│       ├── launch/pick_place.launch.py   # 主启动（grasp_impl 选择抓取节点）
│       ├── config/pick_place_params.yaml # 全部调参
│       └── wheeltec_mycobot_pick_place/
│           ├── coordinator_node.py       # 任务状态机
│           ├── apple_detection_node.py   # YOLO+深度检测
│           ├── navigation_node.py        # 视觉伺服导航
│           ├── grasp_node.py             # 竖直抓取（默认）
│           ├── grasp_angle_node.py       # 规划器选角度抓取
│           └── voice_command_node.py     # 语音指令接入
├── wheeltec/                              # 底盘/相机/雷达驱动
│   ├── astra_camera_ros/                  # Astra 相机
│   └── ...
├── mycobot/
│   ├── mycobot_moveit_config/             # MoveIt 配置(TRAC-IK)
│   └── ...
└── models/yolo26n.onnx                    # 检测模型
```

---

## 八、启动与使用

### 8.1 环境准备

```bash
# ROS 2 Jazzy + 依赖（参见"硬件接线要点"安装 pymycobot / onnxruntime）
python3 -m pip install --user --break-system-packages pymycobot==3.9.9 onnxruntime pypinyin

# 构建（必须 PYTHONDONTWRITEBYTECODE=1，否则 rosidl 写 /opt 被拦）
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
colcon build --continue-on-error
source install/setup.bash
```

### 8.2 启动抓取系统

```bash
# 默认：竖直抓取节点（推荐，已充分测试）
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py

# 使用"规划器选角度"抓取节点
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py grasp_impl:=angle

# 可选参数
#   use_rviz:=true     启动 RViz
#   use_speech:=false  关闭语音栈
#   sim_mode:=true     仿真模式
```

### 8.3 触发任务

```bash
# 语音：直接说 "把苹果拿过来"
# 或手动触发完整流程：
ros2 service call /pick_place/trigger_manual std_srvs/srv/Trigger

# 单步调试：
ros2 service call /pick_place/detect_apple std_srvs/srv/Trigger   # 检测
ros2 service call /pick_place/pick std_srvs/srv/Trigger           # 抓取
ros2 service call /pick_place/go_home std_srvs/srv/Trigger        # 回起点
```

---

## 九、关键参数

`config/pick_place_params.yaml` 摘录：

| 参数 | 值 | 说明 |
|---|---|---|
| `confidence_threshold` | 0.2 | YOLO 检测阈值（红苹果置信度偏低，已下调） |
| `detect_timeout` | 10s | 检测重试窗口（深度闪烁容错） |
| `apple_fixed_z` | 0.155 | 苹果相对 base_link 的固定高度（标定值） |
| `gripper_open_value` | 0.15 rad | 夹爪全开（pymycobot 100） |
| `gripper_close_value` | -0.42 rad | 抓取闭合（value 33，张开约 1/3） |
| `grasp_pitch_angles_deg` | [90,75,60,45,30,15,0] | 抓取俯仰角候选（最优→最差，竖直优先） |
| `explore_planning_time` | 1.5s | 候选试错单次规划时限 |
| 导航速度 | 0.05 m/s | 线/角速度上限（狭小环境） |
| 最大移动距离 | 0.5 m | 防运动过量 |

---

## 十、经验与踩坑记录

1. **红色苹果几乎不反射结构光红外** → 深度图斑块状、中心像素常无效。改为检测框内有效深度的 50mm 直方图主导聚类中值，并在超时窗口内持续重试。
2. **USB 带宽不足是深度空洞的最终根因**：关闭 IR 流和点云流（只留彩色+深度+彩色内参）后深度立即稳定。
3. **深度内参必须用 `/camera/color/camera_info`**（彩色内参），深度图已与彩色配准；用 IR 内参（未标定，全 NaN）会导致反投影全 NaN。
4. **`do_transform_pose` 不能传 PoseStamped**，要用 `do_transform_pose_stamped`，否则抛异常穿透 executor 杀进程。服务回调务必加 try/except 兜底。
5. **TF 双父节点陷阱**：URDF 静态 `world->base_footprint` 与 EKF 动态 `odom->base_footprint` 冲突会拆断 TF 树。以 base_footprint 为根 + 静态 `world->odom` 解决。
6. **MoveGroup 报成功 ≠ 物理到位**，必须二次到位判定；MotionPlanRequest 必须显式设 `max_velocity/acceleration_scaling_factor > 0`。
7. **OMPL 规划有随机性**：同一目标偶发失败，重试或稍内收即成功，不必立刻怀疑配置。
8. **检测服务连续失败先查图像流**：`ros2 topic hz /camera/color/image_raw`，图像流可能整体无发布（环境暗/USB 抖动），重启 launch 栈恢复。

---
