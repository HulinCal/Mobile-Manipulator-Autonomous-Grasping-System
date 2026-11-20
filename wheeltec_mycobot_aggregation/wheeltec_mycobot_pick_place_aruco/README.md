# wheeltec_mycobot_pick_place_aruco

> ArUco 标定块抓取/放下成功率测试包。**默认不主动导航**，仅在原地用机械臂夹取
> 0.03m 立方体；`aruco_align.launch.py` 下若 marker 距机械臂基座超过 20cm，
> `aruco_align_node` 会直接发 `/cmd_vel` 驱动底盘靠近，再执行抓取。

## 功能

检测 ArUco marker（ID=0, 边长 0.026m）。marker 在机械臂基坐标系（link1）下
**稳定出现 2 秒**后，机械臂执行一次完整抓取循环：预抓取 → 慢速下探 → 闭合夹爪
→ 提升 → 旋转 60° → 松开夹爪 → 回 home → 等待下一次检测。循环自动重复，
Ctrl+C 退出。

| 项 | 值 |
|---|---|
| Marker ID | 0 |
| Marker 边长 | 0.026 m（黑色方块边缘） |
| 方块尺寸 | 0.03 × 0.03 × 0.03 m |
| Marker 位置 | 方块顶面 |
| 机械臂基座系 | link1（mycobot280 真实 base） |
| 末端法兰 | link6_flange |
| 相机 | Astra Pro (D435 风格) |

## 文件结构

```
wheeltec_mycobot_pick_place_aruco/
├── package.xml                  # 包定义
├── setup.py                     # ament_python 安装脚本
├── setup.cfg
├── resource/wheeltec_mycobot_pick_place_aruco
├── config/
│   └── aruco_grasp_params.yaml   # 所有可调参数
├── launch/
│   └── aruco_align.launch.py     # 一键启动
└── wheeltec_mycobot_pick_place_aruco/
    ├── __init__.py
    └── aruco_grasp_node.py       # 唯一核心节点
```

## 架构

```
┌─────────────────────────────┐   ┌──────────────────────┐
│ wheeltec_mycobot_bringup     │   │ easy_handeye2         │
│   all.launch.py              │   │   publish.launch.py   │
│   ├─ robot_state_publisher   │   └──────┬───────────────┘
│   ├─ astra_camera            │          │ TF link1 → camera_color_optical_frame
│   ├─ mycobot_driver (串口)  │          ▼
│   ├─ ros2_control + 控制器  │   ┌──────────────────────┐
│   └─ MoveIt2 move_group      │   │ aruco_ros single      │
│                              │   │   /aruco_single/pose  │
└──────────────┬───────────────┘   └──────┬───────────────┘
               │                          │ 话题
               ▼                          ▼
        ┌────────────────────────────────────┐
        │        aruco_grasp_node.py         │
        │                                    │
        │  ① 稳定性检测 (2 秒窗口)           │
        │  ② link1→aruco_marker_frame        │
        │     TF 变换                        │
        │  ③ 抓取流程编排                     │
        │     ├─ MoveGroup ActionClient       │
        │     └─ GripperCommand ActionClient  │
        │  ④ 循环等待下一次检测               │
        └────────────────────────────────────┘
```

### 模块职责

`aruco_grasp_node.py` 内部按单一职责拆分为以下方法：

| 方法 | 层级 | 职责 |
|---|---|---|
| `__init__` | 节点 | 声明参数、创建 pub/sub/action client、初始化 TF 缓冲 |
| `_on_marker_pose` | 回调 | 接收 `/aruco_single/pose`，做稳定性判定（位置差 < 0.015m 持续 2 秒） |
| `_check_stable` | 工具 | 单帧稳定检查，更新参考位姿 |
| `_to_frame` | 工具 | 跨坐标系 PoseStamped 变换（依赖 TF 树已就绪） |
| `_get_block_center_pose` | 工具 | marker 中心 → 方块几何中心（顶面 marker 时 z 下偏 block_size/2） |
| `_flange_target_vertical` | 几何 | block_center → link6_flange 俯抓目标位姿 |
| `_send_move_goal` | 工具 | 发送 MoveGroup goal 并等待结果 |
| `_move_to_pose` | 动作 | 规划 + 执行到目标末端位姿（含重试、速度缩放） |
| `_move_to_joint_values` | 动作 | 规划 + 执行到指定关节角 |
| `_move_gripper` | 动作 | 控制夹爪开合 |
| `_wait_flange_settled` | 动作 | MoveGroup 成功后等待法兰物理到位且静止（防止在"还在下降途中就闭合"） |
| `run_pick_place_cycle` | 业务 | **完整抓取循环**，串联 8 个步骤 |
| `run` | 主循环 | 轮询 `_stable_marker_pose`，触发抓取后锁存直到完成 |

## TF 链路（关键）

```
link1 ──────────────────────────────► camera_color_optical_frame
   │         easy_handeye2 手眼标定 TF
   │         (publish.launch.py)
   │
   └──────────────────────────────── aruco_marker_frame
           │
           │  aruco_ros single 发布
           │  (marker 中心, Z 指向相机)
           │
           └──────────────────────────────► 机械臂可达性判定
                                         link1→link6_flange (robot_state_publisher)
```

节点内调用 `tf2_ros.Buffer.lookup_transform('link1', 'aruco_marker_frame')`
即可得到 marker 在机械臂基座系下的 3D 位置。TF 未就绪时稳定性判定会自动重置，
**不会触发抓取**。

> 注意：`aruco_ros single.launch.py` 的 `reference_frame` 参数留空（默认相机系），
> 因为我们在 Python 节点里自己做 `link1` 变换——可以精确控制变换时机和异常处理。

## 话题 / 服务 / Action

### 订阅

| 话题 | 类型 | 来源 | 说明 |
|---|---|---|---|
| `/aruco_single/pose` | `geometry_msgs/PoseStamped` | aruco_ros single | marker 在相机系下的位姿，frame_id = `camera_color_optical_frame` |

### 发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `/pick_place/status` | `std_msgs/String` | 抓取进度文本（日志+UI 状态） |
| `/mycobot/stop` | `std_msgs/Empty` | 机械臂固件急停（当前未主动使用，保留接口） |

### Action Client

| Action | 类型 | 目标 | 说明 |
|---|---|---|---|
| `/move_action` | `moveit_msgs/action/MoveGroup` | MoveIt2 move_group | 所有末端/关节运动 |
| `/gripper_action_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | ros2_control gripper | 夹爪开合 |

## 抓取流程（8 步）

```
┌─ 等待 marker 稳定 2s ──────────────────────────────────────────────────┐
│                                                                         │
│  Step 0: home + 张爪 (并行)                                              │
│           └─ 6 关节全零 + gripper_open_value (0.15 rad = 全开)            │
│                                                                         │
│  Step 1: 预抓取                                                        │
│           └─ 法兰 z 轴竖直朝下, 在 block_center 上方 approach_distance   │
│              (0.08 m); 直上不可达时尝试内收 0.06 m                       │
│                                                                         │
│  Step 2: 慢速下探                                                       │
│           └─ vel_scale=0.1 (速度缩放到 10%), 到 block_center 正上方     │
│              tcp_flange_z 处; 到位后 _wait_flange_settled 静止确认       │
│                                                                         │
│  Step 3: 闭合夹爪                                                       │
│           └─ gripper_close_value (-0.42 rad ≈ 张开 1/3), sleep 1.5s     │
│                                                                         │
│  Step 4: 提升 0.10 m                                                    │
│           └─ 保持俯抓姿态, block_center + lift_dist 竖直上移             │
│                                                                         │
│  Step 5: 回 home → 移到旋转 60° 放下点                                  │
│           └─ 先回 home (夹爪保持夹紧)                                    │
│           └─ 计算: drop_x = r·cos(θ+60°), drop_y = r·sin(θ+60°)       │
│              (r=原 marker 到 link1 径向距离, θ=原极角, +60° 绕 Z 旋转)   │
│           └─ 到 drop 上方 0.05 m, 再低速下降                            │
│                                                                         │
│  Step 6: 松开夹爪 (drop)                                                │
│           └─ gripper_open_value (0.15 rad), sleep 1.0s                  │
│                                                                         │
│  Step 7: 抬升 + 回 home                                                 │
│           └─ 先向上 0.05 m 退出, 再 6 关节全零                           │
│                                                                         │
└─────────────── 等待下一次检测 ───────────────────────────────────────────┘
```

## 快速开始

### 1. 编译

```bash
cd /home/admi/wheeltec_mycobot_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wheeltec_mycobot_pick_place_aruco
source install/setup.bash
```

### 2. 一键启动

```bash
# 整机启动 (机械臂 bringup + 手眼标定 + aruco + 抓取节点)
ros2 launch wheeltec_mycobot_pick_place_aruco aruco_align.launch.py
```

Launch 内部时序（TimerAction 串联，避免 TF 未就绪）：

| 时间 | 动作 |
|---|---|
| t=0 | `wheeltec_mycobot_bringup/all.launch.py`（机械臂、相机、MoveIt2） |
| t=2s | `easy_handeye2/publish.launch.py`（发布 `link1→camera_color_optical_frame`） |
| t=3s | `aruco_ros single`（检测 marker） |
| t=4s | `aruco_grasp_node`（主抓取逻辑） |

### 3. 可选参数

```bash
ros2 launch wheeltec_mycobot_pick_place_aruco aruco_align.launch.py \
    marker_id:=0 \
    marker_size:=0.026 \
    calibration_name:=mycobot280_calibration \
    use_rviz:=true \
    use_real_hardware:=true
```

### 4. 分步测试

```bash
# 仅 bringup + RViz + MoveIt2 (不启硬件)
ros2 launch wheeltec_mycobot_pick_place_aruco aruco_align.launch.py \
    use_real_hardware:=false

# 单独验证手眼标定 TF
ros2 run tf2_ros tf2_echo link1 camera_color_optical_frame

# 单独验证 aruco 检测
ros2 topic echo /aruco_single/pose --once

# 单独验证 marker 在 link1 系的位置
ros2 run tf2_ros tf2_echo link1 aruco_marker_frame
```

## 底盘接近（aruco_align_node）

`aruco_align.launch.py` 启动的 `aruco_align_node` 在 `ST_ALIGN` 状态下每拍先检查
marker 在 `link1` 系下的**水平距离** `r = sqrt(x² + y²)`：

| 条件 | 动作 |
|---|---|
| `r > 0.20 m`（`approach_start_distance`） | 发布 `/cmd_vel`：边走边调，前进与航向修正同时进行 |
| 接近过程中（每 0.2 s） | 用最新 marker 位姿重算方位角，叠加到当前角速度上持续修正（5Hz），途中移动 marker 也能跟随 |
| `r ≤ 0.18 m`（`approach_stop_distance`） | 立即零速，清空旧的对准/稳定状态，停稳后**重新对准一次再抓取** |
| `r < 0.15 m`（`retreat_trigger_distance`） | 太近：**直线后退**（`vx<0`、`wz=0`，不看方向）到 `r ≥ 0.18 m`（`retreat_stop_distance`），停稳后重新对准抓取 |
| marker 丢失超过 `marker_timeout` | 立即零速并退出接近/后退 |
| 单次累计指令行程 > 0.5 m（`approach_max_travel` / `retreat_max_travel`） | 停止接近/后退并告警，交回机械臂对准 |
| 进入抓取状态机（张开/前伸/闭合/回位） | 强制零速，不再动底盘 |

**为什么用水平距离**：marker 通常贴在地面/低处，3D 距离会把竖直高度差算进去，
水平方向已经够近时仍会判成“太远”，导致小车持续前顶。

**方向控制（边走边调）**：marker 经 `link1 -> base_link` 静态 TF 转到
`base_link`，取方位角 `atan2(y, x)`；每 `approach_update_period`（默认 0.2 s，即
1 秒 5 次）重新解算一次，角速度 `wz = 1.5·err`（限幅 0.20 rad/s）与恒定的
`vx = 0.05 m/s` 同时下发，**不会因航向偏差中断前进**，用户途中移动 marker 也能
随时调整方向。

停车瞬间允许残余十几度的航向偏差（用户要求，不为此停车原地补转），由停车后
机械臂重新对准兜底。注意：若“原地转正再走”，航向容差稍大一点点底盘就不会走
（低速转向死区/检测抖动），表现为一直原地转不前进。

**太近后退**：marker 水平距离 < `retreat_trigger_distance`（默认 0.15 m）时，
直接发负 `vx`（`wz=0`，不考虑方向）直线后退，退到 `r ≥ retreat_stop_distance`
（默认 0.18 m）停车，再清空对准状态重新对准抓取；单次后退累计指令行程受
`retreat_max_travel`（默认 0.5 m）限制，防止测距异常时一直后倒。

后退速度 `retreat_max_linear` 默认 0.01 m/s（比前进 0.05 m/s 慢很多）。后退过快
时停车延迟会让 `r` 冲过 `approach_start_distance`（0.20 m），立刻又触发前进，
表现为小车在 15~20 cm 之间反复前进/后退。

marker 面法线偏航角也会解析并打印（`face_yaw`），但因小 marker 的 PnP 姿态存在
翻折二义性，转向目标仍以“对准 marker 中心”为准，不用面法线直接控制航向。

**到达后必须重新对准**：底盘移动后，机械臂会清空移动前的 `_aligned`、
`_stable_since`、`_stable_ref`、`_last_cmd`、`_cmd_q`，从当前实际关节重新解算
J1~J5 目标角，并重新满足 `grasp_stable_time` 稳定时间后才进入抓取，避免沿用
移动前的对准结论导致抓不到。

> 本 launch 关闭了 `chassis_guard`，所以默认直接发 `/cmd_vel` 给底盘驱动。
> 若启用守卫，把 `chassis_cmd_topic` 改为 `/cmd_vel_in`，由守卫做避障修正。

对应 launch 参数：`chassis_approach`、`chassis_cmd_topic`、
`approach_start_distance`、`approach_stop_distance`、`approach_max_linear`、
`approach_max_angular`、`approach_max_travel`、`approach_update_period`。

## 可调参数

完整列表见 [config/aruco_grasp_params.yaml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec_mycobot_aggregation/wheeltec_mycobot_pick_place_aruco/config/aruco_grasp_params.yaml)：

| 参数 | 默认值 | 说明 |
|---|---|---|
| **ArUco 检测** | | |
| `marker_id` | 0 | 要检测的 marker ID |
| `marker_size` | 0.026 | marker 边长（黑色方块）[m] |
| `block_size` | 0.030 | 方块边长 [m] |
| `marker_on_top` | true | marker 是否在方块顶面（决定方块中心 z 偏移方向） |
| `stable_duration` | 2.0 | 稳定判定时长 [s] |
| `stable_pos_tol` | 0.015 | 稳定判定的位置容差 [m] |
| **抓取几何** | | |
| `tcp_flange_z` | 0.06 | link6_flange +z 到夹爪夹持中心距离 [m] |
| `approach_distance` | 0.08 | 预抓取高度（方块上方）[m] |
| `lift_distance` | 0.10 | 提升距离 [m] |
| `rotate_yaw_deg` | 60.0 | 放下点相对原 marker 位置的 Z 轴旋转角 [deg] |
| **运动** | | |
| `planning_time` | 5.0 | MoveIt 规划超时 [s] |
| `explore_planning_time` | 1.5 | 候选位姿试错的短规划时限 [s] |
| `vel_scale_slow` | 0.1 | 精细下探的速度缩放 |
| `vel_scale_fast` | 0.3 | home / lift 的速度缩放 |
| **夹爪** | | |
| `gripper_open_value` | 0.15 | 全开 [rad]（映射 pymycobot 100） |
| `gripper_close_value` | -0.42 | 约 1/3 张开 [rad]（映射 pymycobot 33） |
| `gripper_settle_time` | 1.5 | 闭合指令后等待机械夹紧 [s] |
| **坐标系** | | |
| `arm_base_frame` | link1 | 机械臂基座 TF |
| `eef_link` | link6_flange | 末端法兰 TF |

## 依赖

| 包 | 版本 | 用途 |
|---|---|---|
| `rclpy` | Jazzy | ROS2 Python 客户端 |
| `moveit_ros_move_group` | Jazzy | MoveIt2 move_group action 服务 |
| `moveit_ros_planning_interface` | Jazzy | MoveGroup action 定义 |
| `mycobot_moveit_config` | — | SRDF + kinematics.yaml（arm planning group） |
| `aruco_ros` | Jazzy | single marker 检测 |
| `tf2_ros` | Jazzy | TF 订阅与变换 |
| `wheeltec_mycobot_bringup` | 本仓库 | 机械臂/相机/控制器 bringup |
| `easy_handeye2` | 本仓库 | 手眼标定 TF 发布 |

## 常见问题

### 1. aruco_grasp_node 一直打印 "Waiting for stable ArUco marker ..."

可能原因：
- **手眼标定 TF 未发布** — 确认 `~/.ros/easy_handeye/mycobot280_calibration.yaml` 存在
  ```bash
  ros2 run tf2_ros tf2_echo link1 camera_color_optical_frame
  # 有输出 = TF 正常; 抛异常 = 手眼标定 TF 缺失
  ```
- **aruco_ros 未启动** — `ros2 topic echo /aruco_single/pose` 看有没有数据
- **相机未启动** — bringup 里 Astra Pro 要先起来

### 2. MoveGroup 报 "start point deviates from current robot state"

原因是 ros2_control 控制器刚启动，`/joint_states` 还是 URDF 默认值。等 1~2s
joint_state_broadcaster 发布真实关节后自动恢复；否则检查控制器加载顺序。

### 3. 预抓取规划失败 "Cannot reach pre-grasp in any orientation"

marker 超出臂展极限（mycobot280 水平可达约 0.32 m）。解决：
- 增大 `approach_distance` 让 marker 更靠近机械臂
- 检查 marker 的 link1 坐标是否合理

### 4. 夹紧后苹果/方块被碰掉

`_wait_flange_settled` 超时或 vel_scale 太大。降低 `vel_scale_slow` 到 0.05，
确认法兰在闭合前完全停稳。

### 5. 夹爪根本没夹紧

`gripper_close_value=-0.42 rad` 映射到 pymycobot value=33（张开约 1/3）。
可调小（更紧）或调大（松一些），实测硬限位 rad≈-0.683（value=0）。

### 6. 机械臂固件 error 5

关节角度超出 myCobot280 固件限位（J1 ±168°, J2 ±140°, J3 ±150°, J4 ±150°,
J5 [-155°, 160°], J6 ±180°）。MoveIt 规划虽然能出解，但固件拒绝执行。
解决：临时放宽限位 → clear_error_information → 下发回限位内 → 恢复限位。

## 与 wheeltec_mycobot_pick_place 的关系

| 对比项 | wheeltec_mycobot_pick_place | wheeltec_mycobot_pick_place_aruco |
|---|---|---|
| 感知 | YOLOv8 苹果检测 + 深度图 | aruco_ros 单 marker 检测 |
| 抓取对象 | 红苹果（半径 0.02m，未知姿态） | 0.03m ArUco 方块（marker 已知姿态） |
| 是否移动小车 | 是（Nav2 导航接近目标） | **否**（原地夹取） |
| 抓取触发 | 语音指令 + 导航到位后 | marker 稳定 2 秒自动触发 |
| 放下动作 | 无（夹着苹果回家） | 回 home → 旋转 60° 放下 |
| 主循环 | coordinator_node 编排 | aruco_grasp_node.run() 自己循环 |
| 稳定性判定 | 无（视觉每帧覆盖） | 有（2s 窗口） |
