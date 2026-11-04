# wheeltec_mycobot_test 使用说明

本包用于测试组合后的小车（WHEELTEC R680 底盘 + myCobot 280 机械臂）。
当前包含三个测试节点：

| 节点 | 可执行名 | 启动文件 | 用途 |
|---|---|---|---|
| 里程计原地旋转漂移测试 | `odom_spin_test` | `odom_spin_test.launch.py` | 原地自转 N 圈，检查里程计轨迹是否画圈漂移 |
| IMU 误差测试 | `imu_error_test` | `test_imu.launch.py` | 对比 IMU 与里程计的角速度/姿态误差 |
| IMU 零偏标定 | `imu_bias_calib` | `imu_bias_calib.launch.py` | 静止采集，计算陀螺仪/加速度零偏 |

---

## 1. 里程计原地旋转漂移测试（odom_spin_test）

### 1.1 测试原理

让小车原地自转 N 圈，同时记录两路里程计：

- `/odom` —— 底盘轮速里程计（原始输出）
- `/odom_combined` —— EKF 融合里程计

理想情况下原地旋转的轨迹应收缩在一点。若轮径/轮距标定不准、角速度与线速度换算不匹配，
里程计轨迹会画出一个圆，并随圈数逐圈漂移。

### 1.2 前置条件

- 已编译并 source 工作区：

```bash
cd ~/wheeltec_mycobot_ws
colcon build --packages-select wheeltec_mycobot_test
source install/setup.bash
```

- 小车周围（尤其机械臂回转范围）没有障碍物，默认会转 2 圈，约 31 秒。
- 底盘串口、机械臂串口正常（默认 `/dev/wheeltec_controller`、`/dev/mycobot_controller`）。

### 1.3 默认运行

```bash
ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py
```

默认行为：

1. include `wheeltec_mycobot_bringup/all.launch.py`，拉起组合整车完整栈
   （聚合 URDF/RSP + 底盘串口 + 雷达/相机 + EKF + 机械臂 + RViz）；
2. 为做纯 odom 测试，默认关闭 `chassis_guard`（避障守卫）、语音控制、`move_group`，
   让测试节点直接发 `/cmd_vel` 驱动底盘；
3. 等待 5 秒（底盘与 EKF 稳定）后，原地以 0.4 rad/s 旋转 2 圈；
4. 结束后打印两路里程计的漂移统计，并把轨迹发布到 `/odom_spin_test/path`。

### 1.4 常用示例

```bash
# 原地 1 圈，慢速
ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py \
    revolutions:=1.0 angular_speed:=0.3

# 整车栈已经手动启动，只跑测试节点
ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py bringup:=false

# 只采样不发速度指令（自己用键盘/遥控器控制旋转）
ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py publish_cmd_vel:=false

# 不启动 RViz（无显示器或想看纯日志）
ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py use_rviz:=false

# 想连同 MoveIt 一起启动
ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py use_move_group:=true

# 用原始轮速里程计判断停止条件（默认用 EKF）
ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py control_source:=raw

# 保留 chassis_guard（此时测试节点应改发 /cmd_vel_in，见下方说明）
ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py \
    use_chassis_guard:=true publish_cmd_vel:=false
```

### 1.5 启动参数说明

| 参数 | 默认值 | 说明 |
|---|---|---|
| `bringup` | `true` | 是否 include `all.launch.py` 拉起整车栈 |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `use_move_group` | `false` | 是否启动 MoveIt2 `move_group`（odom 测试不需要） |
| `use_chassis_guard` | `false` | 是否启动 `chassis_guard` 避障守卫（纯 odom 测试需关闭） |
| `use_speech` | `false` | 是否启动语音控制链路 |
| `use_real_hardware` | `true` | 是否连接真实硬件（串口/相机/雷达） |
| `angular_speed` | `0.4` | 原地旋转角速度（rad/s），正值逆时针 |
| `revolutions` | `2.0` | 原地旋转圈数 |
| `duration` | `45.0` | 超时兜底秒数，超过即停车统计 |
| `publish_cmd_vel` | `true` | 是否由测试节点自己发 `/cmd_vel` |
| `control_source` | `fused` | 用哪路里程计判断何时停止：`fused`(/odom_combined) / `raw`(/odom) |
| `wait_seconds_before_start` | `5.0` | 底盘稳定后等待多少秒再开始旋转 |

节点级参数（默认值一般无需改，可用 `--ros-args -p` 覆盖）：
`odom_topic`、`odom_fused_topic`、`cmd_vel_topic`、`path_topic`、`sample_rate`。

### 1.6 输出指标解读

测试结束（或 Ctrl+C 中途退出）会为每路里程计打印一段结果：

```
====== 漂移结果 [odom_combined(EKF)] ======
  采样点数     : 1520
  累计旋转角   :   720.35 °
  目标旋转角   :   720.00 °
  角度误差     :     0.35 °
  起点→终点位移: 0.0210 m (dx=+0.0180, dy=+0.0108)
  最大半径     : 0.0300 m   (相对起点)
  RMS 半径     : 0.0110 m   (相对质心)
  每圈漂移     : 0.0105 m/圈
  结论         : 定点旋转, 无明显漂移
==================================
```

| 指标 | 含义 |
|---|---|
| 累计旋转角 / 角度误差 | 实际转过的角度与目标角度的差，反映角速度标定 |
| 起点→终点位移 | 结束后车体在里程计坐标中偏离起点的距离，即"漂移量" |
| 最大半径 | 整个过程中离起点最远的距离，反映"画圈"的最大幅度 |
| RMS 半径 | 轨迹相对质心的离散程度，理想原地旋转应接近 0 |
| 每圈漂移 | `起点→终点位移 / 圈数` |

结论判定阈值：

- `< 0.05 m`：定点旋转，无明显漂移
- `< 0.15 m`：轻微画圈漂移
- `>= 0.15 m`：明显画圈漂移，需检查轮径/轮距参数与轮速里程计标定

### 1.7 在 RViz 中查看轨迹

测试结束后，轨迹发布在 `/odom_spin_test/path`（`nav_msgs/Path`，frame 为里程计坐标系）：

1. RViz → Add → 选择 **Path** 显示；
2. Topic 填 `/odom_spin_test/path`；
3. 如果轨迹看起来是一个圆而不是一个点，说明里程计存在画圈漂移。

### 1.8 注意事项与故障排查

- **没有里程计数据**：节点会每 5 秒重试并打印
  `尚无里程计数据 ...`。依次检查底盘驱动进程、`/odom` 是否发布、EKF 是否发布
  `/odom_combined`（`ros2 topic hz /odom`、`ros2 topic hz /odom_combined`）。
- **小车不转**：确认 `publish_cmd_vel:=true`；若同时开启了 `chassis_guard`，
  守卫只接受 `/cmd_vel_in`，此时应关闭守卫或改由其他速度源驱动。
- **角度误差偏大**：多为 `angular_speed` 与底盘实际响应不匹配，或里程计角速度标定偏差；
  可先降到 0.2~0.3 rad/s 复测。
- **位移偏大但 RMS 半径很小**：轨迹是直线偏移而非画圈，通常是里程计线速度标定
  （轮径）问题；若 RMS 半径也大，则更接近轮距/角速度标定问题。
- **Ctrl+C**：节点会先持续发布约 1 秒零速，再退出；中途退出也会打印一次统计结果。

---

## 2. IMU 误差测试（imu_error_test）

```bash
# 原地转 90° + 前进 1m + 后退 1m
ros2 launch wheeltec_mycobot_test test_imu.launch.py

# 原地转 180°，前进 2m
ros2 launch wheeltec_mycobot_test test_imu.launch.py \
    target_rotation_deg:=180 target_distance:=2.0

# 只测前进 1.5m
ros2 launch wheeltec_mycobot_test test_imu.launch.py \
    test_mode:=forward target_distance:=1.5

# 只采样，不发 cmd_vel
ros2 launch wheeltec_mycobot_test test_imu.launch.py publish_cmd_vel:=false
```

参数：`test_mode`（rotate/forward/backward/all）、`target_distance`、`target_rotation_deg`、
`angular_speed`、`linear_speed`、`publish_cmd_vel`、`sample_rate`、
`wait_seconds_before_start`、`duration`。

---

## 3. IMU 零偏标定（imu_bias_calib）

### 3.1 原理

- 车体完全静止时，陀螺仪真实角速度为 0，其输出**均值**就是零偏（bias）。
- 加速度计静止时只感受重力，水平安装时均值应为 `(0, 0, +g)`，
  减去该期望值即为加速度零偏（`assume_flat_mount:=true` 时按 `|g|` 扣除）。
- 采样期间车体必须完全静止，否则均值里混入真实运动。节点会用三轴标准差做静止性检查。

### 3.2 运行

```bash
# 默认：静止采集 20s
ros2 launch wheeltec_mycobot_test imu_bias_calib.launch.py

# 采集 30s
ros2 launch wheeltec_mycobot_test imu_bias_calib.launch.py duration:=30.0

# 整车栈已启动，只跑标定节点
ros2 launch wheeltec_mycobot_test imu_bias_calib.launch.py bringup:=false

# 车体非水平安装（不按 |g| 扣重力，只报告均值）
ros2 launch wheeltec_mycobot_test imu_bias_calib.launch.py assume_flat_mount:=false
```

### 3.3 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `bringup` | `true` | 是否 include `all.launch.py` 拉起整车栈 |
| `use_rviz` / `use_move_group` | `true` / `false` | RViz、MoveIt 开关 |
| `use_chassis_guard` / `use_speech` | `false` | 标定期间保持车体静止，无需避障与语音 |
| `use_real_hardware` | `true` | 是否连接真实硬件 |
| `duration` | `20.0` | 静止采样时长（s） |
| `assume_flat_mount` | `true` | 车体是否水平安装（决定加速度零偏是否扣重力） |
| `auto_exit` | `true` | 打印结果后自动退出节点 |

节点级参数（`--ros-args -p` 覆盖）：`imu_topic`（默认 `/mobile_base/sensors/imu_data`）、
`min_samples`、`gravity`、`max_gyro_std`（默认 0.02 rad/s）、`max_accel_std`（默认 0.10 m/s²）。

### 3.4 实测输出示例（静止 15s）

```
====== IMU 零偏标定结果 ======
  采样     : 300 帧 / 15.1 s  (19.9 Hz)

  --- 陀螺仪 (静止时输出均值即零偏) ---
  零偏 rad/s : x=-0.013922  y=+0.001701  z=-0.000107
  零偏 °/s   : x=-0.7977  y=+0.0974  z=-0.0062
  噪声 std   : x=0.000367  y=0.000377  z=0.000393 rad/s

  --- 加速度计 ---
  均值 m/s²  : x=-0.0237  y=+0.0646  z=+9.6685
  噪声 std   : x=0.0045  y=0.0042  z=0.0093
  零偏 m/s²  : x=-0.0237  y=+0.0646  z=-0.1382   (已按 |g| 扣除重力)

  静止性检查 : OK
```

### 3.5 结果在哪里修改（重点）

**本车的 IMU 零偏在当前工程里没有 yaml 可配。** 核查结果：

- [wheeltec_robot.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/turn_on_wheeltec_robot/src/wheeltec_robot.cpp#L359-L368)
  直接把底盘 MCU 上送的原始值乘固定比例后发布，**没有任何零偏扣除**；
- [base_serial.launch.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/turn_on_wheeltec_robot/launch/base_serial.launch.py)
  只配置 `usart_port_name`、`serial_baud_rate`、`robot_frame_id`、`odom_frame_id`、`cmd_vel` 等，
  **没有 bias 字段**；
- [wheeltec_robot.cpp 的 declare_parameter 区域](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/turn_on_wheeltec_robot/src/wheeltec_robot.cpp#L445-L459)
  也只声明串口与坐标系参数；
- 比例常数固定写在 [wheeltec_robot.h](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/turn_on_wheeltec_robot/include/turn_on_wheeltec_robot/wheeltec_robot.h#L52-L57)：
  `GYROSCOPE_RATIO = 0.00026644f`、`ACCEL_RATIO = 1671.84f`；
- `ekf.yaml` 里的 `imu0_differential: true` 只是用相邻帧差分消除缓慢漂移，
  **不消除绝对零偏**（robot_localization 也没有 bias 参数）。

因此要真正扣除零偏，只有三条途径：

| 途径 | 位置 | 说明 |
|---|---|---|
| A. 改驱动代码（推荐，可控） | [wheeltec_robot.cpp L366-368](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/turn_on_wheeltec_robot/src/wheeltec_robot.cpp#L366-L368) 角速度、[L359-361](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/turn_on_wheeltec_robot/src/wheeltec_robot.cpp#L359-L361) 加速度 | 在换算后减去标定值；可顺带把 bias 声明为参数并写进 launch 的 `parameters`，这样就变成可配置的 yaml/launch 参数 |
| B. 上位机 / 固件标定 | WHEELTEC 底盘上位机 | 由 MCU 侧做零偏补偿，驱动不用改 |
| C. 下游节点补偿 | 自己的 filter 节点 / EKF 前置 | 订阅 IMU 后扣零偏再转发，驱动保持原样 |

对本车 2D 导航而言，只有**绕 z 轴**的陀螺零偏会直接造成航向漂移。
本次实测 z 轴零偏约 `-0.0001 rad/s`（≈ `-0.006°/s`，约 0.36°/分钟），
可忽略；x 轴 `-0.8°/s` 虽然较大，但绕 z 的航向推算不使用 x 轴，影响有限。
