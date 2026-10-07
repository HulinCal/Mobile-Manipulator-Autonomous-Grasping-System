# mycobot280_moveit2_control

ROS 2 (Jazzy) 包，为 Elephant Robotics 的 mycobot280 M5 机械臂提供 [ros2_control](https://control.ros.org/jazzy/index.html) 硬件接口，作为 MoveIt2 与现有 Python 驱动 `mycobot_driver_node.py` 之间的桥接层。

由于机械臂底层驱动是 Python 版本（基于 `pymycobot`），本包用 C++ 实现 `hardware_interface::SystemInterface`，仅负责"协议转换"：把 ros2_control 的位置指令以 `Float64MultiArray` 形式发布到话题，并订阅真实关节状态回传给 ros2_control。这样 MoveIt2 / `JointTrajectoryController` 即可在不感知 Python 驱动细节的前提下，规划并执行真实机械臂运动。夹爪同样以 0~100 的开合度（pymycobot 原生单位）独立桥接，由 `GripperActionController` 暴露为 `control_msgs/GripperCommand` action。

- 版本：0.0.1
- 许可证：BSD-3-Clause
- 维护者：hl \<hl@todo.com\>

---

## 1. 目录结构

```
mycobot280_moveit2_control/
├── CMakeLists.txt                              # 构建配置
├── package.xml                                 # 包元数据与依赖
├── mycobot280_moveit2_control.xml               # pluginlib 插件描述
├── config/
│   └── mycobot_280_real_controllers.yaml        # 控制器配置（arm + gripper + joint_state_broadcaster）
├── include/mycobot280_moveit2_control/
│   └── mycobot280_hardware_interface.hpp        # 硬件接口类声明
├── src/
│   ├── mycobot280_hardware_interface.cpp         # 硬件接口实现
│   └── mycobot_driver_node.py                   # Python 驱动（pymycobot → 串口），作为 ros2 可执行程序安装
├── launch/
│   ├── mycobot_280_real_moveit.launch.py        # 一键启动整套真机控制栈（+ MoveIt2 + RViz）
│   └── mycobot_280_real_pick_place.launch.py   # 在真机上运行 MTC pick & place demo
└── scripts/
    └── test_arm_movement.py                     # 端到端测试脚本（FollowJointTrajectory + GripperCommand）
```

---

## 2. 架构与数据链路

整套系统按以下链路把 MoveIt2 规划路径下发到真实机械臂与夹爪，并把真实关节状态回传：

```
                          ┌──────────────────┐
                          │   MoveIt2       │
                          │  move_group     │
                          └────────┬────────┘
                                   │ FollowJointTrajectory action
                                   ▼
                ┌──────────────────────────────────┐
                │ arm_controller                   │
                │ (JointTrajectoryController)      │
                └──────────────┬───────────────────┘
                               │ position command_interfaces (rad)
              ┌────────────────┼────────────────┐
              │                │                │
              │                ▼                │
              │  ┌──────────────────────────────────────┐
              │  │ gripper_action_controller           │
              │  │ (GripperCommand action, 0..100)    │
              │  └──────────────┬───────────────────┘
              │                 │ position command_interface (0..100)
              ▼                 ▼
        ┌──────────────────────────────────────────────────┐
        │ MyCobot280HardwareInterface (本包, C++)          │
        │  - write(): 把 arm 指令拷贝到 pending_cmd_，     │
        │    把 gripper 指令拷贝到 pending_gripper_cmd_   │
        │  - 周期 wall_timer 同时发布：                    │
        │      /mycobot/cmd_joint_pos  (rad, 6 个)         │
        │      /mycobot/cmd_gripper_pos (0..100, 1 个)    │
        │  - state_callback 订阅：                         │
        │      /mycobot/current_joint_pos  (rad, 6 个)     │
        │      /mycobot/current_gripper_pos (0..100, 1 个) │
        └────────┬─────────────────────┬───────────────────┘
                 │ cmd                  │ state
                 ▼                      ▲
        ┌──────────────────────────────────────────────────┐
        │  mycobot_driver_node.py (Python, pymycobot)      │
        │  位于本包 src/，由 CMakeLists.txt 安装为         │
        │  lib/<pkg>/mycobot_driver_node.py 可执行程序     │
        │  - 订阅 /mycobot/cmd_joint_pos                  │
        │      rad -> deg，调用 mc.send_angles(deg, 30)   │
        │  - 50 Hz 读 mc.get_angles()，deg -> rad，        │
        │    发布到 /mycobot/current_joint_pos            │
        │  - 订阅 /mycobot/cmd_gripper_pos                │
        │      调用 mc.set_gripper_value(value, 30)        │
        │  - 50 Hz 读 mc.get_gripper_value()，             │
        │    发布到 /mycobot/current_gripper_pos          │
        └──────────────────┬───────────────────────────────┘
                           │ 串口 (USB)
                           ▼
                ┌──────────────────────┐
                │   mycobot280 M5 真机 │
                │   (6 arm + 夹爪)    │
                └──────────────────────┘
```

关节名称（与 URDF `mycobot_280_ros2_control.urdf.xacro` 一致，无前缀）：

| 索引 | 关节名                       | 类型           | 单位                  |
|------|------------------------------|----------------|-----------------------|
| 0    | `link1_to_link2`             | 手臂关节       | rad                   |
| 1    | `link2_to_link3`             | 手臂关节       | rad                   |
| 2    | `link3_to_link4`             | 手臂关节       | rad                   |
| 3    | `link4_to_link5`             | 手臂关节       | rad                   |
| 4    | `link5_to_link6`             | 手臂关节       | rad                   |
| 5    | `link6_to_link6_flange`      | 手臂关节       | rad                   |
| 6    | `gripper_controller`         | 夹爪开合度     | 0..100（pymycobot 原生）|

> 📌 夹爪关节由 URDF `mycobot_280_ros2_control.urdf.xacro` 的 `use_gazebo:=false` 分支声明，与 6 个手臂关节一起进同一个 `ros2_control` 硬件插件。夹爪的 `command_interface` 是 `position`（范围 0~100），`state_interface` 同时声明 `position` 和 `velocity`（后者恒为 0，仅用于满足 `GripperActionController` 的接口要求）。

---

## 3. 硬件接口设计要点

文件：[`include/mycobot280_moveit2_control/mycobot280_hardware_interface.hpp`](include/mycobot280_moveit2_control/mycobot280_hardware_interface.hpp) 与 [`src/mycobot280_hardware_interface.cpp`](src/mycobot280_hardware_interface.cpp)

### 3.1 继承体系

```cpp
class MyCobot280HardwareInterface : public hardware_interface::SystemInterface;
```

`SystemInterface`（Jazzy 起 `hardware_interface::SystemInterface`，等价于旧版 `SystemInterface`）提供：

- `on_init(params)` — 从 URDF `<param>` 读取 `cmd_topic`、`state_topic`、`cmd_publish_rate`、`dry_run`
- `on_export_state_interfaces()` — 暴露 6 个 `position` 状态接口
- `on_export_command_interfaces()` — 暴露 6 个 `position` 指令接口
- `on_configure / on_cleanup / on_activate / on_deactivate` — 生命周期回调
- `read(time, period)` / `write(time, period)` — 控制循环每周期调用

### 3.2 URDF 可配置参数

通过 [`mycobot_description/urdf/control/mycobot_280_ros2_control.urdf.xacro`](../mycobot_description/urdf/control/mycobot_280_ros2_control.urdf.xacro) 中 `<xacro:unless value="${use_gazebo}">` 分支注入：

| 参数名                  | 默认值                          | 含义                                   |
|-------------------------|---------------------------------|----------------------------------------|
| `cmd_topic`             | `/mycobot/cmd_joint_pos`        | 发布手臂关节指令的话题                 |
| `state_topic`           | `/mycobot/current_joint_pos`    | 订阅手臂关节状态的话题                 |
| `gripper_cmd_topic`     | `/mycobot/cmd_gripper_pos`      | 发布夹爪开合指令的话题（0~100）        |
| `gripper_state_topic`   | `/mycobot/current_gripper_pos`  | 订阅夹爪状态的话题（0~100）            |
| `cmd_publish_rate`      | `50.0`                          | 发布指令的频率（Hz，手臂与夹爪共用）  |
| `dry_run`               | `false`                         | 为 `true` 时只读不写，适合离线调试    |

URDF 片段：

```xml
<xacro:unless value="${use_gazebo}">
  <plugin>mycobot280_moveit2_control/MyCobot280HardwareInterface</plugin>
  <param name="cmd_topic">/mycobot/cmd_joint_pos</param>
  <param name="state_topic">/mycobot/current_joint_pos</param>
  <param name="gripper_cmd_topic">/mycobot/cmd_gripper_pos</param>
  <param name="gripper_state_topic">/mycobot/current_gripper_pos</param>
  <param name="cmd_publish_rate">50.0</param>
  <param name="dry_run">false</param>

  <!-- 6 个手臂关节：command/state 均为 position (rad) -->
  <joint name="${prefix}link1_to_link2">...</joint>
  ...
  <joint name="${prefix}link6_to_link6_flange">...</joint>

  <!-- 夹爪关节：0..100 开合度，pymycobot 原生单位 -->
  <joint name="${prefix}gripper_controller">
    <command_interface name="position">
      <param name="min">0.0</param>
      <param name="max">100.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>  <!-- 占位，恒为 0 -->
  </joint>
</xacro:unless>
```

### 3.3 实时性与线程安全

- `state_callback` / `gripper_state_callback` 在 ROS executor 线程被调用，分别写入 `latest_state_` / `latest_gripper_state_`；`read()` 在控制循环线程被调用，通过 `latest_state_mutex_` / `latest_gripper_state_mutex_` 安全读取。
- `write()` 仅把 `hw_commands_position_` 拷贝到 `pending_cmd_`（带 `pending_cmd_mutex_`），把 `hw_commands_position_[gripper]` 拷贝到 `pending_gripper_cmd_`（带 `pending_gripper_cmd_mutex_`），不在控制循环里直接 `publish`，以避免阻塞。
- 一个独立的 `wall_timer` 按 `cmd_publish_rate` 唤醒，使用 `realtime_tools::RealtimePublisher` 非阻塞地把 `pending_cmd_` 发布到 `cmd_topic_`、把 `pending_gripper_cmd_` 发布到 `gripper_cmd_topic_`。
- `on_activate` 时把 `latest_state_` 复制到 `pending_cmd_` 与 `hw_commands_position_`（手臂部分），把 `latest_gripper_state_` 复制到 `pending_gripper_cmd_` 与 `hw_commands_position_[gripper]`，防止激活瞬间机械臂/夹爪跳变。

### 3.4 角度与夹爪单位

- 手臂关节：ros2_control 侧统一使用 **弧度（rad）**。`mycobot_driver_node.py` 在订阅回调里把 rad 转成 deg 再调用 `mc.send_angles(deg, 30)`，并在 50 Hz 定时器里把 `mc.get_angles()` 的 deg 转回 rad 发布。本 C++ 接口不做任何单位换算。
- 夹爪：使用 **0~100 的开合度**（pymycobot 原生单位，0 = 完全闭合，100 = 完全张开）。`mycobot_driver_node.py` 在 `gripper_cmd_callback` 里取整后直接调用 `mc.set_gripper_value(value, 30)`，并在 50 Hz 定时器里把 `mc.get_gripper_value()` 原样发布到 `current_gripper_pos`。本 C++ 接口不做任何单位换算。
- 夹爪的 `velocity` state interface 在 C++ 端始终返回 0（占位），因为 Python 驱动不报告速度。`GripperActionController` 仅用 `position` 做闭环，`velocity` 仅为满足接口声明要求。

---

## 4. 控制器配置

文件：[`config/mycobot_280_real_controllers.yaml`](config/mycobot_280_real_controllers.yaml)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    gripper_action_controller:
      type: position_controllers/GripperActionController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints:
      - link1_to_link2
      - link2_to_link3
      - link3_to_link4
      - link4_to_link5
      - link5_to_link6
      - link6_to_link6_flange
    command_interfaces: [position]
    state_interfaces: [position]
    open_loop_control: false
    allow_integration_in_goal_trajectories: true
    allow_nonzero_velocity_at_trajectory_end: true
    goal: [0.05]   # rad
    path: [0.05]

gripper_action_controller:
  ros__parameters:
    joint: gripper_controller
    action_monitor_rate: 20.0
    goal_tolerance: 1.0      # 1 unit out of 100
    max_effort: 100.0
    allow_stalling: false
    stall_velocity_threshold: 0.001
    stall_timeout: 1.0
```

- `arm_controller`：接收 MoveIt2 的 FollowJointTrajectory 目标，按 100 Hz 把插值后的位置指令写入硬件接口的 6 个手臂 command_interfaces。
- `gripper_action_controller`：暴露 `control_msgs/GripperCommand` action 到 `/gripper_action_controller/gripper_cmd`，把 goal.position（0~100）写入 `gripper_controller/position` command_interface。MoveIt2 或脚本可通过此 action 控制夹爪开合。`goal_tolerance: 1.0` 表示 1 个单位（满分 100）的容差，对开关型夹爪足够。
- `joint_state_broadcaster`：把硬件接口暴露的所有 state_interfaces（6 个手臂 position + 1 个夹爪 position + 1 个夹爪 velocity）发布到 `/joint_states`，供 MoveIt2 与 RViz 显示真实位姿。

> 📌 `position_controllers/GripperActionController` 在 Jazzy 已标记为 deprecated，推荐使用 `parallel_gripper_controllers/GripperActionController`。本包为兼容已存在的配置仍使用旧版；切换到新版只需改 `controller_manager` 中的 `type` 字段。
>
> 📌 与仿真侧 [`mycobot_moveit_config/config/mycobot_280/ros2_controllers.yaml`](../mycobot_moveit_config/config/mycobot_280/ros2_controllers.yaml) 的差异：本配置针对真机硬件接口，`gripper_action_controller` 控制的是 `gripper_controller` 这个 0~100 开合度关节，而非仿真侧的 `gripper_joint1`/`gripper_joint2` 物理关节。

---

## 5. 构建与安装

### 5.1 依赖

`package.xml` 中声明：

- `buildtool_depend`: `ament_cmake`
- `depend`: `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, `realtime_tools`, `std_msgs`
- `exec_depend`: `control_msgs`, `trajectory_msgs`, `gripper_action_controller`, `joint_trajectory_controller`, `joint_state_broadcaster`（用于运行 `scripts/test_arm_movement.py`）
- `test_depend`: `ament_lint_auto`, `ament_lint_common`

间接依赖：

- `mycobot_description`（提供 URDF xacro）
- `mycobot_moveit_config`（提供 `move_group.launch.py` 与 RViz 配置）
- `joint_trajectory_controller`, `joint_state_broadcaster`（ros2_controllers）
- `robot_state_publisher`, `controller_manager`
- Python 端：`pymycobot >= 3.6.1`（`mycobot_driver_node.py` 使用）

### 5.2 编译

本工程位于 `mycobot_ros2-main` 工作区根目录（与 `mycobot_description`、`mycobot_moveit_config` 等同级）。

```bash
cd /home/hl/mycobot/mycobot_ros2-main
source /opt/ros/jazzy/setup.bash
# 可选：先 source 已有 install，避免重新构建依赖
source install/setup.bash 2>/dev/null

colcon build --packages-select mycobot280_moveit2_control
# 修改 URDF 后需重建 mycobot_description：
# colcon build --packages-select mycobot_description
```

构建完成后，`mycobot_driver_node.py` 会被安装为 ROS 2 可执行程序（位于 `install/mycobot280_moveit2_control/lib/mycobot280_moveit2_control/`），可以直接用 `ros2 run` 启动，无需手写 `python3 /abs/path/...`。

开发期推荐使用符号链接安装，便于修改脚本/launch 后立即生效：

```bash
colcon build --packages-select mycobot280_moveit2_control --symlink-install
```

### 5.3 lint / 单元测试

```bash
colcon test --packages-select mycobot280_moveit2_control
colcon test-result --verbose --test-result-base build/mycobot280_moveit2_control
```

预期输出：`Summary: 16 tests, 0 errors, 0 failures, 2 skipped`（cppcheck / flake8 / pep257 / lint_cmake / uncrustify / xmllint 全部通过）。

### 5.4 验证插件可被 pluginlib 发现

```bash
source install/setup.bash
# ament 索引
cat install/mycobot280_moveit2_control/share/ament_index/resource_index/hardware_interface__pluginlib__plugin/mycobot280_moveit2_control
# 应输出：share/mycobot280_moveit2_control/mycobot280_moveit2_control.xml
```

---

## 6. 启动与使用

### 6.1 启动参数

文件 [`launch/mycobot_280_real_moveit.launch.py`](launch/mycobot_280_real_moveit.launch.py)，使用 `ros2 launch ... --show-args` 查看：

| 参数                 | 默认值                  | 含义                                       |
|----------------------|-------------------------|--------------------------------------------|
| `robot_name`         | `mycobot_280`           | 选择 MoveIt 配置子目录                     |
| `use_rviz`           | `true`                  | 是否启动 RViz2                             |
| `use_move_group`     | `true`                  | 是否启动 MoveIt2 `move_group` 节点         |
| `use_sim_time`       | `false`                 | 真机模式固定为 false                       |
| `prefix`             | `''`                    | 关节前缀                                   |
| `flange_link`        | `link6_flange`          | 法兰链接名                                 |
| `use_gripper`        | `true`                  | URDF 中是否挂载夹爪（仅视觉，不控制）      |
| `driver_port`        | `/dev/ttyACM0`          | 机械臂串口设备                             |
| `driver_baud`        | `115200`                | 波特率                                     |
| `rviz_config_file`   | `move_group.rviz`       | RViz 配置文件名                            |
| `rviz_config_package`| `mycobot_moveit_config` | RViz 配置所在包                            |

> 注：Python 驱动 `mycobot_driver_node.py` 已纳入本包的 `src/` 目录并通过 `CMakeLists.txt` 安装为 `lib/<pkg>/mycobot_driver_node.py` 可执行程序。启动文件用 `Node(package='mycobot280_moveit2_control', executable='mycobot_driver_node.py', ...)` 引用它，因此不再需要 `driver_script` 这种绝对路径参数。

启动流程（自动）：

1. `robot_state_publisher` 发布 `/robot_description`（xacro `use_gazebo:=false`）
2. `controller_manager`（`ros2_control_node`）订阅 robot_description，加载并激活 `MyCobot280HardwareInterface` 插件
3. `mycobot_driver_node.py`（通过 `ros2 run`/`Node` 启动，传入 `port`、`baudrate` 参数）连接真机
4. 3 秒延时后用 `ros2 control load_controller --set-state active` 顺序加载：`joint_state_broadcaster` → 2 秒后 `arm_controller` → 1 秒后 `gripper_action_controller`
5. 包含 `mycobot_moveit_config/launch/move_group.launch.py` 启动 MoveIt2
6. （可选）启动 RViz2（由 `move_group.launch.py` 在 `use_rviz:=true` 时启动）

### 6.2 启动命令

```bash
# 终端 1
cd /home/hl/mycobot/mycobot_ros2-main
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 默认 /dev/ttyACM0
ros2 launch mycobot280_moveit2_control mycobot_280_real_moveit.launch.py

# 或指定串口/波特率
ros2 launch mycobot280_moveit2_control mycobot_280_real_moveit.launch.py \
    driver_port:=/dev/ttyUSB0 driver_baud:=115200

# 不启动 RViz / MoveIt2，仅起 ros2_control + 驱动
ros2 launch mycobot280_moveit2_control mycobot_280_real_moveit.launch.py \
    use_rviz:=false use_move_group:=false
```

### 6.3 启动后核验

```bash
# 控制器列表（三者均应为 active）
ros2 control list_controllers
# 预期：
#  arm_controller            joint_trajectory_controller/JointTrajectoryController  active
#  gripper_action_controller position_controllers/GripperActionController          active
#  joint_state_broadcaster   joint_state_broadcaster/JointStateBroadcaster        active

# 硬件接口（gripper 的 position/velocity 都应出现）
ros2 control list_hardware_interfaces
# 预期 command interfaces 中包含 gripper_controller/position [available] [claimed]
# 预期 state    interfaces 中包含 gripper_controller/position 与 gripper_controller/velocity

# 关节状态是否在更新（手动转动真机 / 捏夹爪，数值应随之变化）
ros2 topic echo /joint_states
# name 数组中应同时出现 6 个手臂关节 + gripper_controller

# Python 驱动是否在发布手臂/夹爪状态
ros2 topic hz /mycobot/current_joint_pos
ros2 topic hz /mycobot/current_gripper_pos

# 硬件接口是否在发布指令（仅在收到轨迹/夹爪目标后）
ros2 topic echo /mycobot/cmd_joint_pos
ros2 topic echo /mycobot/cmd_gripper_pos
```

### 6.4 在 RViz 中用 MoveIt 规划并执行

1. 启动上述 launch 文件（`use_rviz:=true` 默认即开）
2. 在 RViz 的 MotionPlanning 显示里拖动末端球或点击 `Plan` 规划
3. 点击 `Execute`，`move_group` 会把轨迹作为 FollowJointTrajectory 目标发送到 `/arm_controller/follow_joint_trajectory`，进而通过硬件接口下发到真机
4. 真机开始运动，同时 RViz 中的模型随 `/joint_states` 同步更新
5. 夹爪可通过 7.2 命令行或 7.1 测试脚本单独控制（MoveIt2 PlanningScene 默认未集成夹爪 planning group）

### 6.5 在真机上运行 MTC pick & place demo

文件 [`launch/mycobot_280_real_pick_place.launch.py`](launch/mycobot_280_real_pick_place.launch.py) 把真机控制栈与 [`mycobot_mtc_pick_place_demo`](../mycobot_mtc_pick_place_demo) 的 `mtc_node` 串联起来，使 MTC 规划的抓取-放置任务直接驱动真实机械臂与夹爪。

启动：

```bash
ros2 launch mycobot280_moveit2_control mycobot_280_real_pick_place.launch.py
```

可选参数：

| 参数                | 默认值      | 含义                                                       |
|---------------------|-------------|------------------------------------------------------------|
| `robot_name`        | `mycobot_280` | 选择 MoveIt 配置子目录                                   |
| `use_rviz`          | `true`      | 是否启动 RViz2                                             |
| `use_move_group`    | `true`      | 是否启动 `move_group`（提供 PlanningScene 监控与运动学）  |
| `driver_port`       | `/dev/ttyACM0` | 串口设备路径                                            |
| `driver_baud`       | `115200`    | 波特率                                                     |
| `mtc_start_delay`   | `8.0`       | 延迟多少秒后启动 `mtc_node`（等控制器激活）              |
| `mtc_exe`           | `mtc_node`  | MTC demo 可执行程序名                                      |
| `use_perception`    | `false`     | 是否启用真实点云感知（详见下文"感知模式"）                |

工作原理：

1. `IncludeLaunchDescription` 启动 [`mycobot_280_real_moveit.launch.py`](launch/mycobot_280_real_moveit.launch.py)，带起 `robot_state_publisher` / `controller_manager` / `mycobot_driver_node.py` / `move_group` / RViz2，并按 6.3 的链路顺序激活 `joint_state_broadcaster` / `arm_controller` / `gripper_action_controller`。
2. 立即启动一个 `get_planning_scene_mycobot` service 提供方（见下文"感知模式"），让 service 在 `mtc_node` 启动前就 advertised。
3. 延迟 `mtc_start_delay` 秒后，用 `OpaqueFunction` 启动 `mtc_node`，其参数构造与 [`mycobot_mtc_pick_place_demo/launch/pick_place_demo.launch.py`](../mycobot_mtc_pick_place_demo/launch/pick_place_demo.launch.py) 一致（`MoveItConfigsBuilder` 加载 SRDF / 运动学 / joint_limits / moveit_controllers / planning_pipelines / `mtc_node_params.yaml`），但强制 `use_sim_time:=false`。
4. `mtc_node` 启动后立即 `setupPlanningScene()` + `doTask()`：MTC 用 `task_.execute(*solution)` 直接通过 `trajectory_execution` manager 把规划好的轨迹下发到对应 controller（`arm_controller` 走 FollowJointTrajectory action、`gripper_action_controller` 走 GripperCommand action）。**此路径不经过 `move_group`**，`move_group` 仅用于 PlanningScene 监控和运动学求解服务。
5. RViz2 可在 `MTC Solution` 显示里看到规划好的子轨迹（注意：`mtc_node_params.yaml` 默认 `execute: true`，规划成功后会立刻执行）。

#### 感知模式（use_perception）

`mtc_node` 在 `setupPlanningScene()` 中会调用 `get_planning_scene_mycobot` service 来获取场景中的物体信息。本 launch 通过 `use_perception` 参数在两种 service 提供方之间切换：

| `use_perception` | 启动的节点                                                           | 行为                                                                                                                  |
|------------------|----------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------|
| `false`（默认） | [`scripts/fake_planning_scene_server.py`](scripts/fake_planning_scene_server.py) | 立即响应 service，返回 1 个硬编码的 CollisionObject（id = `object_name`，frame = `object_reference_frame`，pose/dims 来自 `mtc_node_params.yaml` 的 `object_pose`/`object_dimensions`）。**无需相机**，适合真机无感知测试。 |
| `true`           | `mycobot_mtc_pick_place_demo/get_planning_scene_server.launch.py`   | 真实感知：订阅 `/camera_head/depth/color/points` 点云，跑平面/物体分割，返回场景中所有 collision objects。**需要相机驱动在跑**。            |

两个节点共用同一份 `mtc_node_params.yaml`，所以编辑其中的 `object_name` / `object_type` / `object_dimensions` / `object_pose` / `object_reference_frame` 会同时配置 mtc_node 的目标参数和 fake server 返回的硬编码物体。例如改成自己桌面上实际物体的位姿后，`use_perception:=false` 也能让 MTC 在指定位置规划抓取。

> 📌 `mtc_node_params.yaml` 中 `controller_names` 同时列出了 `arm_controller` 与 `gripper_action_controller`，这正是 MTC 把每个 stage 的轨迹路由到对应 controller 的依据。
>
> ⚠️ **执行时机**：`mtc_node` 一启动就立即规划并执行。如果 `mtc_start_delay` 太短导致 controller 还没 active，会在 `task_.execute()` 阶段报 action server unavailable。把 `mtc_start_delay` 调大到 10~15 秒可解决。
>
> ⚠️ **真机安全**：MTC 规划默认用 `mtc_node_params.yaml` 中的目标位姿（`object_pose: [0.22, 0.12, 0.0, ...]`、`place_pose: [-0.183, -0.14, 0.0, ...]`），首次运行前请确认这些位姿在真机可达范围内、不会撞桌/撞自身。可先把 `mtc_node_params.yaml` 中 `execute: true` 改为 `false`，在 RViz 里检查 MTC Solution 后再改回 `true` 执行。

---

## 7. 测试

### 7.1 端到端测试脚本

文件 [`scripts/test_arm_movement.py`](scripts/test_arm_movement.py)，作为可执行程序安装：

```bash
ros2 run mycobot280_moveit2_control test_arm_movement.py
```

行为参考了 `mycobot_system_tests/scripts/arm_gripper_loop_controller.py`（针对 Gazebo 仿真），同时发手臂轨迹目标和夹爪开合目标：

1. 等待 `/arm_controller/follow_joint_trajectory` 与 `/gripper_action_controller/gripper_cmd` 两个 action server 上线（最多各 10 秒）
2. 张开夹爪（`GRIPPER_OPEN = 80`，单位 0~100）
3. 发送 `TARGET_POS = [0.5, -0.4, 0.3, -0.3, 0.5, -0.5]`（rad），时长 4 秒
4. 闭合夹爪（`GRIPPER_CLOSE = 0`）
5. 发送 `HOME_POS = [0, 0, 0, 0, 0, 0]`（rad）回到零位
6. 在零位再次张开夹爪
7. 两次手臂目标都返回 `SUCCESSFUL` 即视为测试通过；夹爪失败仅警告（不影响手臂判定）

> ⚠️ 测试前请确保机械臂周围有足够空间，并且已经过标定（不要让 `TARGET_POS` 触发限位或自碰）。如有需要，可直接修改脚本里的 `TARGET_POS`、`GRIPPER_OPEN`。

### 7.2 命令行单点测试

也可以直接用 `ros2 action send_goal` 发一个最小轨迹或夹爪目标：

```bash
# 手臂单点轨迹
ros2 action send_goal /arm_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    "{trajectory: {
        joint_names: [link1_to_link2, link2_to_link3, link3_to_link4,
                      link4_to_link5, link5_to_link6, link6_to_link6_flange],
        points: [{positions: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                  time_from_start: {sec: 2}}]
    }}"

# 夹爪开合（80 = 张开，0 = 闭合）
ros2 action send_goal /gripper_action_controller/gripper_cmd \
    control_msgs/action/GripperCommand \
    "{command: {position: 80.0, max_effort: 100.0}}"
```

预期返回 `Goal finished with status: SUCCEEDED`，并且另一终端 `ros2 topic echo /mycobot/cmd_joint_pos --once` 能看到 `data: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]`，`ros2 topic echo /mycobot/cmd_gripper_pos --once` 能看到 `data: [80.0]`。

### 7.3 离线（无真机）烟雾测试

如需在未连接真机时验证插件链路：

```bash
# 1) 准备 URDF
xacro $(ros2 pkg prefix mycobot_description --share)/urdf/robots/mycobot_280.urdf.xacro \
    use_gazebo:=false > /tmp/mycobot_280_real.urdf

# 2) 启动 robot_state_publisher + controller_manager（不启动 Python 驱动）
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat /tmp/mycobot_280_real.urdf)" &
ros2 run controller_manager ros2_control_node \
    --ros-args --params-file $(ros2 pkg prefix mycobot280_moveit2_control --share)/share/mycobot280_moveit2_control/config/mycobot_280_real_controllers.yaml \
    -p robot_description:="$(cat /tmp/mycobot_280_real.urdf)" &

# 3) 顺序加载控制器
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active arm_controller
ros2 control load_controller --set-state active gripper_action_controller

# 4) 发送轨迹 / 夹爪目标（见 7.2）
```

由于没有 Python 驱动，硬件接口收不到 `current_joint_pos` / `current_gripper_pos`，状态保持为 0；但控制器仍会接收目标、桥接出 `/mycobot/cmd_joint_pos` 与 `/mycobot/cmd_gripper_pos`，可验证插件能否成功加载、激活并对外发布指令。

---

## 8. 常见问题

### 8.1 `Loaded hardware 'RobotSystem' from plugin 'mycobot280_moveit2_control/MyCobot280HardwareInterface'` 没出现

- 确认 install 后已 `source install/setup.bash`
- 确认 `mycobot_description` 已重建（URDF 改动）
- 用 `xacro ... use_gazebo:=false | grep plugin` 确认 URDF 中 `<plugin>` 标签已切换到 `mycobot280_moveit2_control/MyCobot280HardwareInterface`

### 8.2 `Unable to activate controller 'gripper_action_controller' since the state interface 'gripper_controller/velocity' is not available`

`GripperActionController` 要求夹爪关节同时提供 `position` 和 `velocity` 两个 state interface。请在 URDF 的 `gripper_controller` 关节中加上 `<state_interface name="velocity"/>`，并确保 C++ 硬件接口在 `on_export_state_interfaces()` 中为夹爪导出 velocity（本包已默认如此，返回恒为 0 的占位值）。

### 8.3 `Goal finished with status: ABORTED` 且 `/mycobot/cmd_joint_pos` 无数据

- 检查 Python 驱动是否成功连接真机：查看 `mycobot_driver` 节点日志是否输出 `MyCobot connected successfully.`
- 检查串口权限：`sudo chmod 666 /dev/ttyACM0` 或把用户加入 `dialout` 组
- 检查 `dry_run` 是否被设为 `true`（为 true 时硬件接口不会发布指令）

### 8.4 真机不动但 `/mycobot/cmd_joint_pos` 有数据

- 多半是 Python 驱动收到指令但未能下发：检查 `mycobot_driver_node.py` 日志是否有 `Error sending angles` 或 `Expected 6 joints`
- 确认 `pymycobot` 版本 `>= 3.6.1`：`python3 -c "import pymycobot; print(pymycobot.__version__)"`

### 8.5 夹爪不动但 `/mycobot/cmd_gripper_pos` 有数据

- 检查 Python 驱动日志是否有 `Error sending gripper value`
- 检查 `gripper_action_controller` 是否 `active`：`ros2 control list_controllers`
- 检查夹爪目标值是否在 0~100 范围内（超出会被 URDF 的 `min`/`max` 限幅，但仍可能拒绝）
- 部分夹爪硬件需要先 `set_gripper_ini()` 初始化；如长期异常，可在 `mycobot_driver_node.py` 的 `__init__` 末尾加上 `self.mc.set_gripper_ini()`

### 8.6 关节方向"反向"或 RViz 不跟随真机

- 确认 `joint_state_broadcaster` 已 `active`：`ros2 control list_controllers`
- 确认 Python 驱动 50 Hz 定时器在跑：`ros2 topic hz /mycobot/current_joint_pos` 应在 ~50 Hz
- 确认 URDF 与控制器配置的关节名一致（均为 `link1_to_link2` … `link6_to_link6_flange`，无前缀）

---

## 9. 设计取舍

- **为何用话题而非 action/service 桥接？** 话题是 `mycobot_driver_node.py` 已有的接口，且 `Float64MultiArray` 足以表达手臂关节位置或夹爪开合度，省去额外的序列化开销。代价是无法拿到"轨迹执行完成"的细粒度反馈，但这由 `JointTrajectoryController` / `GripperActionController` 自己评估。
- **为何把 publish 放在 wall_timer 而非 `write()` 里？** 真实控制循环（`update_rate: 100` Hz）对延迟敏感，`publish` 可能因 DDS 内部锁阻塞，放在非 RT 线程更安全。
- **为何 `on_activate` 用 `latest_state_` 作为指令初值？** 避免 `joint_trajectory_controller` 一上来就收到 `0` 而把机械臂拉回零位，造成意外运动。夹爪同理，用 `latest_gripper_state_` 作初值。
- **为何夹爪也纳入同一个硬件插件，而不是单独写一个 gripper 插件？** ros2_control 的 `SystemInterface` 允许在一个插件内同时管理多个关节（手臂 + 夹爪），把夹爪与手臂放一起可以复用 wall_timer、publisher、状态回调线程的初始化逻辑，简化 launch 文件（只需启动一次 `controller_manager`）。代价是夹爪的单位与手臂不同（0~100 vs rad），需要 C++ 端不做换算、交给 Python 驱动分别用 `set_gripper_value` / `set_angles` 处理。
- **为何夹爪的 `velocity` state 是恒 0 的占位值？** `GripperActionController` 在 Jazzy 实现里强制要求夹爪关节暴露 `position` + `velocity` 两个 state interface，否则激活时报 `velocity is not available`。Python 驱动只报告夹爪位置（`get_gripper_value`），没有速度反馈，因此 C++ 端用静态 0 满足接口声明，控制器实际只用 position 做闭环。
- **为何夹爪用 `position_controllers/GripperActionController` 而非更新的 `parallel_gripper_controllers/GripperActionController`？** 旧版仍可用且配置参数兼容；新版接口签名相同，未来切换只需改 `controller_manager` 中的 `type` 字段。
