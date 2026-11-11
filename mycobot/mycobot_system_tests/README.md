# mycobot_system_tests

Integration and system tests for verifying myCobot robot functionality.

## Overview

This package provides system-level integration tests for the myCobot robotic arm,
focusing on verifying joint movements, gripper operations, and coordinated actions.
The tests help ensure reliable operation of the robot in production environments.

## Python Example Test
```bash
ros2 run mycobot_system_tests arm_gripper_loop_controller.py
```

## C++ Example Test
```bash
ros2 run mycobot_system_tests arm_gripper_loop_controller
```

## 重定位精度测试（relocation_test）

用于评估 mycobot280 的重复定位精度：从不同位置回到同一目标位置时，实际到达位置的一致性。

节点行为：
1. 订阅 `/joint_states`，读到 6 个关节角后打印当前角度（度 + 弧度），便于人工量取实际位置；
2. 启动时在关节限位内随机生成 10 个可达目标并打印（度 + 弧度）；
3. 订阅 `/relocation_test/target`（`std_msgs/Float64MultiArray`，6 个关节角），
   通过 `/arm_controller/follow_joint_trajectory` 驱动机械臂，到位后打印实测关节角。

### 前置条件
launch 默认会 include `wheeltec_mycobot_bringup/arm_controller.launch.py` 自动启动机械臂
（RSP + ros2_control + 串口驱动 + 控制器 spawn），无需手动准备；最终需要以下接口可用：
- `/arm_controller/follow_joint_trajectory`
- `/joint_states`

### 使用
```bash
# 终端1：启动机械臂 + 测试节点（默认输入单位为弧度）
ros2 launch mycobot_system_tests relocation_test.launch.py

# 串口不是默认 /dev/mycobot_controller 时覆盖（实测机械臂在 /dev/ttyACM2 时）：
ros2 launch mycobot_system_tests relocation_test.launch.py driver_port:=/dev/ttyACM2

# 机械臂栈已在别处启动时，只起测试节点：
ros2 launch mycobot_system_tests relocation_test.launch.py use_arm_bringup:=false

# 终端2：发布目标位置（6 个关节角，顺序为 link1_to_link2 ... link6_to_link6_flange）
ros2 topic pub --once /relocation_test/target std_msgs/msg/Float64MultiArray \
  "{data: [-0.6984, 1.8264, 0.6159, 0.5468, 0.6399, -0.3467]}"
```

常用参数：
```bash
# 以“度”为单位发布，生成 10 个目标，单次运动 4 秒
ros2 launch mycobot_system_tests relocation_test.launch.py angle_unit:=deg
# 固定随机种子，便于复现同一组目标
ros2 launch mycobot_system_tests relocation_test.launch.py seed:=42
```

> 随机目标仅在关节限位内采样，不保证不自碰撞或不会碰到桌面；下发前请先确认姿态安全。

mycobot_system_tests （机械臂循环测试）
功能 ：让机械臂在"目标位置"和"Home 位置"之间循环运动，并在每个位置同步开/关夹爪，用于验证关节控制器和夹爪动作的可靠性。

### 前置条件
必须先启动 Gazebo + 控制器（或连接真实硬件），确保以下 action server 可用：

- /arm_controller/follow_joint_trajectory
- /gripper_action_controller/gripper_cmd
### 使用流程
# 终端1：启动 Gazebo 仿真（含控制器）
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash mycobot_bringup/scripts/mycobot_280_gazebo.sh
# 等待 Gazebo 完全启动后再继续

# 终端2：运行测试节点
source install/setup.bash
# Python 版本
ros2 run mycobot_system_tests arm_gripper_loop_controller.py
# 或 C++ 版本
ros2 run mycobot_system_tests arm_gripper_loop_controller
节点启动后机械臂会循环执行：移动到目标位 → 关夹爪 → 回 Home → 开夹爪，循环往复。 停止按 Ctrl+C 。
- 此包是独立的集成测试， 不需要 MoveIt 或 MTC ，仅依赖 ros2_control 的 action 接口
- 既可用于仿真，也可用于真实硬件（真实硬件需先启动 mycobot_bringup/scripts/mycobot_280.sh 等真实机器人启动脚本）