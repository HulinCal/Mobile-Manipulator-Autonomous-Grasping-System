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