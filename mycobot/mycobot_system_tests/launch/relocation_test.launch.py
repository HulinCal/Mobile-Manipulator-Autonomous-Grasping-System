#!/usr/bin/env python3
"""启动 mycobot280 重定位精度测试（含机械臂完整启动流程）。

参考 wheeltec_mycobot_pick_place/launch/pick_place.launch.py 的机械臂启动方式，
通过 include wheeltec_mycobot_bringup/arm_controller.launch.py 启动机械臂栈：
    1. robot_state_publisher   - 发布 /robot_description 与 TF
    2. controller_manager      - ros2_control + 硬件接口
    3. mycobot_driver_node.py  - Python 串口驱动（driver_port/driver_baud）
    4. spawn 控制器            - joint_state_broadcaster -> arm_controller
                                 -> gripper_action_controller
    5. joint_state_publisher   - 合并关节状态
    6. move_group              - MoveIt2（可选，本测试不需要，默认关闭）

启动后再向 /relocation_test/target 发布 6 个关节角即可让机械臂运动。

用法：
    ros2 launch mycobot_system_tests relocation_test.launch.py
    ros2 launch mycobot_system_tests relocation_test.launch.py \
        driver_port:=/dev/ttyACM2 use_rviz:=true
    # 已有机械臂栈在运行时，只起测试节点：
    ros2 launch mycobot_system_tests relocation_test.launch.py use_arm_bringup:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- 机械臂启动参数 -------------------------------------------------
    use_arm_bringup = LaunchConfiguration('use_arm_bringup')
    driver_port = LaunchConfiguration('driver_port')
    driver_baud = LaunchConfiguration('driver_baud')
    use_rviz = LaunchConfiguration('use_rviz')
    use_move_group = LaunchConfiguration('use_move_group')

    # --- 测试节点参数 ---------------------------------------------------
    angle_unit = LaunchConfiguration('angle_unit')
    num_random_positions = LaunchConfiguration('num_random_positions')
    joint_margin_deg = LaunchConfiguration('joint_margin_deg')
    min_joint_separation_deg = LaunchConfiguration('min_joint_separation_deg')
    move_duration_sec = LaunchConfiguration('move_duration_sec')
    settle_time_sec = LaunchConfiguration('settle_time_sec')
    seed = LaunchConfiguration('seed')
    arm_action = LaunchConfiguration('arm_action')

    declare_use_arm_bringup = DeclareLaunchArgument(
        'use_arm_bringup', default_value='true',
        description='是否启动机械臂栈(arm_controller.launch.py); '
                    '已有机械臂驱动在运行时设为 false 仅启动测试节点')
    declare_driver_port = DeclareLaunchArgument(
        'driver_port', default_value='/dev/mycobot_controller',
        description='机械臂串口设备路径(udev 规则名); 实测可用 /dev/ttyACM2 等覆盖')
    declare_driver_baud = DeclareLaunchArgument(
        'driver_baud', default_value='115200',
        description='机械臂串口波特率')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='是否启动 RViz2 (随 move_group 启动, use_move_group:=true 时有效)')
    declare_use_move_group = DeclareLaunchArgument(
        'use_move_group', default_value='false',
        description='是否启动 MoveIt2 move_group (重定位测试直接走 arm_controller, 不需要)')

    declare_angle_unit = DeclareLaunchArgument(
        'angle_unit', default_value='rad',
        description='目标话题的角度单位：rad 或 deg')
    declare_num_random = DeclareLaunchArgument(
        'num_random_positions', default_value='10',
        description='随机生成的可达目标数量')
    declare_margin = DeclareLaunchArgument(
        'joint_margin_deg', default_value='20.0',
        description='随机采样时距关节限位保留的余量（度）')
    declare_min_sep = DeclareLaunchArgument(
        'min_joint_separation_deg', default_value='20.0',
        description='随机目标之间的最小关节角差（度）')
    declare_duration = DeclareLaunchArgument(
        'move_duration_sec', default_value='4.0',
        description='单次运动时长（秒）')
    declare_settle = DeclareLaunchArgument(
        'settle_time_sec', default_value='1.0',
        description='到位后延迟采样实测值的等待时间（秒），'
                    '应对控制器早于机械臂停稳返回成功的情况')
    declare_seed = DeclareLaunchArgument(
        'seed', default_value='-1',
        description='随机种子，-1 表示每次不同')
    declare_arm_action = DeclareLaunchArgument(
        'arm_action', default_value='/arm_controller/follow_joint_trajectory',
        description='机械臂关节轨迹动作名')

    # --- 机械臂完整启动 (RSP + ros2_control + 驱动 + 控制器) ------------
    arm_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('wheeltec_mycobot_bringup'),
            '/launch/arm_controller.launch.py',
        ]),
        launch_arguments={
            'driver_port': driver_port,
            'driver_baud': driver_baud,
            'use_rviz': use_rviz,
            'use_move_group': use_move_group,
        }.items(),
        condition=IfCondition(use_arm_bringup),
    )

    # --- 重定位测试节点 ---------------------------------------------------
    relocation_test_node = Node(
        package='mycobot_system_tests',
        executable='relocation_test.py',
        name='relocation_test',
        output='screen',
        parameters=[{
            'angle_unit': angle_unit,
            'num_random_positions': ParameterValue(
                num_random_positions, value_type=int),
            'joint_margin_deg': ParameterValue(
                joint_margin_deg, value_type=float),
            'min_joint_separation_deg': ParameterValue(
                min_joint_separation_deg, value_type=float),
            'move_duration_sec': ParameterValue(
                move_duration_sec, value_type=float),
            'settle_time_sec': ParameterValue(
                settle_time_sec, value_type=float),
            'seed': ParameterValue(seed, value_type=int),
            'arm_action': arm_action,
        }],
    )

    info = LogInfo(msg=(
        '[relocation_test] 机械臂栈启动中，待 arm_controller 激活后，'
        '向 /relocation_test/target 发布 6 个关节角即可运动'))

    return LaunchDescription([
        declare_use_arm_bringup,
        declare_driver_port,
        declare_driver_baud,
        declare_use_rviz,
        declare_use_move_group,
        declare_angle_unit,
        declare_num_random,
        declare_margin,
        declare_min_sep,
        declare_duration,
        declare_settle,
        declare_seed,
        declare_arm_action,
        info,
        arm_bringup_launch,
        relocation_test_node,
    ])
