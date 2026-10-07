#!/usr/bin/env python3
"""
启动聚合机器人(WHEELTEC R680 + myCobot 280)的激光雷达可视化。

该 launch 文件启动:
  1. robot_state_publisher  - 发布聚合机器人的 /robot_description (use_gazebo:=false)
  2. joint_state_publisher  - (默认启动) 无头关节状态发布, 发布零位关节状态,
                              保证轮子与机械臂各 link 的 TF 完整显示
     joint_state_publisher_gui - (可选, 默认不启动) GUI 滑块, 用于手动拖动机械臂
                              关节调试, 启动方式: jsp_gui:=true (与无头版互斥)
  3. base_to_laser          - 静态 TF: base_footprint -> laser
                             (xyz='0.0911 0 0.155' rpy='0 0 0', 与 senior_4wd_bs_robot 一致)
  4. lslidar_driver         - 启动 LSLidar N10 驱动 (复用 lsn10_launch.py)
  5. rviz2                  - 使用 wheeltec_mycobot_lidar.rviz, 同时显示机器人模型与激光点

用法:
    ros2 launch wheeltec_mycobot_bringup lidar.launch.py
    ros2 launch wheeltec_mycobot_bringup lidar.launch.py use_rviz:=false
    ros2 launch wheeltec_mycobot_bringup lidar.launch.py jsp_gui:=true
    ros2 launch wheeltec_mycobot_bringup lidar.launch.py use_jsp:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'wheeltec_mycobot_bringup'
    description_package = 'wheeltec_mycobot_description'
    xacro_filename = 'wheeltec_mycobot.urdf.xacro'
    rviz_filename = 'wheeltec_mycobot_lidar.rviz'

    # 聚合机器人 xacro 路径 (惰性求值, 即使描述包未安装也不会在解析时报错)
    urdf_xacro = PathJoinSubstitution(
        [FindPackageShare(description_package), 'urdf', xacro_filename])
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'rviz', rviz_filename])

    # --- Launch 参数 ------------------------------------------------------
    use_rviz = LaunchConfiguration('use_rviz')
    use_jsp = LaunchConfiguration('use_jsp')
    jsp_gui = LaunchConfiguration('jsp_gui')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='是否启动 RViz2')

    declare_use_jsp_cmd = DeclareLaunchArgument(
        name='use_jsp',
        default_value='true',
        description='是否启动无头 joint_state_publisher (发布零位, 保证模型完整显示)')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        # 默认不启动 GUI 滑块, 需要手动调试关节时通过 jsp_gui:=true 开启
        default_value='false',
        description='是否启动 joint_state_publisher_gui 关节滑块界面 (默认不启动, '
                    '启动时替代无头 joint_state_publisher)')

    # --- 通过 xacro 构建机器人描述 (use_gazebo:=false, use_gripper:=true) ---
    robot_description_content = ParameterValue(
        Command([
            'xacro', ' ', urdf_xacro, ' ',
            'use_gazebo:=false', ' ',
            'use_gripper:=true',
        ]),
        value_type=str,
    )

    # 1) robot_state_publisher - 发布 /robot_description 与 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }],
    )

    # 2a) joint_state_publisher (无头, 默认启动)
    #     发布零位关节状态, 使轮子与机械臂各转动关节下游 link 的 TF 完整,
    #     RViz 才能显示完整机器人模型。jsp_gui:=true 时不启动 (由 GUI 版替代)。
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", use_jsp, "' == 'true' and '", jsp_gui, "' != 'true'"])),
    )

    # 2b) joint_state_publisher_gui - (可选, jsp_gui:=true 时启动)
    #    无控制器时可通过 GUI 滑块拖动机械臂关节, 默认不启动
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(jsp_gui),
    )

    # 3) base_to_laser 静态 TF: base_footprint -> laser
    #    与 turn_on_wheeltec_robot/launch/robot_mode_description.launch.py 中
    #    senior_4wd_bs_robot 的 base_to_laser 保持一致 (xyz='0.0911 0 0.155')
    base_to_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=[
            '--x', '0.0911',
            '--y', '0.0',
            '--z', '0.155',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    # 4) LSLidar N10 驱动 (复用 lslidar_driver 包的 lsn10_launch.py)
    lsn10_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('lslidar_driver'), 'launch', 'lsn10_launch.py'])
        ),
    )

    # 5) RViz2 (默认启动), 使用本包的激光雷达专用配置
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    # --- 组装 -------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_jsp_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(base_to_laser_node)
    ld.add_action(lsn10_launch)
    ld.add_action(rviz_node)
    return ld
