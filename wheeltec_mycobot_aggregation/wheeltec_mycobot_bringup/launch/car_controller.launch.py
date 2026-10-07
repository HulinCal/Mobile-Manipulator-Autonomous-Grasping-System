#!/usr/bin/env python3
"""
启动聚合机器人(WHEELTEC R680 + myCobot 280)的底盘控制器栈。

该 launch 文件在 turn_on_wheeltec_robot.launch.py 基础上改用聚合 URDF,
启动 WHEELTEC R680 底盘的串口驱动与 EKF 状态估计:

  1. robot_state_publisher   - 发布聚合机器人 /robot_description (use_gazebo:=false)
                               (聚合 URDF 已内嵌 base_footprint->base_link, 无需
                               额外的 base_to_link 静态 TF)
  2. base_serial             - WHEELTEC 底盘串口驱动 (复用 turn_on_wheeltec_robot 的
                               base_serial.launch.py, akmcar:=false)
  3. base_to_gyro            - 静态 TF: base_link -> gyro_link (单位变换)
  4. base_to_laser           - 静态 TF: base_footprint -> laser
                               (xyz='0.0911 0 0.155' rpy='0 0 0')
  5. robot_ekf               - EKF 融合节点 (配置: turn_on_wheeltec_robot/config/ekf.yaml)
                               (carto_slam:=true 时禁用, 默认 false)
  6. joint_state_publisher   - 无头模式发布关节状态 (真实硬件提供关节状态)
  7. rviz2                  - 使用 wheeltec_mycobot_car_controller.rviz

用法:
    ros2 launch wheeltec_mycobot_bringup car_controller.launch.py
    ros2 launch wheeltec_mycobot_bringup car_controller.launch.py \
        use_rviz:=false carto_slam:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'wheeltec_mycobot_bringup'
    description_package = 'wheeltec_mycobot_description'
    xacro_filename = 'wheeltec_mycobot.urdf.xacro'
    rviz_filename = 'wheeltec_mycobot_car_controller.rviz'

    # 聚合机器人 xacro / rviz 路径 (惰性求值)
    urdf_xacro = PathJoinSubstitution(
        [FindPackageShare(description_package), 'urdf', xacro_filename])
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'rviz', rviz_filename])
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('turn_on_wheeltec_robot'), 'config', 'ekf.yaml'])

    # --- Launch 参数 ------------------------------------------------------
    use_rviz = LaunchConfiguration('use_rviz')
    carto_slam = LaunchConfiguration('carto_slam')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='是否启动 RViz2')
    declare_carto_slam_cmd = DeclareLaunchArgument(
        name='carto_slam',
        default_value='false',
        description='是否使用 Cartographer SLAM (为 true 时禁用 EKF)')

    # --- 通过 xacro 构建聚合机器人描述 (use_gazebo:=false, use_gripper:=true) ---
    robot_description_content = ParameterValue(
        Command([
            'xacro', ' ', urdf_xacro, ' ',
            'use_gazebo:=false', ' ',
            'use_gripper:=true',
        ]),
        value_type=str,
    )

    # 1) robot_state_publisher - 发布 /robot_description 与 TF
    #    (聚合 URDF 已内嵌 base_footprint->base_link, 不再需要 base_to_link)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }],
    )

    # 2) WHEELTEC 底盘串口驱动 (复用 base_serial.launch.py, akmcar:=false)
    base_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('turn_on_wheeltec_robot'),
                 'launch', 'base_serial.launch.py'])
        ),
        launch_arguments={'akmcar': 'false'}.items(),
    )

    # 3) base_to_gyro 静态 TF: base_link -> gyro_link (单位变换)
    base_to_gyro_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'gyro_link',
        ],
    )

    # 4) base_to_laser 静态 TF: base_footprint -> laser
    #    (与 senior_4wd_bs_robot 一致: xyz='0.0911 0 0.155' rpy='0 0 0')
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
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'laser',
        ],
    )

    # 5) EKF 融合节点 (carto_slam:=true 时禁用, 默认 false)
    robot_ekf_node = Node(
        condition=UnlessCondition(carto_slam),
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odom_combined')],
    )

    # 6) joint_state_publisher (无头模式, 真实硬件提供关节状态)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # 7) RViz2 (默认启动), 使用本包的底盘控制器专用配置
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
    ld.add_action(declare_carto_slam_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(base_serial_launch)
    ld.add_action(base_to_gyro_node)
    ld.add_action(base_to_laser_node)
    ld.add_action(robot_ekf_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    return ld
