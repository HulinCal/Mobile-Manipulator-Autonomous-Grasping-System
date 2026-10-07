#!/usr/bin/env python3
"""
启动文件：控制真实机械臂执行抓取

启动内容：
1. 真实机械臂 bringup（robot_state_publisher + ros2_control + driver + move_group + RViz）
   —— 通过 include mycobot280_moveit2_control/mycobot_280_real_moveit.launch.py
2. grasp_node (订阅 /grasp_target_pose，稳定后让 MoveIt 规划并执行)

调试流程：
  步骤 1：先启动 grasp.launch.py，确认 ArUco marker 检测正常
            ros2 launch mycobot280_grasp_arcuo_markers grasp.launch.py
  步骤 2：再启动本文件，启动 MoveIt + 真实机械臂 + grasp_node
            ros2 launch mycobot280_grasp_arcuo_markers execute_grasp.launch.py

注意：grasp.launch.py 会发布 /grasp_target_pose，本文件的 grasp_node 订阅它。
两者必须在同一个 ROS_DOMAIN_ID 下运行（默认即可）。

使用方法：
    source /opt/ros/jazzy/setup.bash
    source /home/hl/mycobot/easy_handeye2_ws/install/setup.bash
    source /home/hl/mycobot/mycobot_ros2-main/install/setup.bash
    ros2 launch mycobot280_grasp_arcuo_markers execute_grasp.launch.py

可选参数：
    use_rviz:=false                 关闭 RViz
    driver_port:=/dev/ttyUSB0       串口
    driver_baud:=115200             波特率

:author hl
:date September 2026
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 参数 ----
    use_rviz       = LaunchConfiguration('use_rviz')
    use_move_group = LaunchConfiguration('use_move_group')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    driver_port    = LaunchConfiguration('driver_port')
    driver_baud    = LaunchConfiguration('driver_baud')

    declare_args = [
        DeclareLaunchArgument('use_rviz', default_value='true',
            description='启动 RViz'),
        DeclareLaunchArgument('use_move_group', default_value='true',
            description='启动 move_group (设为 false 用于已在外部启动的场景)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='使用仿真时钟 (真机设为 false)'),
        DeclareLaunchArgument('driver_port', default_value='/dev/ttyACM0',
            description='mycobot 串口'),
        DeclareLaunchArgument('driver_baud', default_value='115200',
            description='mycobot 波特率'),
    ]

    # ---- 1. 真实机械臂 bringup（含 robot_state_publisher + ros2_control + driver + move_group + RViz）----
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mycobot280_moveit2_control'),
                'launch', 'mycobot_280_real_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_move_group': use_move_group,
            'use_sim_time': use_sim_time,    # 关键：真机必须 false，否则 move_group 等 /clock
            'driver_port': driver_port,
            'driver_baud': driver_baud,
        }.items(),
    )

    # ---- 2. grasp_node (订阅 /grasp_target_pose，MoveIt 规划+执行) ----
    # 话题 /grasp_target_pose 由 grasp.launch.py 启动的 detect_markers_node 发布
    grasp_node = Node(
        package='mycobot280_grasp_arcuo_markers',
        executable='grasp_node',
        name='grasp_node',
        output='screen',
        # 参数可以通过命令行 --ros-args -p 覆盖，默认值在 C++ 源码中已声明
    )

    return LaunchDescription(
        declare_args + [
            arm_bringup,
            grasp_node,
        ]
    )
