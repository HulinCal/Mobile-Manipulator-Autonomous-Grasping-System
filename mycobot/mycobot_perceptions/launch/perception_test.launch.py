#!/usr/bin/env python3
"""
End-to-end test for the mycobot_perceptions package.

This launch file brings up everything needed to verify the camera_perception
node end to end:

    1. (optional) Gazebo simulation with the RGB-D camera, plus topic relays that
       publish the simulated camera data onto the canonical topic names
       /camera/depth_registered/points and /camera/color/image_raw.
    2. The MoveIt 2 move_group node (from mycobot_moveit_config) so that the
       published PlanningSceneWorld can later be consumed by grasp planning.
    3. The camera_perception node (this package) with its YAML configuration.
    4. The perception_test_node, which subscribes to the perception output and
       logs statistics to the console.
    5. RViz preconfigured to display the published point clouds and markers.

Run with the defaults (Gazebo simulation):
    ros2 launch mycobot_perceptions perception_test.launch.py

Run against a *real* camera that already publishes the canonical topics:
    ros2 launch mycobot_perceptions perception_test.launch.py use_gazebo:=false

:author: hl
:date: August 2026
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'mycobot_perceptions'

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    config_file = os.path.join(pkg_share, 'config', 'camera_perception.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'perception_test.rviz')

    # Launch configuration variables
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # NOTE: 用独立参数名 `start_rviz` 而不是 `use_rviz`。因为 move_group.launch.py
    # (mycobot_moveit_config) 内部也声明了同名参数 `use_rviz`，IncludeLaunchDescription
    # 会共享外层参数上下文，导致 move_group 的 DeclareLaunchArgument 把外部
    # 传入的值重置为它自己的默认值（false），从而让本节点的 RViz IfCondition
    # 永远为 false，RViz 永不启动。改名后两参数互不干扰，move_group 始终拿到
    # 'false'（不在 move_group 内启 RViz），本节点的 RViz 由 start_rviz 独立控制。
    start_rviz = LaunchConfiguration('start_rviz')
    world_file = LaunchConfiguration('world_file')

    # ---- Declare arguments ----
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Launch the Gazebo simulation (with camera) for testing. '
                    'Set to false to use a real camera.')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='start_rviz',
        default_value='true',
        description='Whether to start RViz for this perception package. '
                     'Renamed from use_rviz to avoid clashing with the same '
                     'argument declared inside mycobot_moveit_config/move_group.launch.py, '
                     'which would otherwise reset our value to its own default.')

    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value='pick_and_place_demo.world',
        description='Gazebo world file')

    # ---- 1. Gazebo simulation (optional) ----
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare(package='mycobot_gazebo').find('mycobot_gazebo'),
                         'launch', 'mycobot.gazebo.launch.py')
        ),
        launch_arguments={
            'load_controllers': 'true',
            'world_file': world_file,
            'use_camera': 'true',
            'use_rviz': 'false',
            'use_robot_state_pub': 'true',
            'use_sim_time': use_sim_time,
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    # Relays that publish the Gazebo camera topics onto the canonical names the
    # perception node subscribes to. This keeps the node's topic contract fixed
    # while still allowing simulation testing.
    relay_pointcloud = Node(
        package='topic_tools',
        executable='relay',
        name='relay_depth_points',
        arguments=[
            '/camera_head/depth/color/points',
            '/camera/depth_registered/points',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_gazebo),
    )

    relay_color_image = Node(
        package='topic_tools',
        executable='relay',
        name='relay_color_image',
        arguments=[
            '/camera_head/color/image_raw',
            '/camera/color/image_raw',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_gazebo),
    )

    # ---- 2. move_group (from mycobot_moveit_config) ----
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare(package='mycobot_moveit_config').find('mycobot_moveit_config'),
                         'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': 'false',
        }.items(),
    )

    # ---- 3. camera_perception node ----
    # Remap the node's MoveIt-scene output topics onto the canonical names
    # move_group's PlanningSceneMonitor subscribes to. This way the perceived
    # CollisionObjects are applied to the live planning scene automatically:
    #   ~/planning_scene     -> /planning_scene        (PSM scene-diff input)
    #   ~/collision_object  -> /collision_object      (CollisionObject stream)
    camera_perception_node = Node(
        package=package_name,
        executable='camera_perception',
        name='camera_perception',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('~/planning_scene', '/planning_scene'),
            ('~/collision_object', '/collision_object'),
        ],
    )

    # ---- 4. perception_test_node ----
    perception_test_node = Node(
        package=package_name,
        executable='perception_test_node',
        name='perception_test_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---- 5. RViz ----
    # NOTE: 历史上这里把 rviz_node 塞进 TimerAction 并加 IfCondition(use_rviz)，
    # 但实测 RViz 进程从未启动（日志里没有 rviz2 的 process started 条目）。
    # 原因是 IfCondition 在 TimerAction 内部求值时机有问题。改成把 RViz
    # 单独包进一个带 condition 的 GroupAction，独立于 TimerAction 求值，
    # 保证 use_rviz:=true 时 RViz 一定启动。
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_group = GroupAction(
        actions=[rviz_node],
        condition=IfCondition(start_rviz),
    )

    # Build the launch description. The perception-related nodes are delayed
    # slightly so the simulation + TF tree + move_group are up before the first
    # pipeline run.
    ld = LaunchDescription()

    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(gazebo_launch)
    ld.add_action(relay_pointcloud)
    ld.add_action(relay_color_image)
    ld.add_action(move_group_launch)

    ld.add_action(TimerAction(
        period=10.0,
        actions=[camera_perception_node, perception_test_node],
    ))

    # RViz 单独启动，不进 TimerAction，确保 use_rviz 求值正确。
    ld.add_action(rviz_group)

    return ld
