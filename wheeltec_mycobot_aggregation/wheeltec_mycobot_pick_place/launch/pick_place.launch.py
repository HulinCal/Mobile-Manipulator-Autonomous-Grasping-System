#!/usr/bin/env python3
"""Main launch file for the WHEELTEC + myCobot apple pick-and-place.

Modes
-----
* ``sim_mode:=false`` (default): also includes the aggregated robot
  bringup (all.launch.py) and the speech stack
  (speech_control_all.launch.py). Uses the real car and arm.
* ``sim_mode:=true``: launches only the pick_place nodes in
  simulation mode. No hardware / sensors required.

Usage
-----
    # Real hardware (default; vertical top-down grasp node)
    ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py

    # Simulation
    ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py \
        sim_mode:=true

    # Use the angle-sweep grasp node (planner picks the best reachable
    # approach angle; horizontal along car-x preferred, vertical fallback)
    ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py \
        grasp_impl:=angle
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = 'wheeltec_mycobot_pick_place'
    pkg_share = get_package_share_directory(pkg)

    sim_mode = LaunchConfiguration('sim_mode')
    use_rviz = LaunchConfiguration('use_rviz')
    use_speech = LaunchConfiguration('use_speech')
    grasp_impl = LaunchConfiguration('grasp_impl')

    declare_sim = DeclareLaunchArgument(
        'sim_mode', default_value='false',
        description='Run in simulation mode (no hardware); false = real car and arm')
    declare_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Start RViz2 (real mode only)')
    declare_speech = DeclareLaunchArgument(
        'use_speech', default_value='true',
        description='Start the speech recognition stack (real mode only)')
    declare_grasp_impl = DeclareLaunchArgument(
        'grasp_impl', default_value='vertical',
        description="Grasp node to use: 'vertical' = grasp_node (tested "
                    "top-down grasp, default); 'angle' = grasp_angle_node "
                    "(planner picks the best reachable approach angle, "
                    "horizontal-along-car-x preferred, vertical as fallback)")

    params_file = os.path.join(pkg_share, 'config', 'pick_place_params.yaml')

    def node_params():
        return [params_file, {'sim_mode': sim_mode}]

    # ---- pick_place nodes (always launched) ---------------------------
    voice_node = Node(
        package=pkg, executable='voice_command_node',
        name='voice_command_node', output='screen',
        parameters=node_params())
    detection_node = Node(
        package=pkg, executable='apple_detection_node',
        name='apple_detection_node', output='screen',
        parameters=node_params())
    navigation_node = Node(
        package=pkg, executable='navigation_node',
        name='navigation_node', output='screen',
        parameters=node_params())
    # 抓取节点二选一 (grasp_impl): 服务/话题接口完全一致, coordinator 无感。
    grasp_node = Node(
        package=pkg, executable='grasp_node',
        name='grasp_node', output='screen',
        parameters=node_params(),
        condition=IfCondition(PythonExpression([
            "'", grasp_impl, "' == 'vertical'"])))
    grasp_angle_node = Node(
        package=pkg, executable='grasp_angle_node',
        name='grasp_angle_node', output='screen',
        parameters=node_params(),
        condition=IfCondition(PythonExpression([
            "'", grasp_impl, "' == 'angle'"])))
    coordinator_node = Node(
        package=pkg, executable='coordinator_node',
        name='coordinator_node', output='screen',
        parameters=node_params())

    # ---- Real hardware bringup ----------------------------------------
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('wheeltec_mycobot_bringup'),
            '/launch/all.launch.py',
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_move_group': 'true',
        }.items(),
        condition=UnlessCondition(sim_mode),
    )

    # ---- Speech stack (real mode + use_speech) ------------------------
    # bringup_robot:=false: 整车栈已由上面的 all.launch.py 启动, 禁止语音栈
    # 再启动一套 turn_on_wheeltec_robot (否则重复 RSP 发布 base-only URDF,
    # RViz 的 RobotModel 拿不到含机械臂的 /robot_description -> 机械臂不显示)。
    speech_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('speech_controller'),
            '/launch/speech_control_all.launch.py',
        ]),
        launch_arguments={
            'bringup_robot': 'false',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", sim_mode, "' == 'false' and '", use_speech, "' == 'true'"])),
    )

    info_sim = LogInfo(
        msg='[pick_place] SIM mode. Trigger: '
            'ros2 service call /pick_place/trigger_manual std_srvs/srv/Trigger',
        condition=IfCondition(sim_mode))
    info_real = LogInfo(
        msg='[pick_place] REAL mode. Say "把苹果拿过来" or call '
            '/pick_place/trigger_manual.',
        condition=UnlessCondition(sim_mode))

    ld = LaunchDescription()
    ld.add_action(declare_sim)
    ld.add_action(declare_rviz)
    ld.add_action(declare_speech)
    ld.add_action(declare_grasp_impl)
    ld.add_action(info_sim)
    ld.add_action(info_real)

    ld.add_action(voice_node)
    ld.add_action(detection_node)
    ld.add_action(navigation_node)
    ld.add_action(grasp_node)
    ld.add_action(grasp_angle_node)
    ld.add_action(coordinator_node)
    ld.add_action(bringup_launch)
    ld.add_action(speech_launch)

    return ld
