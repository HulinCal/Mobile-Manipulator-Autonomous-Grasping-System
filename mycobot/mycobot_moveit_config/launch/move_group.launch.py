#!/usr/bin/env python3
"""
Launch MoveIt 2 for the myCobot robotic arm.

This script creates a ROS 2 launch file that starts the necessary nodes and services
for controlling a myCobot robotic arm using MoveIt 2. It loads configuration files,
starts the move_group node, and optionally launches RViz for visualization.

:author: Addison Sears-Collins
:date: December 13, 2024
"""

import os
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Generate a launch description for MoveIt 2 with myCobot robot.

    This function sets up the necessary configuration and nodes to launch MoveIt 2
    for controlling a myCobot robotic arm. It includes setting up paths to config files,
    declaring launch arguments, configuring the move_group node, and optionally starting RViz.

    Returns:
        LaunchDescription: A complete launch description for the MoveIt 2 system
    """
    # Constants for paths to different files and folders
    package_name_moveit_config = 'mycobot_moveit_config'

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_package = LaunchConfiguration('rviz_config_package')

    # Get the package share directory
    pkg_share_moveit_config_temp = FindPackageShare(package=package_name_moveit_config)

    # Declare the launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='mycobot_280',
        description='Name of the robot to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='move_group.rviz',
        description='RViz configuration file')

    declare_rviz_config_package_cmd = DeclareLaunchArgument(
        name='rviz_config_package',
        default_value=package_name_moveit_config,
        description='Package containing the RViz configuration file')

    # 可选: 用外部 URDF 覆盖 moveit 配置包内的模型。
    # 例如 wheeltec_mycobot_bringup 使用聚合机器人
    # (WHEELTEC 底盘 + myCobot 280) 时，需要让 MoveIt / RViz 也使用同一个模型，
    # 否则规划场景里显示的还是旧的独立机械臂模型。留空则保持原行为。
    declare_robot_description_cmd = DeclareLaunchArgument(
        name='robot_description',
        default_value='',
        description='可选: 外部机器人 URDF 字符串，非空时覆盖配置包内的 '
                    'robot_description（用于 wheeltec 聚合机器人等模型）')

    def configure_setup(context):
        """Configure MoveIt and create nodes with proper string conversions."""
        # Get the robot name as a string for use in MoveItConfigsBuilder
        robot_name_str = LaunchConfiguration('robot_name').perform(context)

        # Get package path
        pkg_share_moveit_config = pkg_share_moveit_config_temp.find(package_name_moveit_config)

        # Construct file paths using robot name string
        config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)

        # Define all config file paths
        initial_positions_file_path = os.path.join(config_path, 'initial_positions.yaml')
        joint_limits_file_path = os.path.join(config_path, 'joint_limits.yaml')
        kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
        srdf_model_path = os.path.join(config_path, f'{robot_name_str}.srdf')
        pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

        # Create MoveIt configuration
        moveit_config = (
            MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
            .to_moveit_configs()
        )

        # 可选的外部 URDF 覆盖（例如 wheeltec 聚合机器人模型）
        external_urdf = LaunchConfiguration('robot_description').perform(context)
        moveit_config_dict = moveit_config.to_dict()
        if external_urdf.strip():
            moveit_config_dict['robot_description'] = external_urdf
            # MoveIt 要求 SRDF 的 <robot name="..."> 与 URDF 的 robot name 一致，
            # 否则会报 "Semantic description is not specified for the same robot
            # as the URDF"。外部模型名不同时把 SRDF 的 name 改成一致。
            urdf_match = re.search(r'<robot[^>]*\bname="([^"]+)"', external_urdf)
            urdf_name = urdf_match.group(1) if urdf_match else None
            srdf_xml = moveit_config_dict['robot_description_semantic']
            if urdf_name:
                srdf_xml = re.sub(r'(<robot[^>]*\bname=")[^"]*(")',
                                  lambda m: m.group(1) + urdf_name + m.group(2),
                                  srdf_xml, count=1)
                moveit_config_dict['robot_description_semantic'] = srdf_xml

        # MoveIt capabilities
        # 注意：显式设置 `capabilities` 会覆盖 MoveGroup 默认列表。MoveIt 2 加载
        # capability 时若遇到不存在的类名会抛异常，导致后续 capability 全部不
        # 加载 → action server 全空 → MoveGroupInterface.plan() 失败。
        # 这里只显式启用实际需要的 capability：
        #   - MoveGroupMoveAction: 提供 /move_group action (plan + execute)
        #     是 MoveGroupInterface.plan() 必需的
        #   - ExecuteTaskSolutionCapability: 执行 MTC 生成的任务解
        move_group_capabilities = {
            "capabilities": "move_group/MoveGroupMoveAction "
                            "move_group/ExecuteTaskSolutionCapability"
        }

        # Create move_group node
        # 注意：MoveIt 2 Jazzy 把 MoveGroup action 默认命名为 /move_action
        # （Humble 之前叫 /move_group）。MoveGroupInterface 客户端在 Jazzy
        # 默认连 /move_action，所以不需要重映射。直接保持默认即可。
        start_move_group_node_cmd = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config_dict,
                {'use_sim_time': use_sim_time},
                {'start_state': {'content': initial_positions_file_path}},
                move_group_capabilities,
            ],
        )

        # Create RViz node
        rviz_robot_description = dict(moveit_config.robot_description)
        rviz_robot_description.update(moveit_config.robot_description_semantic)
        if external_urdf.strip():
            rviz_robot_description['robot_description'] = external_urdf
            rviz_robot_description['robot_description_semantic'] = \
                moveit_config_dict['robot_description_semantic']
        start_rviz_node_cmd = Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                [FindPackageShare(rviz_config_package), "/rviz/", rviz_config_file]
            ],
            output="screen",
            parameters=[
                rviz_robot_description,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {'use_sim_time': use_sim_time}
            ],
        )

        # RViz exit handler
        exit_event_handler = RegisterEventHandler(
            condition=IfCondition(use_rviz),
            event_handler=OnProcessExit(
                target_action=start_rviz_node_cmd,
                on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
            ),
        )

        return [start_move_group_node_cmd, start_rviz_node_cmd, exit_event_handler]

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_config_package_cmd)
    ld.add_action(declare_robot_description_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add the setup and node creation
    ld.add_action(OpaqueFunction(function=configure_setup))

    return ld
