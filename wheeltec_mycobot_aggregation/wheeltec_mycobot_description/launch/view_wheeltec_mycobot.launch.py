#!/usr/bin/env python3
"""
Visualize the aggregated WHEELTEC R680 + myCobot 280 robot.

By default only robot_state_publisher, joint_state_publisher_gui and RViz2 are
started (pure visualization). Gazebo is NOT started by default; pass
'use_gazebo:=true' to additionally spawn the (display-only, no controller)
model in modern Gazebo (gz, ros_gz_sim).
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
    package_name = 'wheeltec_mycobot_description'
    xacro_filename = 'wheeltec_mycobot.urdf.xacro'
    rviz_filename = 'wheeltec_mycobot.rviz'

    pkg_share = FindPackageShare(package_name)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share, 'urdf', xacro_filename])
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share, 'rviz', rviz_filename])

    # Launch configurations
    urdf_model = LaunchConfiguration('urdf_model')
    use_gripper = LaunchConfiguration('use_gripper')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    use_jsp = LaunchConfiguration('use_jsp')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    robot_name = LaunchConfiguration('robot_name')
    headless = LaunchConfiguration('headless')
    world_file = LaunchConfiguration('world_file')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')

    # --- Declare arguments -------------------------------------------------
    declare_urdf_model_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to the aggregated robot xacro file')

    declare_use_gripper_cmd = DeclareLaunchArgument(
        name='use_gripper',
        default_value='true',
        choices=['true', 'false'],
        description='Whether the myCobot adaptive gripper is attached')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='false',
        choices=['true', 'false'],
        description='Whether to start Gazebo and spawn the robot (display only)')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to start RViz2')

    declare_use_jsp_cmd = DeclareLaunchArgument(
        name='use_jsp',
        default_value='false',
        choices=['true', 'false'],
        description='Enable headless joint_state_publisher')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Enable joint_state_publisher_gui sliders')

    # Follow use_gazebo by default; can still be overridden explicitly
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value=PythonExpression(
            ["'true' if '", use_gazebo, "' == 'true' else 'false'"]),
        description='Use simulation (Gazebo) clock if true')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RViz config file to use')

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='wheeltec_mycobot',
        description='Name used for the spawned Gazebo entity')

    declare_headless_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='false',
        choices=['true', 'false'],
        description='Run gz server only (no Gazebo GUI)')

    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value='',
        description='Optional Gazebo world file (empty = default empty world)')

    declare_x_cmd = DeclareLaunchArgument(
        name='x', default_value='0.0',
        description='Spawn x position in Gazebo [m]')
    declare_y_cmd = DeclareLaunchArgument(
        name='y', default_value='0.0',
        description='Spawn y position in Gazebo [m]')
    declare_z_cmd = DeclareLaunchArgument(
        name='z', default_value='0.0',
        description='Spawn z position in Gazebo [m]')

    # --- Robot description -------------------------------------------------
    # Note: use_gazebo is intentionally fixed to false for the xacro here:
    # this launch file is for visualization, so the arm gz_ros2_control plugin
    # (which needs controller configuration/activation) is not loaded.
    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'use_gripper:=', use_gripper, ' ',
        'use_gazebo:=false'
    ]), value_type=str)

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }])

    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp))

    joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz))

    # --- Optional Gazebo (modern gz via ros_gz_sim) ------------------------
    # headless:=true adds '-s' so only the gz server runs (no GUI).
    gz_arguments = PythonExpression([
        "'-r -v 4 -s ' if '", headless, "' == 'true' else '-r -v 4 '"])
    # FindPackageShare is evaluated lazily, so launching without Gazebo
    # installed (use_gazebo defaults to false) works without ros_gz_sim.
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments=[('gz_args', [gz_arguments, world_file])],
        condition=IfCondition(use_gazebo))

    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
        condition=IfCondition(use_gazebo))

    ld = LaunchDescription()

    ld.add_action(declare_urdf_model_cmd)
    ld.add_action(declare_use_gripper_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_jsp_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(joint_state_publisher_gui_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld
