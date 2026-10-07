#!/usr/bin/env python3
"""
Bring up the real mycobot280 M5 arm with ros2_control + MoveIt2.

This launch file wires together everything needed to plan motions with MoveIt2
and execute them on the real arm through the ros2_control hardware interface
provided by the mycobot280_moveit2_control package:

  1. robot_state_publisher   - publishes /robot_description (URDF with use_gazebo:=false)
  2. controller_manager      - ros2_control node that loads our hardware plugin and
                               hosts the arm_controller + joint_state_broadcaster
  3. mycobot_driver_node.py  - the Python driver (installed as a ros2 executable
                               from this package's src/ directory). It subscribes
                               to /mycobot/cmd_joint_pos (6 arm joint commands,
                               radians) and /mycobot/cmd_gripper_pos (gripper 0..100
                               open ratio), and publishes the corresponding states
                               on /mycobot/current_joint_pos and
                               /mycobot/current_gripper_pos.
  4. load_controllers        - spawns joint_state_broadcaster, arm_controller,
                              and gripper_action_controller (in sequence)
  5. move_group              - MoveIt2 planning & execution node (optional, reuse the
                               existing mycobot_moveit_config launch file)
  6. rviz2                   - visualization (optional)

The data flow is:

    MoveIt2  --> /arm_controller/follow_joint_trajectory (action)
       |
       v
    arm_controller (JointTrajectoryController)
       |
       v
    MyCobot280HardwareInterface (this package, C++)
       |  publishes /mycobot/cmd_joint_pos   (Float64MultiArray, radians)
       |  subscribes /mycobot/current_joint_pos (Float64MultiArray, radians)
       v
    mycobot_driver_node.py (pymycobot -> serial -> arm)

Usage:
    ros2 launch mycobot280_moveit2_control mycobot_280_real_moveit.launch.py
    ros2 launch mycobot280_moveit2_control mycobot_280_real_moveit.launch.py \
        use_rviz:=false driver_port:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Resolve package shares
    pkg_share_control = get_package_share_directory('mycobot280_moveit2_control')
    pkg_share_moveit = get_package_share_directory('mycobot_moveit_config')

    # --- Launch arguments ---------------------------------------------------
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='mycobot_280',
        description='Robot name (selects MoveIt config subfolder)')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz2 alongside the rest of the stack')
    declare_use_move_group = DeclareLaunchArgument(
        'use_move_group', default_value='true',
        description='Start the MoveIt2 move_group node')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (false for real hardware)')
    declare_prefix = DeclareLaunchArgument(
        'prefix', default_value='', description='Joint prefix')
    declare_flange_link = DeclareLaunchArgument(
        'flange_link', default_value='link6_flange',
        description='Flange link name')
    declare_use_gripper = DeclareLaunchArgument(
        'use_gripper', default_value='true',
        description='Attach a gripper (visual only - the Python driver '
                    'does not control the gripper)')
    declare_driver_port = DeclareLaunchArgument(
        'driver_port', default_value='/dev/mycobot_controller',
        description='Serial port for the mycobot arm')
    declare_driver_baud = DeclareLaunchArgument(
        'driver_baud', default_value='115200',
        description='Baud rate for the mycobot arm')

    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_move_group = LaunchConfiguration('use_move_group')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    flange_link = LaunchConfiguration('flange_link')
    use_gripper = LaunchConfiguration('use_gripper')
    driver_port = LaunchConfiguration('driver_port')
    driver_baud = LaunchConfiguration('driver_baud')

    # --- Build the URDF via xacro (use_gazebo:=false selects our plugin) ----
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('mycobot_description'), 'urdf', 'robots', 'mycobot_280.urdf.xacro'])
    robot_description_content = ParameterValue(
        Command([
            'xacro', ' ', urdf_xacro, ' ',
            'robot_name:=', robot_name, ' ',
            'prefix:=', prefix, ' ',
            'flange_link:=', flange_link, ' ',
            'use_gazebo:=false', ' ',
            'use_gripper:=', use_gripper, ' ',
            # Override the URDF <param> default with the launch argument.
            # We pass the same value to the hardware plugin via the URDF.
        ]),
        value_type=str,
    )

    # Controller config lives inside this package.
    controllers_yaml = os.path.join(
        pkg_share_control, 'config', 'mycobot_280_real_controllers.yaml')

    # --- Nodes --------------------------------------------------------------

    # 1) robot_state_publisher - publishes /robot_description and the tfs.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }],
    )

    # 2) controller_manager (ros2_control_node) - hosts the hardware plugin
    #    and the controllers. It reads the URDF from the robot_description
    #    parameter and instantiates our MyCobot280HardwareInterface plugin.
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content},
            controllers_yaml,
        ],
        output='screen',
        # Make sure the hardware plugin can find its own ROS node handles.
        remappings=[
            ('/mycobot/cmd_joint_pos', '/mycobot/cmd_joint_pos'),
            ('/mycobot/current_joint_pos', '/mycobot/current_joint_pos'),
        ],
    )

    # 3) Python driver. Lives in this package (src/mycobot_driver_update_node.py)
    #    and is installed as a ros2 executable via CMakeLists.txt PROGRAMS.
    #    使用防抖动流式驱动节点 (set_fresh_mode(1) + send_angles 非阻塞
    #    + EMA 平滑 + 动态节拍), 替代旧版 sync_send_angles 阻塞驱动。
    #    Parameters port and baudrate are read through declare_parameter
    #    defaults inside mycobot_driver_update_node.py.
    declare_driver_speed = DeclareLaunchArgument(
        'driver_speed', default_value='80',
        description='Firmware interpolation speed 1-100 (anti-jitter driver)')
    declare_driver_smooth = DeclareLaunchArgument(
        'driver_smooth', default_value='3',
        description='EMA smoothing window, larger = softer (anti-jitter driver)')
    declare_driver_stream_hz = DeclareLaunchArgument(
        'driver_stream_hz', default_value='40.0',
        description='Streaming send frequency Hz, <=50 (anti-jitter driver)')
    declare_driver_gripper_speed = DeclareLaunchArgument(
        'driver_gripper_speed', default_value='30',
        description='Gripper open/close speed 1-100 (anti-jitter driver)')
    driver_speed = LaunchConfiguration('driver_speed')
    driver_smooth = LaunchConfiguration('driver_smooth')
    driver_stream_hz = LaunchConfiguration('driver_stream_hz')
    driver_gripper_speed = LaunchConfiguration('driver_gripper_speed')

    # driver_node = Node(
    #     package='mycobot280_moveit2_control',
    #     executable='mycobot_driver_update_node.py',
    #     name='mycobot_driver_update',
    #     output='screen',
    #     parameters=[{
    #         'port': driver_port,
    #         'baudrate': driver_baud,
    #         'speed': driver_speed,
    #         'smooth': driver_smooth,
    #         'stream_hz': driver_stream_hz,
    #         'gripper_speed': driver_gripper_speed,
    #     }],
    # )
    
    driver_node = Node(
        package='mycobot280_moveit2_control',
        executable='mycobot_driver_node.py',
        name='mycobot_driver',
        output='screen',
        parameters=[{
            'port': driver_port,
            'baudrate': driver_baud,
        }],
    )


    # 4) Spawn the controllers sequentially after the controller_manager is up.
    spawn_jsb_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')
    spawn_arm_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen')
    spawn_gripper_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_action_controller'],
        output='screen')
    # Chain: joint_state_broadcaster -> arm_controller -> gripper_action_controller.
    load_arm_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_jsb_cmd,
            on_start=[TimerAction(period=2.0, actions=[spawn_arm_cmd])]))
    load_gripper_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_arm_cmd,
            on_start=[TimerAction(period=1.0, actions=[spawn_gripper_cmd])]))
    # Defer spawning controllers by 3s to give the plugin time to initialize.
    spawn_controllers_timer = TimerAction(
        period=3.0, actions=[spawn_jsb_cmd, load_arm_handler, load_gripper_handler])

    # 5) MoveIt2 move_group (re-use the existing launch file). Let it spawn
    #    RViz itself when use_rviz:=true so that RViz receives all the MoveIt
    #    parameters (robot_description, robot_description_semantic,
    #    planning_pipelines, kinematics, joint_limits) needed to display the
    #    MotionPlanning panel. Starting RViz separately from this package would
    #    miss those parameters and the panel would be empty / broken.
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_moveit, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'rviz_config_package': 'mycobot_moveit_config',
            'rviz_config_file': 'move_group.rviz',
        }.items(),
        condition=IfCondition(use_move_group),
    )

    # --- Compose ------------------------------------------------------------
    ld = LaunchDescription()
    for arg in [
        declare_robot_name, declare_use_rviz, declare_use_move_group,
        declare_use_sim_time, declare_prefix, declare_flange_link,
        declare_use_gripper, declare_driver_port, declare_driver_baud,
        declare_driver_speed, declare_driver_smooth,
        declare_driver_stream_hz, declare_driver_gripper_speed,
    ]:
        ld.add_action(arg)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(driver_node)
    ld.add_action(spawn_controllers_timer)
    ld.add_action(move_group_launch)
    return ld
