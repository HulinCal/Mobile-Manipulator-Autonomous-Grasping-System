#!/usr/bin/env python3
"""
Bring up the real mycobot280 arm + MoveIt Task Constructor (MTC) pick & place
demo on real hardware.

This launch file composes two existing launch files so that the MTC
pick-and-place pipeline (from mycobot_mtc_pick_place_demo) drives the real
mycobot280 arm through the ros2_control hardware interface (from this
package):

  1. mycobot_280_real_moveit.launch.py (this package)
       - robot_state_publisher
       - controller_manager  (+ arm_controller, gripper_action_controller,
                               joint_state_broadcaster)
       - mycobot_driver_node.py  (pymycobot <-> serial)
       - move_group  (MoveIt2, for PlanningScene monitoring + kinematics)
       - rviz2  (optional)

  2. pick_place_demo.launch.py (mycobot_mtc_pick_place_demo)
       - mtc_node  (plans and executes the MTC task; calls
         /arm_controller/follow_joint_trajectory and
         /gripper_action_controller/gripper_cmd directly, no move_group
         involvement during execution)

The mtc_node is started with a configurable delay (default 8s) so the
controller_manager has time to load and activate all controllers before MTC
starts planning/executing. Without this delay, task_.execute() will fail
because the action servers are not yet advertised.

Notes
-----
* `use_sim_time` is forced to `false` for the real arm. mtc_node is launched
  with the same `use_sim_time:=false` override.
* The perception pipeline (point_cloud_viewer, plane/object segmentation,
  get_planning_scene_server) is NOT started here. If you need real-scene
  perception, launch
  `mycobot_mtc_pick_place_demo/get_planning_scene_server.launch.py`
  separately and feed it a point cloud topic. Without perception, mtc_node
  will fall back to the object pose hardcoded in mtc_node_params.yaml.

Usage
-----
    ros2 launch mycobot280_moveit2_control mycobot_280_real_pick_place.launch.py

    # Custom serial port / delay / no RViz
    ros2 launch mycobot280_moveit2_control mycobot_280_real_pick_place.launch.py \
        driver_port:=/dev/ttyUSB0 mtc_start_delay:=10.0 use_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share_control = get_package_share_directory('mycobot280_moveit2_control')
    pkg_share_mtc_demo = get_package_share_directory('mycobot_mtc_pick_place_demo')

    # --- Launch arguments ---------------------------------------------------
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='mycobot_280',
        description='Robot name (selects MoveIt config subfolder)')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz2 alongside the rest of the stack')
    declare_use_move_group = DeclareLaunchArgument(
        'use_move_group', default_value='true',
        description='Start the MoveIt2 move_group node (provides PlanningScene '
                    'monitor and kinematics for mtc_node)')
    declare_driver_port = DeclareLaunchArgument(
        'driver_port', default_value='/dev/ttyACM0',
        description='Serial port for the mycobot arm')
    declare_driver_baud = DeclareLaunchArgument(
        'driver_baud', default_value='115200',
        description='Baud rate for the mycobot arm')
    declare_mtc_start_delay = DeclareLaunchArgument(
        'mtc_start_delay', default_value='8.0',
        description='Delay (seconds) before starting mtc_node, to give the '
                    'controller_manager time to load and activate controllers')
    declare_mtc_exe = DeclareLaunchArgument(
        'mtc_exe', default_value='mtc_node',
        description='MTC demo executable name')
    declare_use_perception = DeclareLaunchArgument(
        'use_perception', default_value='false',
        description='If true, also launch get_planning_scene_server (needs a '
                    'point cloud on /camera_head/depth/color/points). If '
                    'false, mtc_node skips the GetPlanningScene service call '
                    'and adds a hardcoded collision object at object_pose from '
                    'mtc_node_params.yaml — useful for running on real '
                    'hardware without the perception pipeline.')

    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_move_group = LaunchConfiguration('use_move_group')
    driver_port = LaunchConfiguration('driver_port')
    driver_baud = LaunchConfiguration('driver_baud')
    mtc_start_delay = LaunchConfiguration('mtc_start_delay')
    exe = LaunchConfiguration('mtc_exe')
    use_perception = LaunchConfiguration('use_perception')

    # --- 1) Real-hardware + move_group stack (this package) ----------------
    real_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_control, 'launch',
                         'mycobot_280_real_moveit.launch.py')
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'use_rviz': use_rviz,
            'use_move_group': use_move_group,
            'use_sim_time': 'false',
            'driver_port': driver_port,
            'driver_baud': driver_baud,
        }.items(),
    )

    # --- 2) mtc_node (reproduce the Node configuration of
    #        mycobot_mtc_pick_place_demo/launch/pick_place_demo.launch.py,
    #        but force use_sim_time:=false) ---------------------------------
    def _build_mtc_node(context):
        robot_name_str = robot_name.perform(context)
        pkg_share_moveit_config = FindPackageShare('mycobot_moveit_config').find(
            'mycobot_moveit_config')
        config_path = os.path.join(
            pkg_share_moveit_config, 'config', robot_name_str)
        mtc_node_config_path = os.path.join(pkg_share_mtc_demo, 'config')

        initial_positions_file_path = os.path.join(
            config_path, 'initial_positions.yaml')
        joint_limits_file_path = os.path.join(config_path, 'joint_limits.yaml')
        kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(
            config_path, 'moveit_controllers.yaml')
        srdf_model_path = os.path.join(config_path, f'{robot_name_str}.srdf')
        pilz_cartesian_limits_file_path = os.path.join(
            config_path, 'pilz_cartesian_limits.yaml')
        mtc_node_params_file_path = os.path.join(
            mtc_node_config_path, 'mtc_node_params.yaml')

        moveit_config = (
            MoveItConfigsBuilder(robot_name_str,
                                 package_name='mycobot_moveit_config')
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(
                pipelines=['ompl', 'pilz_industrial_motion_planner', 'stomp'],
                default_planning_pipeline='ompl')
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True)
            .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
            .to_moveit_configs()
        )

        mtc_demo_node = Node(
            package='mycobot_mtc_pick_place_demo',
            executable=exe.perform(context),
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': False},
                {'start_state': {'content': initial_positions_file_path}},
                mtc_node_params_file_path,
                # Whether mtc_node should call the GetPlanningScene service
                # (true) or use the hardcoded object_pose from
                # mtc_node_params.yaml (false). The corresponding
                # launch argument use_perception controls both this flag
                # and whether get_planning_scene_server is launched below.
                {'use_perception': use_perception.perform(context) == 'true'},
            ],
        )
        return [mtc_demo_node]

    # We need an OpaqueFunction because mtc_node parameters depend on the
    # robot_name launch argument (which is only available at runtime via
    # context). The Node itself is then deferred by mtc_start_delay so that
    # the controller_manager has time to come up and activate controllers.
    from launch.actions import OpaqueFunction
    mtc_node_action = OpaqueFunction(function=_build_mtc_node)
    delayed_mtc = TimerAction(period=mtc_start_delay, actions=[mtc_node_action])

    # --- 3) Perception: real OR fake get_planning_scene_server -------------
    # When use_perception:=true, start the real get_planning_scene_server
    # (subscribes to /camera_head/depth/color/points, runs plane + object
    # segmentation). mtc_node will query it for real scene segmentation.
    #
    # When use_perception:=false (the default), start the fake Python server
    # (scripts/fake_planning_scene_server.py) which immediately responds to
    # the get_planning_scene_mycobot service with a single hardcoded
    # collision object built from object_name/object_type/object_dimensions/
    # object_pose parameters — useful for running on real hardware without a
    # camera. The same mtc_node_params.yaml file configures both nodes, so
    # editing object_pose there also updates the fake scene.
    real_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share_mtc_demo, 'launch',
                         'get_planning_scene_server.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
        condition=IfCondition(use_perception),
    )
    # NOTE: mtc_node_params.yaml is structured as
    #     mtc_node:
    #       ros__parameters:
    #         object_pose: [...]
    # ROS 2 matches parameter namespaces by node name. Since this Python
    # node is named 'fake_planning_scene_server' (not 'mtc_node'), the
    # yaml file would be silently ignored, and the fake server would fall
    # back to its built-in defaults (object_pose z=0.0, object_dimensions
    # [0.05, 0.0125]). That pose is unreachable for the arm and MTC's
    # task_.plan() would block forever finding no IK solution.
    # To avoid that, we explicitly pass the scene-relevant parameters
    # from a separate flat yaml file so the fake server always returns a
    # reachable object pose.
    fake_scene_params_file = os.path.join(
        pkg_share_control, 'config', 'fake_planning_scene_params.yaml')
    fake_perception_node = Node(
        package='mycobot280_moveit2_control',
        executable='fake_planning_scene_server.py',
        output='screen',
        parameters=[fake_scene_params_file],
        condition=IfCondition(
            PythonExpression(["'", use_perception, "' == 'false'"]),
        ),
    )

    # --- Compose -----------------------------------------------------------
    ld = LaunchDescription()
    for arg in [
        declare_robot_name, declare_use_rviz, declare_use_move_group,
        declare_driver_port, declare_driver_baud, declare_mtc_start_delay,
        declare_mtc_exe, declare_use_perception,
    ]:
        ld.add_action(arg)

    ld.add_action(real_stack_launch)
    # Start the (real or fake) planning-scene server right away so the
    # service is advertised before mtc_node starts after the delay below.
    ld.add_action(real_perception_launch)
    ld.add_action(fake_perception_node)
    ld.add_action(delayed_mtc)
    return ld
