#!/usr/bin/env python3
"""
Eye-in-hand hand-eye calibration for the mycobot280 M5 with an Astra Pro camera.

This is a **one-shot** launch file: it brings up everything needed to calibrate
so the user only has to run a single command and start sampling in the rqt GUI.

It starts:
  1. mycobot_280_real_moveit.launch.py  - real arm bringup
       (robot_state_publisher + ros2_control + mycobot_driver + move_group + RViz)
  2. astra_pro.launch.xml              - Astra Pro camera driver
       publishes /camera/color/image_raw, /camera/color/camera_info and the TF
       tree rooted at `camera_link` (camera_color_optical_frame, etc.)
  3. a static TF `link6_flange -> camera_link`
       Astra's TF chain is disconnected from the arm; this connects it so that
       `link6_flange -> camera_color_optical_frame` exists in /tf as a *nominal*
       value. The calibration will overwrite this nominal transform with the
       measured one (see camera_calibration.md).
  4. aruco_ros `single`                  - detects ArUco ID 582 and publishes
       TF `camera_color_optical_frame -> aruco_marker_frame` (marker fixed in world)
  5. easy_handeye2 `handeye_server` + `rqt_calibrator.py`
       solves `link6_flange -> camera_color_optical_frame`

Calibration type: `eye_in_hand` (easy_handeye2's name for eye-in-hand).

Important:
  Eye-in-hand means the camera is rigidly mounted on the robot's flange and the
  ArUco marker is fixed in the world (does not move with the arm). If you have
  instead mounted the marker on the flange and the camera is fixed, use the
  eye-to-hand launch file.

:author: hl
:date: August 2026
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- Configurable arguments (override on the CLI) ---------------------
    marker_id = LaunchConfiguration('marker_id')
    marker_size = LaunchConfiguration('marker_size')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    robot_effector_frame = LaunchConfiguration('robot_effector_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    name = LaunchConfiguration('name')
    marker_frame = LaunchConfiguration('marker_frame')

    # Astra / arm bringup tunables
    use_rviz = LaunchConfiguration('use_rviz')
    driver_port = LaunchConfiguration('driver_port')
    driver_baud = LaunchConfiguration('driver_baud')
    # Nominal pose of the camera in the flange frame (used as a seed for the
    # static TF that connects the Astra to the arm). Adjust to your physical
    # mounting; the calibration will replace this with the measured value.
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    declare_args = [
        DeclareLaunchArgument('marker_id', default_value='582',
                              description='ArUco marker ID fixed in the world.'),
        DeclareLaunchArgument('marker_size', default_value='0.035',
                              description='ArUco marker side length [m] (black square edge).'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link',
                              description='Robot base TF frame (world fixed frame).'),
        DeclareLaunchArgument('robot_effector_frame', default_value='link6_flange',
                              description='Robot end-effector (flange) TF frame. '
                                          'Camera is rigidly attached here.'),
        DeclareLaunchArgument('camera_optical_frame',
                              default_value='camera_color_optical_frame',
                              description='Optical frame of the Astra Pro color stream '
                                          '(published by astra_pro.launch.xml).'),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw',
                              description='Absolute image topic name.'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info',
                              description='Absolute CameraInfo topic name.'),
        DeclareLaunchArgument('marker_frame', default_value='aruco_marker_frame',
                              description='TF frame name ArUco publishes the marker pose as.'),
        DeclareLaunchArgument('name', default_value='mycobot280_calibration',
                              description='Calibration name (result yaml filename).'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz2 (via the MoveIt launch).'),
        DeclareLaunchArgument('driver_port', default_value='/dev/ttyACM0',
                              description='Serial port for the mycobot arm.'),
        DeclareLaunchArgument('driver_baud', default_value='115200',
                              description='Baud rate for the mycobot arm.'),
        DeclareLaunchArgument('camera_x', default_value='0.0',
                              description='Nominal camera x in flange frame [m].'),
        DeclareLaunchArgument('camera_y', default_value='0.0',
                              description='Nominal camera y in flange frame [m].'),
        DeclareLaunchArgument('camera_z', default_value='0.05',
                              description='Nominal camera z in flange frame [m].'),
        DeclareLaunchArgument('camera_roll', default_value='0.0',
                              description='Nominal camera roll [rad].'),
        DeclareLaunchArgument('camera_pitch', default_value='0.0',
                              description='Nominal camera pitch [rad].'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0',
                              description='Nominal camera yaw [rad].'),
    ]

    # ---- 1) Real arm bringup: ros2_control + driver + move_group + RViz ---
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mycobot280_moveit2_control'),
            '/launch/mycobot_280_real_moveit.launch.py'
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'driver_port': driver_port,
            'driver_baud': driver_baud,
        }.items(),
    )

    # ---- 2) Astra Pro camera driver ---------------------------------------
    # AnyLaunchDescriptionSource auto-detects the launch file format (py / xml / yaml).
    astra_camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('astra_camera'), '/launch/astra_pro.launch.xml'
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'enable_color': 'true',
            'enable_depth': 'true',
            'publish_tf': 'true',
        }.items(),
    )

    # ---- 3) Static TF connecting the Astra to the robot flange -------------
    # Eye-in-hand: the camera is mounted on the flange. We publish a nominal
    # `link6_flange -> camera_link` transform so that the
    # `link6_flange -> camera_color_optical_frame` chain exists in /tf; the
    # calibration will later overwrite this with the measured value.
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_flange_to_camera',
        output='screen',
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', camera_roll,
            '--pitch', camera_pitch,
            '--yaw', camera_yaw,
            '--frame-id', robot_effector_frame,
            '--child-frame-id', 'camera_link',
        ],
    )

    # ---- 4) ArUco single-marker detector ---------------------------------
    # Publishes TF: camera_optical_frame -> marker_frame
    # In eye-in-hand, the marker is fixed in the world. The `reference_frame`
    # is left empty so ArUco publishes the marker pose relative to the camera
    # (which is what easy_handeye2 expects: it samples (robot pose, marker pose
    # in camera) pairs to solve the flange->camera transform).
    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': False,
            'marker_size': marker_size,
            'marker_id': marker_id,
            'reference_frame': '',                 # empty => publish wrt camera_frame
            'camera_frame': camera_optical_frame,
            'marker_frame': marker_frame,
        }],
        remappings=[
            ('/image', image_topic),
            ('/camera_info', camera_info_topic),
        ],
    )

    # ---- 5) easy_handeye2 hand-eye server + rqt GUI -----------------------
    # eye_in_hand: camera on flange, marker fixed in world.
    # tracking_base_frame = camera_optical_frame  (ArUco publishes marker wrt it)
    # tracking_marker_frame = marker_frame         (the ArUco marker frame)
    handeye_server = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_server',
        output='screen',
        parameters=[{
            'name': name,
            'calibration_type': 'eye_in_hand',
            'tracking_base_frame': camera_optical_frame,
            'tracking_marker_frame': marker_frame,
            'robot_base_frame': robot_base_frame,
            'robot_effector_frame': robot_effector_frame,
        }],
    )

    handeye_rqt_calibrator = Node(
        package='easy_handeye2',
        executable='rqt_calibrator.py',
        name='handeye_rqt_calibrator',
        output='screen',
        parameters=[{
            'name': name,
            'calibration_type': 'eye_in_hand',
            'tracking_base_frame': camera_optical_frame,
            'tracking_marker_frame': marker_frame,
            'robot_base_frame': robot_base_frame,
            'robot_effector_frame': robot_effector_frame,
        }],
    )

    # Defer aruco/handeye briefly so the arm + camera TF is up first.
    calibration_nodes = TimerAction(
        period=4.0,
        actions=[aruco_single, handeye_server, handeye_rqt_calibrator],
    )

    return LaunchDescription(declare_args + [
        arm_bringup,
        astra_camera_launch,
        static_tf_camera,
        calibration_nodes,
    ])
