#!/usr/bin/env python3
"""
Eye-to-hand hand-eye calibration for the mycobot280 M5 with an Astra Pro camera.

This is a **one-shot** launch file: it brings up everything needed to calibrate
so the user only has to run a single command and start sampling in the rqt GUI.

It starts:
  1. mycobot_280_real_moveit.launch.py  - real arm bringup
       (robot_state_publisher + ros2_control + mycobot_driver + move_group + RViz)
  2. astra_pro.launch.xml              - Astra Pro camera driver
       publishes /camera/color/image_raw, /camera/color/camera_info and the TF
       tree rooted at `camera_link` (camera_color_optical_frame, etc.)
  3. a static TF `base_link -> camera_link`
       Astra's TF chain is disconnected from the arm; this connects it so that
       `base_link -> camera_color_optical_frame` exists in /tf as a *nominal*
       value. The calibration will overwrite this nominal transform with the
       measured one (see camera_calibration.md).
  4. aruco_ros `single`                  - detects ArUco ID 582 and publishes
       TF `camera_color_optical_frame -> aruco_marker_frame`
  4b. a static TF `aruco_marker_frame -> marker_mount_frame`
       Compensates for board thickness and marker-center vs flange-center
       misalignment so that `marker_mount_frame` lies at the flange center
       (= `robot_effector_frame`). See `marker_offset_*` / `marker_thickness`
       in calibration_params.yaml. Set all to 0 for a centered flat marker.
  5. easy_handeye2 `handeye_server` + `rqt_calibrator.py`
       solves `base_link -> camera_color_optical_frame`
       (tracks `marker_mount_frame`, NOT `aruco_marker_frame`)

Calibration type: `eye_on_base` (easy_handeye2's name for eye-to-hand).

:author: hl
:date: August 2026
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    # Nominal pose of the camera in the base_link frame (used as a seed for the
    # static TF that connects the Astra to the arm). Adjust to your physical
    # mounting; the calibration will replace this with the measured value.
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    # Marker mounting offset on the flange (compensates board thickness +
    # marker-center vs flange-center misalignment). See calibration_params.yaml
    # for convention. The launch publishes a static TF
    # `aruco_marker_frame -> marker_mount_frame` and tells easy_handeye2 to
    # track `marker_mount_frame` so it solves for the flange center pose.
    marker_offset_x = LaunchConfiguration('marker_offset_x')
    marker_offset_y = LaunchConfiguration('marker_offset_y')
    marker_thickness = LaunchConfiguration('marker_thickness')
    marker_mount_frame = 'marker_mount_frame'

    declare_args = [
        DeclareLaunchArgument('marker_id', default_value='582',
                              description='ArUco marker ID mounted on the flange.'),
        DeclareLaunchArgument('marker_size', default_value='0.095',
                              description='ArUco marker side length [m] (black square edge).'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link',
                              description='Robot base TF frame.'),
        DeclareLaunchArgument('robot_effector_frame', default_value='link6_flange',
                              description='Robot end-effector (flange) TF frame. '
                                          'Marker is rigidly attached here.'),
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
        DeclareLaunchArgument('camera_x', default_value='0.22',
                              description='Nominal camera x in base_link [m].'),
        DeclareLaunchArgument('camera_y', default_value='-0.30',
                              description='Nominal camera y in base_link [m].'),
        DeclareLaunchArgument('camera_z', default_value='0.50',
                              description='Nominal camera z in base_link [m].'),
        DeclareLaunchArgument('camera_roll', default_value='0.0',
                              description='Nominal camera roll [rad].'),
        DeclareLaunchArgument('camera_pitch', default_value='0.436',
                              description='Nominal camera pitch [rad] (~25 deg).'),
        DeclareLaunchArgument('camera_yaw', default_value='1.5708',
                              description='Nominal camera yaw [rad].'),
        DeclareLaunchArgument('marker_offset_x', default_value='0.0',
                              description='Marker center X offset from flange center [m] '
                                          '(in flange frame; eye-to-hand only).'),
        DeclareLaunchArgument('marker_offset_y', default_value='0.0',
                              description='Marker center Y offset from flange center [m] '
                                          '(in flange frame; eye-to-hand only).'),
        DeclareLaunchArgument('marker_thickness', default_value='0.003',
                              description='Distance from ArUco-detected marker surface to '
                                          'flange end-face [m] (eye-to-hand only).'),
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
    # The Astra Pro launch is an XML launch file; AnyLaunchDescriptionSource
    # auto-detects the format (py / xml / yaml).
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

    # ---- 3) Static TF connecting the Astra to the arm base ----------------
    # The Astra publishes `camera_link -> camera_color_optical_frame` etc., but
    # nothing publishes `base_link -> camera_link`. Without this, the
    # `base_link -> camera_color_optical_frame` chain that easy_handeye2 expects
    # does not exist. We publish a nominal transform (the calibration will
    # later overwrite the measured result via publish.launch.py).
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        output='screen',
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', camera_roll,
            '--pitch', camera_pitch,
            '--yaw', camera_yaw,
            '--frame-id', robot_base_frame,
            '--child-frame-id', 'camera_link',
        ],
    )

    # ---- 4) ArUco single-marker detector ---------------------------------
    # Publishes TF: camera_optical_frame -> marker_frame.
    # Include aruco_ros/single.launch.py so this launch stays in sync with the
    # standalone `ros2 launch aruco_ros single.launch.py` invocation that was
    # verified to receive /camera/color/image_raw and publish /aruco_single/result.
    aruco_single = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('aruco_ros'), '/launch/single.launch.py'
        ]),
        launch_arguments={
            'marker_id': marker_id,
            'marker_size': marker_size,
            'image': image_topic,
            'camera_info': camera_info_topic,
            'camera_frame': camera_optical_frame,
            'marker_frame': marker_frame,
            'reference_frame': '',                 # empty => publish wrt camera_frame
            'corner_refinement': 'LINES',
        }.items(),
    )

    # ---- 4b) Marker -> flange-center compensating static TF --------------
    # aruco_ros publishes `aruco_marker_frame` at the marker center with Z
    # pointing OUT of the marker (toward the camera). Hand-eye calibration
    # needs the tracked frame to coincide with `robot_effector_frame` (the
    # flange center). If the marker is off-center on the flange, or the board
    # has thickness, publish `aruco_marker_frame -> marker_mount_frame` so
    # that `marker_mount_frame` lies at the flange center.
    # The translation is the NEGATED marker pose in the flange frame, because
    # we express the flange center in the marker frame.
    #   aruco_marker_frame.origin + T = flange_center  =>  T = -marker_pose
    static_tf_marker_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_marker_to_mount',
        output='screen',
        arguments=[
            # Negate offsets: flange center in marker frame = -(marker in flange)
            '--x', PythonExpression(['-', marker_offset_x]),
            '--y', PythonExpression(['-', marker_offset_y]),
            '--z', PythonExpression(['-', marker_thickness]),
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', marker_frame,
            '--child-frame-id', marker_mount_frame,
        ],
    )

    # ---- 5) easy_handeye2 hand-eye server + rqt GUI -----------------------
    # eye_on_base == eye-to-hand: camera fixed in world, marker on flange.
    # tracking_base_frame = camera_optical_frame  (ArUco publishes marker wrt it)
    # tracking_marker_frame = marker_mount_frame   (flange center, via the
    #   compensating static TF above). NOT `aruco_marker_frame` directly, so
    #   the solver aligns to the flange center, not the marker center.
    handeye_server = Node(
        package='easy_handeye2',
        executable='handeye_server',
        name='handeye_server',
        output='screen',
        parameters=[{
            'name': name,
            'calibration_type': 'eye_on_base',
            'tracking_base_frame': camera_optical_frame,
            'tracking_marker_frame': marker_mount_frame,
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
            'calibration_type': 'eye_on_base',
            'tracking_base_frame': camera_optical_frame,
            'tracking_marker_frame': marker_mount_frame,
            'robot_base_frame': robot_base_frame,
            'robot_effector_frame': robot_effector_frame,
        }],
    )

    # Defer aruco/handeye briefly so the arm + camera TF is up first; otherwise
    # handeye_server logs "Waiting for TF initialization" for a while.
    calibration_nodes = TimerAction(
        period=4.0,
        actions=[
            aruco_single,
            static_tf_marker_mount,
            handeye_server,
            handeye_rqt_calibrator,
        ],
    )

    return LaunchDescription(declare_args + [
        arm_bringup,
        astra_camera_launch,
        static_tf_camera,
        calibration_nodes,
    ])
