#!/usr/bin/env python3
"""
Eye-to-hand hand-eye calibration for the mycobot280 M5 with an Astra Pro camera.

This is a **one-shot** launch file: it brings up everything needed to calibrate
so the user only has to run a single command and start sampling in the rqt GUI.

It starts:
  1. wheeltec_mycobot_bringup/arm_controller.launch.py
       real arm bringup, using the aggregated wheeltec robot model
       (WHEELTEC R680 chassis + myCobot 280, from wheeltec_mycobot_description):
       robot_state_publisher + ros2_control + mycobot_driver + move_group + RViz.
       TF: base_footprint -> base_link -> link1 -> ... -> link6_flange
       (base_link_to_link1 is the fixed on-car mounting joint).
  2. astra_pro.launch.xml              - Astra Pro camera driver
       publishes /camera/color/image_raw, /camera/color/camera_info and the TF
       tree rooted at `camera_link` (camera_color_optical_frame, etc.)
  3. (no extra camera TF needed)
       The aggregated URDF already publishes `base_link -> camera_link`, and
       the Astra driver publishes `camera_link -> camera_color_optical_frame`,
       so `link1 -> camera_color_optical_frame` is connected through the
       normal TF tree. Publishing another `link1 -> camera_link` here would
       give `camera_link` two parents and split the TF tree.
  4. aruco_ros `single`                  - detects ArUco ID 2 and publishes
       TF `camera_color_optical_frame -> aruco_marker_frame`
  4b. a static TF `aruco_marker_frame -> marker_mount_frame`
       Compensates for board thickness and marker-center vs flange-center
       misalignment so that `marker_mount_frame` lies at the flange center
       (= `robot_effector_frame`). See `marker_offset_*` / `marker_thickness`
       in calibration_params.yaml. Set all to 0 for a centered flat marker.
  5. easy_handeye2 `handeye_server` + `rqt_calibrator.py`
       solves `link1 -> camera_color_optical_frame`
       (tracks `marker_mount_frame`, NOT `aruco_marker_frame`)

All default values are loaded from config/calibration_params.yaml (the
`mycobot280` section); any launch argument can still be overridden on the CLI.

Calibration type: `eye_on_base` (easy_handeye2's name for eye-to-hand).

:author: hl
:date: August 2026
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_calibration_params():
    """读取 config/calibration_params.yaml 的 mycobot280 段作为参数默认值。"""
    share = get_package_share_directory('mycobot_handeye_calibration')
    path = os.path.join(share, 'config', 'calibration_params.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data.get('mycobot280', {})


def join_topic(namespace, topic):
    """把命名空间和相对话题名拼成绝对话题名（topic 已是绝对路径时原样返回）。"""
    if topic.startswith('/'):
        return topic
    ns = namespace.rstrip('/')
    return f'{ns}/{topic.lstrip("/")}' if ns else f'/{topic.lstrip("/")}'


def generate_launch_description():
    # ---- 从 calibration_params.yaml 读取默认值（CLI 参数仍可覆盖）---------
    cfg = load_calibration_params()
    camera_namespace = cfg.get('camera_namespace', '/camera')

    def arg_str(key, fallback):
        return str(cfg.get(key, fallback))

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
        DeclareLaunchArgument('marker_id', default_value=arg_str('marker_id', 2),
                              description='ArUco marker ID mounted on the flange.'),
        DeclareLaunchArgument('marker_size', default_value=arg_str('marker_size', 0.026),
                              description='ArUco marker side length [m] (black square edge).'),
        DeclareLaunchArgument('robot_base_frame', default_value=arg_str('robot_base_frame', 'link1'),
                              description='Robot base TF frame. 标定结果发布为 '
                                          'link1 -> camera_color_optical_frame。'),
        DeclareLaunchArgument('robot_effector_frame', default_value=arg_str('robot_effector_frame', 'link6_flange'),
                              description='Robot end-effector (flange) TF frame. '
                                          'Marker is rigidly attached here.'),
        DeclareLaunchArgument('camera_optical_frame',
                              default_value=arg_str('camera_optical_frame', 'camera_color_optical_frame'),
                              description='Optical frame of the Astra Pro color stream '
                                          '(published by astra_pro.launch.xml).'),
        DeclareLaunchArgument('image_topic',
                              default_value=join_topic(camera_namespace, cfg.get('image_topic', 'color/image_raw')),
                              description='Absolute image topic name.'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value=join_topic(camera_namespace, cfg.get('camera_info_topic', 'color/camera_info')),
                              description='Absolute CameraInfo topic name.'),
        DeclareLaunchArgument('marker_frame', default_value='aruco_marker_frame',
                              description='TF frame name ArUco publishes the marker pose as.'),
        DeclareLaunchArgument('name', default_value=arg_str('calibration_name', 'mycobot280_calibration'),
                              description='Calibration name (result yaml filename).'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Start RViz2 (via the MoveIt launch).'),
        DeclareLaunchArgument('driver_port', default_value=arg_str('driver_port', '/dev/mycobot_controller'),
                              description='Serial port for the mycobot arm.'),
        DeclareLaunchArgument('driver_baud', default_value=arg_str('driver_baud', 115200),
                              description='Baud rate for the mycobot arm.'),
        DeclareLaunchArgument('marker_offset_x', default_value=arg_str('marker_offset_x', 0.0),
                              description='Marker center X offset from flange center [m] '
                                          '(in flange frame; eye-to-hand only).'),
        DeclareLaunchArgument('marker_offset_y', default_value=arg_str('marker_offset_y', 0.0),
                              description='Marker center Y offset from flange center [m] '
                                          '(in flange frame; eye-to-hand only).'),
        DeclareLaunchArgument('marker_thickness', default_value=arg_str('marker_thickness', 0.003),
                              description='Distance from ArUco-detected marker surface to '
                                          'flange end-face [m]; 允许负数 (eye-to-hand only).'),
        DeclareLaunchArgument('use_joint_gui', default_value='true',
                              description='启动关节滑块 GUI（拖动手柄控制机械臂位置）。'),
    ]

    # ---- 1) Real arm bringup: 聚合机器人 (WHEELTEC R680 + myCobot 280) -----
    # 使用 wheeltec_mycobot_description 的聚合 URDF（机械臂装在底盘 base_link 上，
    # base_link_to_link1 固定连接），而不是独立的 mycobot_280 桌面模型。
    # 该 launch 提供 robot_state_publisher + ros2_control + driver + MoveIt。
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('wheeltec_mycobot_bringup'),
            '/launch/arm_controller.launch.py'
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

    # ---- 3) 相机与机械臂的 TF 已由聚合模型提供，无需额外静态 TF ----------
    # 聚合 URDF (wheeltec_mycobot_description) 中已包含
    #   base_footprint -> base_link -> camera_link
    # 而 Astra 驱动发布 camera_link -> camera_color_frame -> camera_color_optical_frame，
    # 因此 link1 -> camera_color_optical_frame 的链路天然连通。
    # 注意：不能再发布 `link1 -> camera_link` 静态 TF，否则 camera_link 会有
    # 两个父节点（base_link 和 link1），TF 树会断裂。

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

    # ---- 关节滑块 GUI（拖动手柄控制机械臂位置）----------------------------
    # 与 MoveIt RViz 的 6D InteractiveMarker 互补：拖滑块直接发
    # /arm_controller/joint_trajectory，不依赖规划，松手即动。
    use_joint_gui = LaunchConfiguration('use_joint_gui')
    joint_slider_gui = Node(
        package='mycobot_handeye_calibration',
        executable='joint_slider_gui.py',
        name='joint_slider_gui',
        output='screen',
        condition=IfCondition(use_joint_gui),
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
        joint_slider_gui,
        calibration_nodes,
    ])
