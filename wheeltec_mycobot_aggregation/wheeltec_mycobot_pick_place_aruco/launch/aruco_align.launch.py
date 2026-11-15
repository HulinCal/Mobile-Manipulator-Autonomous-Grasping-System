#!/usr/bin/env python3
"""ArUco (ID=1) 对准 launch.

启动内容:
  1. 机械臂 bringup (复用 wheeltec_mycobot_bringup/all.launch.py)
     关闭底盘避障/语音, 小车底盘启动但不会动
  2. 手眼标定 TF 发布 (link1 -> camera_color_optical_frame)
  3. aruco_ros single (id=1 marker 检测 + TF 发布)
  4. aruco_align_node: marker 转到 link1 系, 用 J1~J5 让法兰 Z 轴对准 marker,
     直接走 FollowJointTrajectory, 不用 MoveIt 规划器

使用方法:
    ros2 launch wheeltec_mycobot_pick_place_aruco aruco_align.launch.py
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---- 参数 ----
    marker_id = LaunchConfiguration('marker_id')
    marker_size = LaunchConfiguration('marker_size')
    calibration_name = LaunchConfiguration('calibration_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_move_group = LaunchConfiguration('use_move_group')
    use_real_hardware = LaunchConfiguration('use_real_hardware')
    marker_offset_x = LaunchConfiguration('marker_offset_x')
    marker_offset_y = LaunchConfiguration('marker_offset_y')
    marker_offset_z = LaunchConfiguration('marker_offset_z')
    tcp_flange_z = LaunchConfiguration('tcp_flange_z')
    insert_extra_depth = LaunchConfiguration('insert_extra_depth')

    declare_args = [
        DeclareLaunchArgument(
            'marker_id', default_value='1',
            description='ArUco marker ID to detect'),
        DeclareLaunchArgument(
            'marker_size', default_value='0.026',
            description='ArUco marker side length [m]'),
        DeclareLaunchArgument(
            'calibration_name', default_value='mycobot280_calibration',
            description='easy_handeye2 calibration result name'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2'),
        DeclareLaunchArgument(
            'use_move_group', default_value='true',
            description='Launch MoveIt2 move_group'),
        DeclareLaunchArgument(
            'use_real_hardware', default_value='true',
            description='Connect real hardware'),
        # 相机外参系统偏差补偿 [m], link1 系; 修正后位置 = 相机解算值 + 偏移
        DeclareLaunchArgument(
            'marker_offset_x', default_value='0.0',
            description='Marker position correction in link1 X [m]'),
        DeclareLaunchArgument(
            'marker_offset_y', default_value='0.0',
            description='Marker position correction in link1 Y [m]'),
        DeclareLaunchArgument(
            'marker_offset_z', default_value='0.0',
            description='Marker position correction in link1 Z [m]'),
        # 夹爪几何: 法兰 +Z 到实际夹持点 [m].
        # 指尖全长约 0.10m, 方块夹在两指根部, 实机实测夹持点距法兰约 0.05m.
        # 前伸量 = P_z - tcp_flange_z + insert_extra_depth
        DeclareLaunchArgument(
            'tcp_flange_z', default_value='0.05',
            description='Flange +Z to gripper grasp center [m]'),
        # ArUco 板厚 0.03m, 位姿给出板中心, 抓取面在板前表面,
        # 额外深入半个板厚 0.015m
        DeclareLaunchArgument(
            'insert_extra_depth', default_value='0.015',
            description='Extra forward depth past the marker [m]'),
    ]

    # ---- 1. 机械臂 bringup ----
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheeltec_mycobot_bringup'),
                'launch', 'all.launch.py'
            ])
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_move_group': use_move_group,
            'use_real_hardware': use_real_hardware,
            'use_chassis_guard': 'false',
            'use_speech': 'false',
        }.items(),
    )

    # ---- 2. 手眼标定 TF 发布 (link1 -> camera_color_optical_frame) ----
    handeye_publish = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('easy_handeye2'),
                '/launch/publish.launch.py'
            ]),
            launch_arguments={'name': calibration_name}.items(),
        )],
    )

    # ---- 3. aruco_ros single ----
    # 使用 aruco_pose_test 的灵敏参数覆盖 single 默认值:
    #   min_marker_size    0.02 → 0.01 (图像面积 1%→2%, 约 78px→55px 边)
    #   detection_mode     DM_FAST → DM_VIDEO_FAST (视频流连续检测更稳定)
    # 实测 marker 在 Astra 640x480 画面中边长仅 ~7px,
    # single 默认 2% 面积阈值会直接过滤掉, 必须调小
    aruco_single = TimerAction(
        period=3.0,
        actions=[Node(
            package='aruco_ros',
            executable='single',
            name='aruco_single',
            output='screen',
            parameters=[{
                'image_is_rectified': False,
                'marker_size': marker_size,
                'marker_id': marker_id,
                'camera_frame': 'camera_color_optical_frame',
                'marker_frame': 'aruco_marker_frame',
                'reference_frame': '',
                'corner_refinement': 'LINES',
                'min_marker_size': 0.01,
                'detection_mode': 'DM_VIDEO_FAST',
            }],
            remappings=[
                ('/camera_info', '/camera/color/camera_info'),
                ('/image', '/camera/color/image_raw'),
            ],
        )],
    )

    # ---- 4. aruco_align_node (J1~J5 对准, 无规划器) ----
    # 检测 id=1 marker -> link1 系位置 -> 事先一次解算 J1~J5 目标角并单条轨迹下发
    aruco_align_node = TimerAction(
        period=4.0,
        actions=[Node(
            package='wheeltec_mycobot_pick_place_aruco',
            executable='aruco_align_node',
            name='aruco_align_node',
            output='screen',
            parameters=[{
                'marker_offset_x': marker_offset_x,
                'marker_offset_y': marker_offset_y,
                'marker_offset_z': marker_offset_z,
                'tcp_flange_z': tcp_flange_z,
                'insert_extra_depth': insert_extra_depth,
            }],
        )],
    )

    return LaunchDescription(
        declare_args + [
            bringup_launch,
            handeye_publish,
            aruco_single,
            aruco_align_node,
        ]
    )
