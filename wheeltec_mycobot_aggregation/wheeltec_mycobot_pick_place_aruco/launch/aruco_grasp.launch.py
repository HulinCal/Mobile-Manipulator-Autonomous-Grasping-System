#!/usr/bin/env python3
"""ArUco (ID=1) 抓取 launch.

启动内容:
  1. 机械臂 bringup (复用 wheeltec_mycobot_bringup/all.launch.py)
     关闭底盘避障/语音, 小车底盘启动但不会动
  2. 手眼标定 TF 发布 (link1 -> camera_color_optical_frame)
  3. aruco_ros single (marker 检测 + TF 发布)
  4. aruco_grasp_node: marker 稳定后按 预抓取 -> 下探 -> 夹紧 -> 提升
     -> 旋转 -> 放下 的流程抓取 (走 MoveIt MoveGroup + 夹爪 action)

使用方法:
    ros2 launch wheeltec_mycobot_pick_place_aruco aruco_grasp.launch.py
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
    block_size = LaunchConfiguration('block_size')
    calibration_name = LaunchConfiguration('calibration_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_move_group = LaunchConfiguration('use_move_group')
    use_real_hardware = LaunchConfiguration('use_real_hardware')
    tcp_flange_z = LaunchConfiguration('tcp_flange_z')
    gripper_open_value = LaunchConfiguration('gripper_open_value')
    gripper_close_value = LaunchConfiguration('gripper_close_value')

    declare_args = [
        DeclareLaunchArgument(
            'marker_id', default_value='1',
            description='ArUco marker ID to detect'),
        DeclareLaunchArgument(
            'marker_size', default_value='0.026',
            description='ArUco marker side length [m]'),
        DeclareLaunchArgument(
            'block_size', default_value='0.030',
            description='Cube side length [m]'),
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
        # ---- 夹爪安装几何 (与 pick_place_params.yaml 保持一致) ----
        # URDF mycobot_280.urdf.xacro: flange -> gripper_base origin
        # xyz z=0.034m, rpy=(1.579,0,0) (绕 X 约 90°, 法兰 x 轴仍为夹爪
        # 开合方向); 手指沿 gripper_base +y 伸出, 变换后即法兰 +z。
        # tcp_flange_z = 法兰面到两指夹持中心的距离, 自适应夹爪长约
        # 12~13cm, 该值必须覆盖夹爪长度, 否则指尖会捅到方块下方/台面。
        DeclareLaunchArgument(
            'tcp_flange_z', default_value='0.12',
            description='Flange +Z to gripper TCP (finger center) [m]'),
        # 夹爪 gripper_controller 关节指令 [rad]: 0.15 -> 全开,
        # -0.42 -> 张开约 1/3 (不夹到硬限位)。
        DeclareLaunchArgument(
            'gripper_open_value', default_value='0.15',
            description='Gripper open joint value [rad]'),
        DeclareLaunchArgument(
            'gripper_close_value', default_value='-0.42',
            description='Gripper close joint value [rad]'),
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
    # min_marker_size / detection_mode 用灵敏参数: 小 marker 在 Astra 画面里
    # 边长只有几像素, single 默认阈值会直接过滤掉
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

    # ---- 4. aruco_grasp_node (MoveIt 抓取) ----
    aruco_grasp_node = TimerAction(
        period=4.0,
        actions=[Node(
            package='wheeltec_mycobot_pick_place_aruco',
            executable='aruco_grasp_node',
            name='aruco_grasp_node',
            output='screen',
            parameters=[{
                'marker_id': marker_id,
                'marker_size': marker_size,
                'block_size': block_size,
                # 夹爪安装几何 (法兰 -> 两指夹持中心)
                'tcp_flange_z': tcp_flange_z,
                'gripper_open_value': gripper_open_value,
                'gripper_close_value': gripper_close_value,
            }],
        )],
    )

    return LaunchDescription(
        declare_args + [
            bringup_launch,
            handeye_publish,
            aruco_single,
            aruco_grasp_node,
        ]
    )
