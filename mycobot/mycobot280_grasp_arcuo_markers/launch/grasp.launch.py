#!/usr/bin/env python3
"""
启动文件：ArUco marker 检测 + MoveIt 目标位姿计算

启动内容：
1. Astra Pro 相机
2. 手眼标定结果发布 (base_link → camera_color_optical_frame)
3. ArUco marker_publisher (检测 ID 1/2/3)
4. grasp_node (计算并打印 MoveIt 目标位姿)

使用方法：
    source /home/hl/mycobot/easy_handeye2_ws/install/setup.bash
    source /home/hl/mycobot/mycobot_ros2-main/install/setup.bash
    ros2 launch mycobot280_grasp_arcuo_markers grasp.launch.py

前提条件：
- 已完成手眼标定 (easy_handeye2)，结果保存在
  ~/.ros/easy_handeye/mycobot280_calibration_eye_to_hand.yaml
- astra_camera, aruco_ros, mycobot_handeye_calibration 已 source

:author hl
:date September 2026
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 参数 ----
    marker_size = LaunchConfiguration('marker_size')
    calibration_name = LaunchConfiguration('calibration_name')

    declare_args = [
        DeclareLaunchArgument(
            'marker_size', default_value='0.026',
            description='ArUco marker 边长 [m]'),
        DeclareLaunchArgument(
            'calibration_name', default_value='mycobot280_calibration',
            description='手眼标定名称 (须与标准时一致)'),
    ]

    # ---- 1. Astra Pro 相机 ----
    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('astra_camera'),
                'launch', 'astra_pro.launch.xml'
            ])
        ]),
    )

    # ---- 2. 手眼标定结果发布 (base_link → camera_link) ----
    # 不使用 easy_handeye2 的 handeye_publisher，因为它会发布 base_link → camera_color_optical_frame，
    # 这与 astra_camera 发布的 camera_color_frame → camera_color_optical_frame 冲突
    # （camera_color_optical_frame 有两个父节点，TF 树会断开成两棵）。
    # 这里使用本包的 camera_link_tf_publisher 节点，把标定结果从 optical_frame 转换到 camera_link，
    # 让 astra_camera 的内部 TF 树挂接到 base_link 上。
    handeye_publish = Node(
        package='mycobot280_grasp_arcuo_markers',
        executable='camera_link_tf_publisher',
        name='camera_link_tf_publisher',
        output='screen',
        parameters=[{
            'calibration_name': calibration_name,
        }],
    )

    # ---- 3. ArUco marker_publisher ----
    # 检测所有可见的 ArUco marker 并发布 MarkerArray
    # 节点名设为 aruco_marker_publisher，输出 topic 为
    #   /aruco_marker_publisher/markers (aruco_msgs/MarkerArray)
    marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        output='screen',
        parameters=[{
            'marker_size': marker_size,
            'reference_frame': '',                  # 默认 = camera_frame
            'camera_frame': 'camera_color_optical_frame',
            'image_is_rectified': False,            # Astra 发布原始图像
            'use_camera_info': True,
        }],
        remappings=[
            ('/image', '/camera/color/image_raw'),
            ('/camera_info', '/camera/color/camera_info'),
        ],
    )

    # ---- 4. grasp_node (计算 MoveIt 目标位姿) ----
    config_file = PathJoinSubstitution([
        FindPackageShare('mycobot280_grasp_arcuo_markers'),
        'config', 'grasp_params.yaml'
    ])

    grasp_node = Node(
        package='mycobot280_grasp_arcuo_markers',
        executable='detect_markers_node',
        name='detect_markers_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription(
        declare_args + [
            astra_launch,
            handeye_publish,
            marker_publisher,
            grasp_node,
        ]
    )
