"""IMU 误差测试启动文件

默认行为:
  1. include turn_on_wheeltec_robot.launch.py  —— 拉起底盘驱动 + IMU + EKF + TF
  2. 启动 imu_error_test 节点, 自动发 /cmd_vel 让小车做
       rotate  (原地旋转 90°) → forward  (前进 1m) → backward (后退 1m)

  停止条件 (三者取先到者):
    - 目标距离/角度达成
    - duration 超时 (默认 30s, 作兜底)

用法示例:
  # 默认: 原地 90° + 前进 1m + 后退 1m
  ros2 launch wheeltec_mycobot_test test_imu.launch.py

  # 原地转 180°, 前进 2m
  ros2 launch wheeltec_mycobot_test test_imu.launch.py \
      target_rotation_deg:=180 target_distance:=2.0

  # 只测前进 1.5m
  ros2 launch wheeltec_mycobot_test test_imu.launch.py test_mode:=forward target_distance:=1.5

  # 节点只采样, 不发 cmd_vel (用 wheeltec_keyboard 手动控制)
  ros2 launch wheeltec_mycobot_test test_imu.launch.py publish_cmd_vel:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_robot_launch = os.path.join(bringup_dir, 'launch', 'turn_on_wheeltec_robot.launch.py')

    # ---- LaunchArguments ----
    arg_akmcar = DeclareLaunchArgument('akmcar', default_value='false',
                                       description='是否使用阿克曼小车')
    arg_test_mode = DeclareLaunchArgument('test_mode', default_value='all',
                                          description='测试模式: rotate / forward / backward / all')
    arg_duration = DeclareLaunchArgument('duration', default_value='30.0',
                                         description='超时兜底秒数 (运动目标优先到达)')
    arg_target_distance = DeclareLaunchArgument('target_distance', default_value='1.0',
                                                description='前进/后退目标距离 (m)')
    arg_target_rotation = DeclareLaunchArgument('target_rotation_deg', default_value='90.0',
                                                description='原地旋转目标角度 (°)')
    arg_angular_speed = DeclareLaunchArgument('angular_speed', default_value='0.5',
                                              description='原地旋转角速度 (rad/s)')
    arg_linear_speed = DeclareLaunchArgument('linear_speed', default_value='0.15',
                                             description='前进/后退线速度 (m/s)')
    arg_publish_cmd_vel = DeclareLaunchArgument('publish_cmd_vel', default_value='true',
                                                description='是否由测试节点自己发 cmd_vel')
    arg_sample_rate = DeclareLaunchArgument('sample_rate', default_value='50.0',
                                            description='采样频率 Hz')
    arg_wait_before = DeclareLaunchArgument('wait_seconds_before_start', default_value='3.0',
                                            description='底盘稳定后等待多少秒再开始测试')

    # ---- 拉起底盘 + IMU + EKF ----
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wheeltec_robot_launch),
        launch_arguments={
            'akmcar': LaunchConfiguration('akmcar'),
            'carto_slam': 'false',
        }.items(),
    )

    # ---- IMU 误差测试节点 ----
    imu_error_test_node = Node(
        package='wheeltec_mycobot_test',
        executable='imu_error_test',
        name='imu_error_test',
        output='screen',
        parameters=[{
            'test_mode': LaunchConfiguration('test_mode'),
            'duration': LaunchConfiguration('duration'),
            'target_distance': LaunchConfiguration('target_distance'),
            'target_rotation_deg': LaunchConfiguration('target_rotation_deg'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'linear_speed': LaunchConfiguration('linear_speed'),
            'publish_cmd_vel': LaunchConfiguration('publish_cmd_vel'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'wait_seconds_before_start': LaunchConfiguration('wait_seconds_before_start'),
        }],
    )

    return LaunchDescription([
        arg_akmcar,
        arg_test_mode,
        arg_duration,
        arg_target_distance,
        arg_target_rotation,
        arg_angular_speed,
        arg_linear_speed,
        arg_publish_cmd_vel,
        arg_sample_rate,
        arg_wait_before,
        wheeltec_robot,
        imu_error_test_node,
    ])
