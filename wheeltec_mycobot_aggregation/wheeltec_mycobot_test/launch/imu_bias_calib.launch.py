"""IMU 零偏标定启动文件 (组合后整车 WHEELTEC R680 + myCobot 280)

默认行为:
  1. include wheeltec_mycobot_bringup/all.launch.py —— 组合整车完整栈
     (聚合 URDF/RSP + 底盘串口 + 雷达/相机 + EKF + 机械臂 + RViz)
     默认关闭 chassis_guard / 语音 / move_group, 保持车体静止
  2. 启动 imu_bias_calib 节点: 静止采集指定时长, 计算三轴陀螺/加速度零偏,
     打印结果与应用位置后自动退出

用法示例:
  # 默认: 静止采集 20s
  ros2 launch wheeltec_mycobot_test imu_bias_calib.launch.py

  # 采集 30s, 提高置信度
  ros2 launch wheeltec_mycobot_test imu_bias_calib.launch.py duration:=30.0

  # 整车栈已手动启动, 只跑标定节点
  ros2 launch wheeltec_mycobot_test imu_bias_calib.launch.py bringup:=false

  # 车体非水平安装(如机械臂导致倾斜), 不按 |g| 扣除重力
  ros2 launch wheeltec_mycobot_test imu_bias_calib.launch.py assume_flat_mount:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- 是否拉起整车栈 ----
    arg_bringup = DeclareLaunchArgument(
        'bringup', default_value='true',
        description='是否 include wheeltec_mycobot_bringup/all.launch.py 拉起整车栈')

    # ---- 整车栈开关 (透传给 all.launch.py) ----
    arg_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='是否启动 RViz2')
    arg_use_move_group = DeclareLaunchArgument(
        'use_move_group', default_value='false',
        description='是否启动 MoveIt2 move_group (标定不需要)')
    arg_use_chassis_guard = DeclareLaunchArgument(
        'use_chassis_guard', default_value='false',
        description='是否启动 chassis_guard 避障守卫 (标定期间无需)')
    arg_use_speech = DeclareLaunchArgument(
        'use_speech', default_value='false', description='是否启动语音控制链路')
    arg_use_real_hardware = DeclareLaunchArgument(
        'use_real_hardware', default_value='true',
        description='是否连接真实硬件(串口/相机/雷达)')

    # ---- 标定节点参数 ----
    arg_duration = DeclareLaunchArgument(
        'duration', default_value='20.0', description='静止采样时长 (s)')
    arg_assume_flat = DeclareLaunchArgument(
        'assume_flat_mount', default_value='true',
        description='车体是否水平安装(决定加速度零偏是否扣重力)')
    arg_auto_exit = DeclareLaunchArgument(
        'auto_exit', default_value='true',
        description='打印结果后是否自动退出节点')

    # ---- 组合整车完整栈 ----
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wheeltec_mycobot_bringup'),
                'launch', 'all.launch.py',
            ])
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_move_group': LaunchConfiguration('use_move_group'),
            'use_chassis_guard': LaunchConfiguration('use_chassis_guard'),
            'use_speech': LaunchConfiguration('use_speech'),
            'use_real_hardware': LaunchConfiguration('use_real_hardware'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('bringup')),
    )

    # ---- IMU 零偏标定节点 ----
    imu_bias_calib_node = Node(
        package='wheeltec_mycobot_test',
        executable='imu_bias_calib',
        name='imu_bias_calib',
        output='screen',
        parameters=[{
            'duration': LaunchConfiguration('duration'),
            'assume_flat_mount': LaunchConfiguration('assume_flat_mount'),
            'auto_exit': LaunchConfiguration('auto_exit'),
        }],
    )

    return LaunchDescription([
        arg_bringup,
        arg_use_rviz,
        arg_use_move_group,
        arg_use_chassis_guard,
        arg_use_speech,
        arg_use_real_hardware,
        arg_duration,
        arg_assume_flat,
        arg_auto_exit,
        bringup,
        imu_bias_calib_node,
    ])
