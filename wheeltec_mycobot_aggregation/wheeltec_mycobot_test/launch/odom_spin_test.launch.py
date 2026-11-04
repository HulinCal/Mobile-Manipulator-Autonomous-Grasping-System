"""里程计原地旋转漂移测试启动文件 (组合后整车 WHEELTEC R680 + myCobot 280)

默认行为:
  1. include wheeltec_mycobot_bringup/all.launch.py —— 组合整车完整栈
     (聚合 URDF/RSP + 底盘串口 + 雷达/相机 + EKF + 机械臂 + RViz)
     为做纯 odom 测试, 默认关闭 chassis_guard / 语音 / move_group,
     让测试节点直接发 /cmd_vel 驱动底盘
  2. 启动 odom_spin_test 节点: 原地旋转 N 圈, 对比 /odom 与 /odom_combined
     统计轨迹是否定点、是否画圈漂移

用法示例:
  # 默认: 原地 2 圈, 角速度 0.4 rad/s
  ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py

  # 原地 1 圈, 慢速
  ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py \
      revolutions:=1.0 angular_speed:=0.3

  # 整车栈已手动启动, 只跑测试节点
  ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py bringup:=false

  # 只采样, 不打速度指令 (自己用键盘控制)
  ros2 launch wheeltec_mycobot_test odom_spin_test.launch.py publish_cmd_vel:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


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
        description='是否启动 MoveIt2 move_group (odom 测试不需要)')
    arg_use_chassis_guard = DeclareLaunchArgument(
        'use_chassis_guard', default_value='false',
        description='是否启动 chassis_guard 避障守卫 (纯 odom 测试需关闭)')
    arg_use_speech = DeclareLaunchArgument(
        'use_speech', default_value='false', description='是否启动语音控制链路')
    arg_use_real_hardware = DeclareLaunchArgument(
        'use_real_hardware', default_value='true',
        description='是否连接真实硬件(串口/相机/雷达)')

    # ---- 测试节点参数 ----
    arg_angular_speed = DeclareLaunchArgument(
        'angular_speed', default_value='0.4', description='原地旋转角速度 (rad/s)')
    arg_revolutions = DeclareLaunchArgument(
        'revolutions', default_value='2.0', description='原地旋转圈数')
    arg_duration = DeclareLaunchArgument(
        'duration', default_value='45.0', description='超时兜底秒数')
    arg_publish_cmd_vel = DeclareLaunchArgument(
        'publish_cmd_vel', default_value='true',
        description='是否由测试节点自己发 /cmd_vel')
    arg_control_source = DeclareLaunchArgument(
        'control_source', default_value='fused',
        description='用哪路里程计判断停止: fused(/odom_combined) / raw(/odom)')
    arg_wait_before = DeclareLaunchArgument(
        'wait_seconds_before_start', default_value='5.0',
        description='底盘与 EKF 稳定后等待多少秒再开始旋转')

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

    # ---- 里程计原地旋转漂移测试节点 ----
    odom_spin_test_node = Node(
        package='wheeltec_mycobot_test',
        executable='odom_spin_test',
        name='odom_spin_test',
        output='screen',
        parameters=[{
            'angular_speed': LaunchConfiguration('angular_speed'),
            'revolutions': LaunchConfiguration('revolutions'),
            'duration': LaunchConfiguration('duration'),
            'publish_cmd_vel': LaunchConfiguration('publish_cmd_vel'),
            'control_source': LaunchConfiguration('control_source'),
            'wait_seconds_before_start': LaunchConfiguration('wait_seconds_before_start'),
        }],
    )

    return LaunchDescription([
        arg_bringup,
        arg_use_rviz,
        arg_use_move_group,
        arg_use_chassis_guard,
        arg_use_speech,
        arg_use_real_hardware,
        arg_angular_speed,
        arg_revolutions,
        arg_duration,
        arg_publish_cmd_vel,
        arg_control_source,
        arg_wait_before,
        bringup,
        odom_spin_test_node,
    ])
