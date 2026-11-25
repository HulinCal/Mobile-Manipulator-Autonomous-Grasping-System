from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数 use_sim_time，支持命令行传参 use_sim_time:=true/false
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='是否同时启动 rviz2'
    )
    launch_rviz = LaunchConfiguration('launch_rviz')

    default_rviz_config = os.path.join(
        get_package_share_directory('wheeltec_slam_toolbox'),
        'rviz',
        'wheeltec_slam_toolbox.rviz'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='rviz2 配置文件路径'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )

    # 直接复用 slam_toolbox 上游官方 online_async_launch.py（异步建图节点 async_slam_toolbox_node）
    # 上游已使用 launch 事件机制自动 configure/activate 生命周期节点
    # （use_lifecycle_manager 默认 false，不依赖 nav2 lifecycle_manager 的 bond）
    slam_params_file = os.path.join(
        get_package_share_directory('wheeltec_slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items()
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_rviz_arg,
        rviz_config_arg,
        wheeltec_robot,
        wheeltec_lidar,
        slam_launch,
        rviz_node,
    ])
