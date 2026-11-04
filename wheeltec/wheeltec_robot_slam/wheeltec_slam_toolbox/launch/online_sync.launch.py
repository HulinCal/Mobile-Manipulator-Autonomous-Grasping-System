from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


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

    # slam_toolbox sync 生命周期节点
    # 注意：Jazzy 的 slam_toolbox 2.8.5 默认不创建节点侧 bond，
    # nav2 lifecycle_manager 的 bond 永远等不到对端（报 unable to be reached by bond），
    # 因此不使用 lifecycle_manager，改用 launch 事件自动 configure/activate（上游官方做法）
    slam_params_file = os.path.join(
        get_package_share_directory("wheeltec_slam_toolbox"),
        'config',
        'mapper_params_online_sync.yaml'
    )
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time}
        ],
        remappings=[('/odom', '/odom_combined')],
        output='screen'
    )

    # 节点进程启动后自动触发 configure
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # configuring 到达 inactive 后自动触发 activate
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] slam_toolbox is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
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
        slam_node,
        configure_event,
        activate_event,
        rviz_node,
    ])
