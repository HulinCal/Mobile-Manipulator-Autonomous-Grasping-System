import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    enable_wait_for_fin = LaunchConfiguration('enable_wait_for_fin', default='true')

    configured_params = [os.path.join(get_package_share_directory("wheeltec_robot_rrt"), 'config', 'exploration_params.yaml'),
                            {'use_sim_time': use_sim_time}]
    wheeltec_rrt_dir = get_package_share_directory('wheeltec_robot_rrt')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_wait_for_fin', default_value='true',
            description='是否启动建图结束判定+回原点节点 wait_for_fin'),

        Node(
            package='wheeltec_robot_rrt',
            executable='local_rrt',
            name='local_rrt',
            output='both',
            parameters=configured_params),

        Node(
            package='wheeltec_robot_rrt',
            executable='global_rrt',
            name='global_rrt',
            output='both',
            parameters=configured_params),

        Node(
            package='wheeltec_robot_rrt',
            executable='filter',
            name='filter',
            output='both',
            parameters=configured_params),
            
        Node(
            package='wheeltec_robot_rrt',
            executable='assigner',
            name='assigner',
            output='both',
            parameters=configured_params),

        Node(
            package='wheeltec_robot_rrt',
            executable='wait_for_fin',
            name='wait_for_fin',
            output='both',
            condition=IfCondition(enable_wait_for_fin),
            parameters=configured_params),

    ])
