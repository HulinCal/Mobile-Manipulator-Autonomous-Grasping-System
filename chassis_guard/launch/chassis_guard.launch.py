import os

import launch
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = os.path.join(get_package_share_directory('chassis_guard'),
                          'config', 'chassis_guard.yaml')
    start_guard = LaunchConfiguration('start_guard')
    guard = launch_ros.actions.Node(
        package='chassis_guard',
        executable='chassis_guard_node',
        name='chassis_guard_node',
        output='screen',
        parameters=[config],
        condition=IfCondition(start_guard),
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'start_guard', default_value='true',
            description='Start the chassis guard node.'),
        guard,
    ])
