import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    threshold = LaunchConfiguration('threshold', default='0.6')
    keywords_file = LaunchConfiguration('keywords_file', default='')

    keyword_node = Node(
        package='asr_keywords',
        executable='keyword_node',
        name='asr_keywords',
        output='screen',
        parameters=[{
            'threshold': threshold,
            'keywords_file': keywords_file,
        }],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'threshold',
            default_value='0.6',
            description='Minimum similarity (0.0~1.0) to accept a command.'),
        launch.actions.DeclareLaunchArgument(
            'keywords_file',
            default_value='',
            description='Path to keywords yaml; empty uses the package default config/keywords.yaml.'),
        keyword_node,
    ])
