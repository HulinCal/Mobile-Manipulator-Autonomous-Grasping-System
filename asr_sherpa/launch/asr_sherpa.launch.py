import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_int8 = LaunchConfiguration('use_int8', default='true')
    silence_duration = LaunchConfiguration('silence_duration', default='0.5')
    language = LaunchConfiguration('language', default='auto')
    input_device = LaunchConfiguration('input_device', default='Yundea')
    models_dir = LaunchConfiguration('models_dir', default='')

    asr_node = Node(
        package='asr_sherpa',
        executable='asr_node',
        name='asr_sherpa',
        output='screen',
        parameters=[{
            'use_int8': use_int8,
            'silence_duration': silence_duration,
            'language': language,
            'input_device': input_device,
            'models_dir': models_dir,
        }],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_int8',
            default_value='true',
            description='Use the int8 model instead of float32.'),
        launch.actions.DeclareLaunchArgument(
            'silence_duration',
            default_value='0.5',
            description='Pause duration in seconds that separates speech segments.'),
        launch.actions.DeclareLaunchArgument(
            'language',
            default_value='auto',
            description='Language for SenseVoice (auto, zh, en, ja, ko, yue).'),
        launch.actions.DeclareLaunchArgument(
            'input_device',
            default_value='Yundea',
            description='Microphone: device index (e.g. 6) or name substring '
                        '(e.g. "Yundea", "hw:2,0", "USB Audio"); -1 for default.'),
        launch.actions.DeclareLaunchArgument(
            'models_dir',
            default_value='',
            description='Directory containing the downloaded models.'),
        asr_node,
    ])
