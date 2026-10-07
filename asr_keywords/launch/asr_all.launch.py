import os

import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- configurable parameters (forwarded to both launch files) ----
    use_int8 = LaunchConfiguration('use_int8', default='true')
    silence_duration = LaunchConfiguration('silence_duration', default='0.5')
    language = LaunchConfiguration('language', default='auto')
    input_device = LaunchConfiguration('input_device', default='Yundea')
    models_dir = LaunchConfiguration('models_dir', default='')
    threshold = LaunchConfiguration('threshold', default='0.6')
    keywords_file = LaunchConfiguration('keywords_file', default='')

    asr_sherpa_dir = get_package_share_directory('asr_sherpa')
    asr_keywords_dir = get_package_share_directory('asr_keywords')

    # ---- asr_sherpa: microphone -> /asr/final_text ----
    asr_sherpa = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(asr_sherpa_dir, 'launch',
                         'asr_sherpa.launch.py')),
        launch_arguments={
            'use_int8': use_int8,
            'silence_duration': silence_duration,
            'language': language,
            'input_device': input_device,
            'models_dir': models_dir,
        }.items(),
    )

    # ---- asr_keywords: /asr/final_text -> /asr_command ----
    asr_keywords = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(asr_keywords_dir, 'launch',
                         'asr_keywords.launch.py')),
        launch_arguments={
            'threshold': threshold,
            'keywords_file': keywords_file,
        }.items(),
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_int8', default_value='true',
            description='Use the int8 ASR model instead of float32.'),
        launch.actions.DeclareLaunchArgument(
            'silence_duration', default_value='0.5',
            description='Pause duration in seconds that separates speech segments.'),
        launch.actions.DeclareLaunchArgument(
            'language', default_value='auto',
            description='Language for SenseVoice (auto, zh, en, ja, ko, yue).'),
        launch.actions.DeclareLaunchArgument(
            'input_device', default_value='Yundea',
            description='Microphone index or name substring.'),
        launch.actions.DeclareLaunchArgument(
            'models_dir', default_value='',
            description='Directory containing the downloaded models.'),
        launch.actions.DeclareLaunchArgument(
            'threshold', default_value='0.6',
            description='Minimum similarity to accept a command.'),
        launch.actions.DeclareLaunchArgument(
            'keywords_file', default_value='',
            description='Path to keywords yaml; empty uses the package default.'),
        asr_sherpa,
        asr_keywords,
    ])
