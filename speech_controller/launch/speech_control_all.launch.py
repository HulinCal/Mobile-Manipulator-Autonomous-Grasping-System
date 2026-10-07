import os

import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- parameters forwarded to the four launch files ----
    # asr_sherpa
    use_int8 = LaunchConfiguration('use_int8', default='true')
    silence_duration = LaunchConfiguration('silence_duration', default='0.5')
    language = LaunchConfiguration('language', default='auto')
    input_device = LaunchConfiguration('input_device', default='Yundea')
    models_dir = LaunchConfiguration('models_dir', default='')
    # asr_keywords
    threshold = LaunchConfiguration('threshold', default='0.6')
    keywords_file = LaunchConfiguration('keywords_file', default='')
    # speech_controller
    commands_file = LaunchConfiguration('commands_file', default='')
    gripper_action = LaunchConfiguration(
        'gripper_action',
        default='/gripper_action_controller/gripper_cmd')
    # wheeltec robot
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    # 是否由本文件顺带启动底盘栈。独立使用语音栈时保持 true;
    # 当底盘/整车栈已由上层 launch (如 pick_place 的 all.launch.py) 启动时,
    # 必须传 false, 否则会出现第二套 robot_state_publisher (base-only URDF)、
    # 第二个 wheeltec_robot_node/JSP/EKF, 争抢串口并让 RViz 收不到含机械臂的
    # /robot_description, 导致机械臂不显示。
    bringup_robot = LaunchConfiguration('bringup_robot', default='true')

    asr_sherpa_dir = get_package_share_directory('asr_sherpa')
    asr_keywords_dir = get_package_share_directory('asr_keywords')
    speech_controller_dir = get_package_share_directory('speech_controller')
    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')

    # 0) bring up the robot (serial driver, URDF, EKF ...)
    #    仅在 bringup_robot:=true 时启动, 避免与上层整车栈重复。
    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_robot_dir, 'launch',
                         'turn_on_wheeltec_robot.launch.py')),
        launch_arguments={
            'carto_slam': carto_slam,
        }.items(),
        condition=IfCondition(bringup_robot),
    )

    # 1) microphone -> /asr/final_text
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

    # 2) /asr/final_text -> /asr_command
    asr_keywords = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(asr_keywords_dir, 'launch',
                         'asr_keywords.launch.py')),
        launch_arguments={
            'threshold': threshold,
            'keywords_file': keywords_file,
        }.items(),
    )

    # 3) /asr_command -> /cmd_vel
    speech_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(speech_controller_dir, 'launch',
                         'speech_controller.launch.py')),
        launch_arguments={
            'commands_file': commands_file,
            'gripper_action': gripper_action,
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
        launch.actions.DeclareLaunchArgument(
            'commands_file', default_value='',
            description='Path to commands yaml; empty uses the package default config/commands.yaml.'),
        launch.actions.DeclareLaunchArgument(
            'gripper_action',
            default_value='/gripper_action_controller/gripper_cmd',
            description='control_msgs/GripperCommand action server for arm gripper voice commands.'),
        launch.actions.DeclareLaunchArgument(
            'carto_slam', default_value='false',
            description='Forwarded to the robot launch; true to disable the EKF node.'),
        launch.actions.DeclareLaunchArgument(
            'bringup_robot', default_value='true',
            description='Also start the WHEELTEC base bringup. Set false when a '
                        'robot bringup is already running (prevents duplicate '
                        'robot_state_publisher/serial/EKF nodes).'),
        wheeltec_robot,
        asr_sherpa,
        asr_keywords,
        speech_controller,
    ])
