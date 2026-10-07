import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    commands_file = LaunchConfiguration('commands_file', default='')
    command_topic = LaunchConfiguration('command_topic', default='/asr_command')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')
    gripper_action = LaunchConfiguration(
        'gripper_action',
        default='/gripper_action_controller/gripper_cmd')

    node = Node(
        package='speech_controller',
        executable='speech_controller_node',
        name='speech_controller',
        output='screen',
        parameters=[{
            'commands_file': commands_file,
            'command_topic': command_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'gripper_action': gripper_action,
        }],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'commands_file', default_value='',
            description='Path to commands yaml; empty uses the package default config/commands.yaml.'),
        launch.actions.DeclareLaunchArgument(
            'command_topic', default_value='/asr_command',
            description='Input topic with text commands.'),
        launch.actions.DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='Output topic for Twist velocity commands.'),
        launch.actions.DeclareLaunchArgument(
            'gripper_action',
            default_value='/gripper_action_controller/gripper_cmd',
            description='control_msgs/GripperCommand action server name.'),
        node,
    ])
