import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments (optional)
        DeclareLaunchArgument('some_param', default_value='default_value', description='A parameter for the node'),

        # Log info (optional)
        LogInfo(condition=None, msg="Launching keyboard_control_emer node"),

        # Node definition
        Node(
            package='keyboard_control_emer',  # Name of the ROS 2 package
            executable='keyboard_control_emer',  # Name of the executable
            name='keyboard_control_emer_node',  # Optional: name of the node
            output='screen',  # Optional: display output in the terminal
            parameters=[{'some_param': 'some_value'}],  # Example: set parameters
        ),
    ])
