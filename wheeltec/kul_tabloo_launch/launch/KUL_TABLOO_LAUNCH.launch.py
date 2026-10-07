from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manual_drive',
            namespace='manual_drive',
            executable='manual_drive',
            name='manual_drive'
        ),
        Node(
            package='quiz_pi_con',
            namespace='quiz_pi_con',
            executable='quiz_pi_con',
            name='quiz_pi_con'
        ),
        Node(
            package='quiz_status',
            executable='quiz_status',
            name='quiz_status',
        )
    ])