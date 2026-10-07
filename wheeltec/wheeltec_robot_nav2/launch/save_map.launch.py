#ros2 run nav2_map_server map_saver_cli -f ~/map
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions


def generate_launch_description():

    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', '/home/admi/wheeltec_ws/src/wheeltec/wheeltec_robot_nav2/map/WHEELTEC'],
        
        parameters=[{'save_map_timeout': 2000.0},
                    ]

        )
    ld = LaunchDescription()

    ld.add_action(map_saver)

    return ld
 
