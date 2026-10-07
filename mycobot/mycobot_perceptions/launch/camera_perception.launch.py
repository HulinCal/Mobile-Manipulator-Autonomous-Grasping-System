#!/usr/bin/env python3
"""
Launch the camera_perception node.

Starts the streaming perception node with parameters loaded from
config/camera_perception.yaml. The node subscribes to the RGB-D camera topics
and publishes the perceived scene as topics.

Launch arguments:
    use_sim_time (bool, default: true): Use the simulation clock.

:author: hl
:date: August 2026
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'mycobot_perceptions'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    config_file = os.path.join(pkg_share, 'config', 'camera_perception.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    start_camera_perception_cmd = Node(
        package=package_name,
        executable='camera_perception',
        name='camera_perception',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # Apply perceived CollisionObjects to move_group's planning scene.
            # Comment these out if you do NOT want move_group to consume the
            # perceived scene (e.g. when running perception standalone).
            ('~/planning_scene', '/planning_scene'),
            ('~/collision_object', '/collision_object'),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_camera_perception_cmd)
    return ld
