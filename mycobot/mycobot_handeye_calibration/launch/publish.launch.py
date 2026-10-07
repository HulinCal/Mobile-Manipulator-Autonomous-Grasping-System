#!/usr/bin/env python3
"""
Publish the hand-eye calibration result as a TF transform.

Wraps easy_handeye2's `publish.launch.py`. After calibration, the result
yaml (saved by easy_handeye2 under `~/.ros/easy_handeye/`) is loaded and the
resulting transform is published to `/tf`.

For eye-to-hand, publishes `base_link -> camera_optical_frame`.
For eye-in-hand, publishes `link6_flange -> camera_optical_frame`.

:author: hl
:date: August 2026
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    name = LaunchConfiguration('name')

    declare_args = [
        DeclareLaunchArgument('name', default_value='mycobot280_calibration',
                              description='Calibration name (must match the one '
                                          'used during calibrate.launch).'),
    ]

    publish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('easy_handeye2'), '/launch/publish.launch.py'
        ]),
        launch_arguments={
            'name': name,
        }.items(),
    )

    return LaunchDescription(declare_args + [publish_launch])
