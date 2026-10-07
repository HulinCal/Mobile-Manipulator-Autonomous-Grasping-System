#!/usr/bin/env python3
#
# Copyright 2020 Wheel Hub Intelligent CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Xinjue Zou
# email: xinjue.zou.whi@gmail.com

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    frequency = LaunchConfiguration('frequency', default='5.5')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar_laser')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    screened_begin = LaunchConfiguration('screened_begin', default='0')
    screened_end = LaunchConfiguration('screened_end', default='360')
    max_distance = LaunchConfiguration('max_distance', default='8.0')
    return LaunchDescription([
    
        DeclareLaunchArgument('serial_port', default_value=serial_port,
            description='Specifying serial port to connected lidar'),
            
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate,
            description='Specifying serial baudrate to connected lidar'),
            
        DeclareLaunchArgument('frame_id', default_value=frame_id,
            description='Specifying frame_id of lidar. Default frame_id is \'laser\''),
            
        DeclareLaunchArgument('inverted', default_value=inverted,
            description='Specifying inverted property of lidar'),
            
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate,
            description='Specifying angle_compensate property of lidar'),
   
        DeclareLaunchArgument('screened_begin', default_value=screened_begin,
            description='Specifying screened_begin property of lidar'),
            
        DeclareLaunchArgument('screened_end', default_value=screened_end,
            description='Specifying screened_end property of lidar'),
            
        DeclareLaunchArgument('max_distance', default_value=max_distance,
            description='Specifying max_distance property of lidar'),

        launch_ros.actions.Node(
            package='rplidar_ros2',
            node_executable='rplidarNode',

            output='screen'),
    ])
