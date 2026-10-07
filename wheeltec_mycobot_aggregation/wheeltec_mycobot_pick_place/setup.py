from setuptools import setup
import os
from glob import glob

package_name = 'wheeltec_mycobot_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Voice-triggered apple pick-and-place for WHEELTEC + myCobot',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_command_node = wheeltec_mycobot_pick_place.voice_command_node:main',
            'apple_detection_node = wheeltec_mycobot_pick_place.apple_detection_node:main',
            'navigation_node = wheeltec_mycobot_pick_place.navigation_node:main',
            'grasp_node = wheeltec_mycobot_pick_place.grasp_node:main',
            'grasp_angle_node = wheeltec_mycobot_pick_place.grasp_angle_node:main',
            'coordinator_node = wheeltec_mycobot_pick_place.coordinator_node:main',
        ],
    },
)
