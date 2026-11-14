from setuptools import setup
import os
from glob import glob

package_name = 'wheeltec_mycobot_pick_place_aruco'

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
    description='ArUco marker block pick-and-place test',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_grasp_node = wheeltec_mycobot_pick_place_aruco.aruco_grasp_node:main',
            'move_to_pick_ready = wheeltec_mycobot_pick_place_aruco.move_to_pick_ready:main',
            'aruco_align_node = wheeltec_mycobot_pick_place_aruco.aruco_align_node:main',
        ],
    },
)
