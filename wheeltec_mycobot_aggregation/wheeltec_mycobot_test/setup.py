import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wheeltec_mycobot_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admi',
    maintainer_email='admi@todo.todo',
    description='Test nodes for wheeltec robot + mycobot',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'imu_error_test = wheeltec_mycobot_test.imu_error_test_node:main',
            'odom_spin_test = wheeltec_mycobot_test.odom_spin_test_node:main',
            'imu_bias_calib = wheeltec_mycobot_test.imu_bias_calib_node:main',
        ],
    },
)
