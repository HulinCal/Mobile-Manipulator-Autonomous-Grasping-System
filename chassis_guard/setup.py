from setuptools import find_packages, setup

package_name = 'chassis_guard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/chassis_guard.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/chassis_guard.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admi',
    maintainer_email='admi@todo.todo',
    description=('Reactive obstacle-avoidance guard overlay for the '
                 'chassis cmd_vel during voice/keyboard manual driving.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_guard_node = chassis_guard.chassis_guard_node:main',
        ],
    },
)
