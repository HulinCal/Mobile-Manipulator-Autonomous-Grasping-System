from setuptools import setup

package_name = 'kul_tabloo_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/kul_tabloo_launch']),
    ('share/kul_tabloo_launch', ['package.xml']),
    ('share/kul_tabloo_launch/launch', ['launch/KUL_TABLOO_LAUNCH.launch.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
