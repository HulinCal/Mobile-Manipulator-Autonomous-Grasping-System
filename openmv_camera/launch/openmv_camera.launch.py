import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/openmv')
    frame_rate = LaunchConfiguration('frame_rate', default='5.0')
    frame_id = LaunchConfiguration('frame_id', default='openmv_camera')
    framesize = LaunchConfiguration('framesize', default='qvga')
    jpeg_quality = LaunchConfiguration('jpeg_quality', default='80')

    node = Node(
        package='openmv_camera',
        executable='openmv_camera_node',
        name='openmv_camera',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'frame_rate': frame_rate,
            'frame_id': frame_id,
            'framesize': framesize,
            'jpeg_quality': jpeg_quality,
        }],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'serial_port', default_value='/dev/openmv',
            description='OpenMV USB CDC serial port (udev symlink recommended).'),
        launch.actions.DeclareLaunchArgument(
            'frame_rate', default_value='5.0',
            description='Publish rate in Hz for /openmv/image_raw.'),
        launch.actions.DeclareLaunchArgument(
            'frame_id', default_value='openmv_camera',
            description='frame_id stamped on the published Image messages.'),
        launch.actions.DeclareLaunchArgument(
            'framesize', default_value='qvga',
            description='OpenMV framesize: qqvga, qcif, qvga, cif or vga.'),
        launch.actions.DeclareLaunchArgument(
            'jpeg_quality', default_value='80',
            description='On-board JPEG compression quality (1-100).'),
        node,
    ])
