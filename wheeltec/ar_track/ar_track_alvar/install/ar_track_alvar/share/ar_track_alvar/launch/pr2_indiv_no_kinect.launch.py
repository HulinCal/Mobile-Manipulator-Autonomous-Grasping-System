from launch import LaunchDescription
import launch_ros.actions
import os 

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ar_track_alvar', 
            executable='individualMarkersNoKinect', 
            output='screen',
            remappings=[
                ("camera_image", "/camera/color/image_raw"),
                ("camera_info","/camera/color/camera_info")
            ],
            parameters=[
                {"marker_size":4.4},
		        {"max_new_marker_error":0.08},
		        {"max_track_error":0.2},
		        {"output_frame":"ar_link"},
		        {"nof_markers":8}
            ],),
        launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='ar_to_camera',
            arguments=['0.3', '0.3', '0.0','0', '0','0','ar_link','camera_link'],)
    ])
