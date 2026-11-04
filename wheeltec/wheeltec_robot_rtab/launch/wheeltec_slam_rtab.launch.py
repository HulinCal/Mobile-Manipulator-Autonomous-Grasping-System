import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    # 本包自带 rviz 配置: Map 订阅 /map, 并含 RobotModel / LaserScan / TF 显示
    default_rviz_config = os.path.join(
        get_package_share_directory('wheeltec_robot_rtab'), 'rviz', 'rtabmap.rviz')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch','turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
    )
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_camera.launch.py')),
    )

    # 注意: parameters 必须是 list[dict], 直接传 dict 会触发 launch_ros TypeError
    # frame_id 必须是底盘基座帧, 用 camera_link 会让 rtabmap 发布 map -> camera_link,
    # 与 base_footprint -> camera_link 的静态变换形成双父节点, 打断 TF 树
    parameters = [{
        'sync_queue_size': 20,
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_scan': True,
        'subscribe_depth': True,
        'wait_for_transform': 1.0,
        # --- 防重影: 里程计(旋转打滑)漂移用 ICP 扫描匹配在入图前校正 ---
        'Reg/Strategy': '1',                       # 1=ICP 扫描配准(默认0=视觉特征,室内弱纹理不稳)
        'Reg/Force3DoF': 'true',
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',    # 限制单次配准幅度, 防错误匹配
        'Icp/Iterations': '30',
        'Icp/Epsilon': '0.001',
        'Icp/MaxTranslation': '0.3',
        # --- 栅格: 改用 2D 雷达生成(深度点云噪声大是重影来源之一) ---
        'Grid/Sensor': '1',                        # 0.23 参数: 1=仅激光扫描建栅格(0=相机,2=混合)
        'Grid/3D': 'false',                        # 2D 占据栅格
        'Grid/RayTracing': 'true',                 # 光线追踪清除幻影格
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '4.0',                    # 只用 4m 内雷达点, 远端误差放大
        # --- 回访近邻配准 + 2D 优化 ---
        'RGBD/NeighborLinkRefining': 'true',       # 关键: 每个新节点与上一节点 ICP 精配, 逐帧吸收轮速打滑漂移
        'RGBD/ProximityByTime': 'true',            # 时间窗内节点互配, 不回访也能纠正本会话漂移
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/LinearUpdate': '0.05',               # 每 5cm/3° 加一个节点, 减小节点间漂移
        'RGBD/AngularUpdate': '0.05',
        'Rtabmap/DetectionRate': '2.0',            # 闭环检测 2Hz, 更早纠正漂移
        'Optimizer/Iterations': '20',
        'Optimizer/GravitySigma': '0'}]

    remappings = [
        ('odom', '/odom_combined'),
        ('scan', '/scan'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch rviz2'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config,
            description='Full path to the rviz2 config file'),

        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        wheeltec_robot, wheeltec_lidar, wheeltec_camera,

        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            condition=IfCondition(launch_rviz),
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config]),
    ])
