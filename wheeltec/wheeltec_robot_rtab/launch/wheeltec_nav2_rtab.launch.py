import os
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    params_file = LaunchConfiguration('params_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    nav2_dir = get_package_share_directory('wheeltec_nav2')

    # 当前整车模型为 senior_4wd_bs, 对应 nav2 参数文件 param_four_wheel_diff_bs.yaml
    default_params_file = os.path.join(
        nav2_dir, 'param', 'wheeltec_param', 'param_four_wheel_diff_bs.yaml')
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
    navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params_file': params_file}.items(),
    )

    # parameters 必须是 list[dict]; 原来直接传 dict 会让 launch_ros 抛 TypeError
    parameters = [{
        'frame_id': 'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_image': qos,
        'qos_imu': qos,
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
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Full path to the nav2 parameters file to use'),

        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch rviz2'),

        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config,
            description='Full path to the rviz2 config file'),

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        wheeltec_robot, wheeltec_lidar, wheeltec_camera,

        # SLAM 模式:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),  # 删除旧数据库 (~/.ros/rtabmap.db)

        # 定位模式: rtabmap 自身发布 map -> odom, 不再叠加 AMCL, 避免双发布者冲突
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            parameters=parameters + [{'Mem/IncrementalMemory': 'False',
                                      'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),

        navigation,

        Node(
            condition=IfCondition(launch_rviz),
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config]),
    ])
