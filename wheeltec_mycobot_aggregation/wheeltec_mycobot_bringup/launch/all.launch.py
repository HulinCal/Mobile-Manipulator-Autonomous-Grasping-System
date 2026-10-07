#!/usr/bin/env python3
"""
启动聚合机器人(WHEELTEC R680 + myCobot 280)的完整栈 (all-in-one)。

为避免重复 RSP, 该 launch 文件内联所有组件, 不通过子 launch 引入:
  1. robot_state_publisher   - 发布聚合机器人 /robot_description (use_gazebo:=false)
  2. base_serial             - WHEELTEC 底盘串口驱动 (akmcar:=false)
  3. astra_camera            - Astra Pro 深度相机驱动 (astra_pro.launch.xml)
  4. lslidar_driver          - LSLidar N10 驱动 (lsn10_launch.py)
  5. base_to_gyro            - 静态 TF: base_link -> gyro_link (单位变换)
  6. base_to_laser           - 静态 TF: base_footprint -> laser
                               (xyz='0.0911 0 0.155' rpy='0 0 0')
  6.1 world_to_odom          - 静态 TF: world -> odom_combined (单位变换);
                               world 必须挂在 EKF 里程计之上, 不能在 URDF 里
                               直连 base_footprint (否则双父节点拆树)
  7. robot_ekf               - EKF 融合节点 (turn_on_wheeltec_robot/config/ekf.yaml)
                               动态发布 odom_combined -> base_footprint
  8. controller_manager       - ros2_control 节点, 加载机械臂硬件接口与控制器
                               (mycobot280_moveit2_control/config/
                                mycobot_280_real_controllers.yaml)
  9. mycobot_driver_node.py  - Python 串口驱动 (默认 /dev/ttyACM0, 115200)
  10. load_controllers       - 依次 spawn joint_state_broadcaster -> arm_controller
                               -> gripper_action_controller (TimerAction + 事件串联)
  11. joint_state_publisher  - 无头模式, 发布轮子等非 ros2_control 关节状态
                               (机械臂关节由 joint_state_broadcaster 发布)
  12. move_group             - MoveIt2 规划节点 (可选, use_rviz:=false, 因为本包
                               自己启动 RViz)
  13. rviz2                  - 使用 wheeltec_mycobot_all.rviz

用法:
    ros2 launch wheeltec_mycobot_bringup all.launch.py
    ros2 launch wheeltec_mycobot_bringup all.launch.py \
        use_rviz:=false use_move_group:=false driver_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'wheeltec_mycobot_bringup'
    description_package = 'wheeltec_mycobot_description'
    xacro_filename = 'wheeltec_mycobot.urdf.xacro'
    rviz_filename = 'wheeltec_mycobot_all.rviz'

    # 聚合机器人 xacro / rviz / 控制器配置路径 (惰性求值)
    urdf_xacro = PathJoinSubstitution(
        [FindPackageShare(description_package), 'urdf', xacro_filename])
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), 'rviz', rviz_filename])
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('mycobot280_moveit2_control'),
        'config',
        'mycobot_280_real_controllers.yaml',
    ])
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('turn_on_wheeltec_robot'), 'config', 'ekf.yaml'])

    # --- Launch 参数 ------------------------------------------------------
    use_rviz = LaunchConfiguration('use_rviz')
    use_move_group = LaunchConfiguration('use_move_group')
    driver_port = LaunchConfiguration('driver_port')
    driver_baud = LaunchConfiguration('driver_baud')
    use_real_hardware = LaunchConfiguration('use_real_hardware')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否启动 RViz2')
    declare_use_move_group = DeclareLaunchArgument(
        'use_move_group', default_value='true',
        description='是否启动 MoveIt2 move_group 节点')
    declare_driver_port = DeclareLaunchArgument(
        'driver_port', default_value='/dev/mycobot_controller',
        description='机械臂串口设备路径')
    declare_driver_baud = DeclareLaunchArgument(
        'driver_baud', default_value='115200',
        description='机械臂串口波特率')
    declare_use_real_hardware = DeclareLaunchArgument(
        'use_real_hardware', default_value='true',
        description='是否连接真实硬件(串口/相机/雷达); false 时跳过硬件节点, 仅启动 RViz+MoveIt')

    # --- 通过 xacro 构建聚合机器人描述 (use_gazebo:=false, use_gripper:=true) ---
    robot_description_content = ParameterValue(
        Command([
            'xacro', ' ', urdf_xacro, ' ',
            'use_gazebo:=false', ' ',
            'use_gripper:=true',
        ]),
        value_type=str,
    )

    # 1) robot_state_publisher - 发布 /robot_description 与 TF (仅此一处 RSP)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
        }],
    )

    # 2) WHEELTEC 底盘串口驱动 (复用 base_serial.launch.py, akmcar:=false)
    #    use_real_hardware:=false 时跳过, 避免串口不存在导致崩溃
    base_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('turn_on_wheeltec_robot'),
                 'launch', 'base_serial.launch.py'])
        ),
        launch_arguments={'akmcar': 'false'}.items(),
        condition=IfCondition(use_real_hardware),
    )

    # 3) Astra Pro 相机驱动 (复用 astra_pro.launch.xml)
    #    关闭本流程不用的 IR 流与点云流, 降低 USB 带宽压力与数据丢失
    #    (检测只需要彩色图、深度图和彩色相机内参)。
    astra_pro_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('astra_camera'), 'launch', 'astra_pro.launch.xml'])
        ),
        launch_arguments={
            'enable_ir': 'false',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
        }.items(),
        condition=IfCondition(use_real_hardware),
    )

    # 4) LSLidar N10 驱动 (复用 lsn10_launch.py)
    lsn10_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('lslidar_driver'), 'launch', 'lsn10_launch.py'])
        ),
        condition=IfCondition(use_real_hardware),
    )

    # 5) base_to_gyro 静态 TF: base_link -> gyro_link (单位变换)
    base_to_gyro_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'gyro_link',
        ],
    )

    # 6) base_to_laser 静态 TF: base_footprint -> laser
    #    (与 senior_4wd_bs_robot 一致: xyz='0.0911 0 0.155' rpy='0 0 0')
    base_to_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=[
            '--x', '0.0911',
            '--y', '0.0',
            '--z', '0.155',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'laser',
        ],
    )

    # 6.1) world -> odom_combined 静态 TF (单位变换)
    #      不能在 URDF 里把 world 直接挂到 base_footprint: EKF 会动态发布
    #      odom_combined -> base_footprint, 静态 world -> base_footprint 会造成
    #      base_footprint 双父节点, TF 拆树, RViz 报
    #      "No transform from [world] to [base_footprint]"。
    #      world 挂在里程计上方形成 REP-105 链:
    #      world -> odom_combined -> base_footprint -> base_link,
    #      同时满足 mycobot_moveit_config SRDF 的固定虚拟关节
    #      (virtual_joint: world -> base_link)。
    world_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'odom_combined',
        ],
    )

    # 7) EKF 融合节点 (turn_on_wheeltec_robot/config/ekf.yaml)
    robot_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odom_combined')],
    )

    # 8) controller_manager (ros2_control_node) - 加载机械臂硬件接口与控制器
    #    use_real_hardware:=false 时跳过, 避免硬件接口初始化失败
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controllers_yaml,
        ],
        output='screen',
        remappings=[
            ('/mycobot/cmd_joint_pos', '/mycobot/cmd_joint_pos'),
            ('/mycobot/current_joint_pos', '/mycobot/current_joint_pos'),
        ],
        condition=IfCondition(use_real_hardware),
    )

    # 9) mycobot Python 驱动节点, 与机械臂通信
    #    use_real_hardware:=false 时跳过, 避免串口不存在导致崩溃
    driver_node = Node(
        package='mycobot280_moveit2_control',
        executable='mycobot_driver_node.py',
        name='mycobot_driver',
        output='screen',
        parameters=[{
            'port': driver_port,
            'baudrate': driver_baud,
        }],
        condition=IfCondition(use_real_hardware),
    )

    # 10) 依次 spawn 控制器: joint_state_broadcaster -> arm_controller
    #     -> gripper_action_controller (TimerAction + RegisterEventHandler 串联)
    #     use_real_hardware:=false 时跳过, 因为 controller_manager 未启动
    spawn_jsb_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
        condition=IfCondition(use_real_hardware))
    spawn_arm_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen',
        condition=IfCondition(use_real_hardware))
    spawn_gripper_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_action_controller'],
        output='screen',
        condition=IfCondition(use_real_hardware))
    load_arm_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_jsb_cmd,
            on_start=[TimerAction(period=2.0, actions=[spawn_arm_cmd])]))
    load_gripper_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_arm_cmd,
            on_start=[TimerAction(period=1.0, actions=[spawn_gripper_cmd])]))
    spawn_controllers_timer = TimerAction(
        period=3.0, actions=[spawn_jsb_cmd, load_arm_handler, load_gripper_handler],
        condition=IfCondition(use_real_hardware))

    # 11) joint_state_publisher (无头模式, 发布轮子等非 ros2_control 关节状态;
    #     机械臂关节由 joint_state_broadcaster 发布)
    #     use_real_hardware:=true 时, source_list 指向 /joint_states 合并 JSB 数据,
    #     避免 JSP 用 URDF 默认值导致 RViz 中机械臂抖动。
    #     use_real_hardware:=false 时, 无 JSB, JSP 直接发布 URDF 默认值。
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['/joint_states'],
        }],
        condition=IfCondition(use_real_hardware),
    )
    joint_state_publisher_fallback_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_real_hardware),
    )

    # 12) MoveIt2 move_group (使用 mycobot_moveit_config)
    #     规划坐标系 = base_footprint，与聚合 URDF 一致
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mycobot_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        ),
        launch_arguments={
            'use_rviz': 'false',
            'robot_name': 'mycobot_280',
        }.items(),
        condition=IfCondition(use_move_group),
    )

    # 13) RViz2 (默认启动), 使用本包的 all-in-one 配置
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    # --- 组装 -------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_move_group)
    ld.add_action(declare_driver_port)
    ld.add_action(declare_driver_baud)
    ld.add_action(declare_use_real_hardware)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(base_serial_launch)
    ld.add_action(astra_pro_launch)
    ld.add_action(lsn10_launch)
    ld.add_action(base_to_gyro_node)
    ld.add_action(base_to_laser_node)
    ld.add_action(world_to_odom_node)
    ld.add_action(robot_ekf_node)
    ld.add_action(controller_manager_node)
    ld.add_action(driver_node)
    ld.add_action(spawn_controllers_timer)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_fallback_node)
    # 注意: rviz_node 必须放在带 condition 的 move_group IncludeLaunchDescription
    # 之前。放在该条件 include 之后的兄弟实体会被 launch 忽略而不启动(RViz 不弹出)。
    ld.add_action(rviz_node)
    ld.add_action(move_group_launch)
    return ld
