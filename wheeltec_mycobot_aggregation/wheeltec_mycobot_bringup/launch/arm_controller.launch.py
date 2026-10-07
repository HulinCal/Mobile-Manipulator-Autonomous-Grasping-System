#!/usr/bin/env python3
"""
启动聚合机器人(WHEELTEC R680 + myCobot 280)的机械臂控制器栈。

该 launch 文件在 mycobot_280_real_moveit.launch.py 基础上改用聚合 URDF,
启动 myCobot 280 机械臂的 ros2_control + MoveIt2 运动规划:

  1. robot_state_publisher   - 发布聚合机器人 /robot_description (use_gazebo:=false)
  2. controller_manager      - ros2_control 节点, 加载机械臂硬件接口与控制器
                               (配置文件: mycobot280_moveit2_control/config/
                                mycobot_280_real_controllers.yaml)
  3. mycobot_driver_node.py  - Python 串口驱动, 与机械臂通信
                               (默认端口 /dev/ttyACM0, 波特率 115200)
  4. load_controllers        - 依次 spawn joint_state_broadcaster -> arm_controller
                               -> gripper_action_controller (使用 TimerAction +
                               RegisterEventHandler 串联)
  5. joint_state_publisher   - (默认启动) 无头关节状态发布, 为轮子等不受 ros2_control
                               管理的关节补零位; 机械臂关节沿用 JSB 实际状态
                               (JSP 自动合并 /joint_states 中其他来源, 不覆盖真实值)
  6. move_group              - MoveIt2 规划与执行节点 (可选, 复用
                               mycobot_moveit_config 的 move_group.launch.py)

用法:
    ros2 launch wheeltec_mycobot_bringup arm_controller.launch.py
    ros2 launch wheeltec_mycobot_bringup arm_controller.launch.py \
        use_rviz:=false driver_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Launch 参数 ------------------------------------------------------
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='是否启动 RViz2 (随 move_group 启动)')
    declare_use_move_group = DeclareLaunchArgument(
        'use_move_group', default_value='true',
        description='是否启动 MoveIt2 move_group 节点')
    declare_driver_port = DeclareLaunchArgument(
        'driver_port', default_value='/dev/mycobot_controller',
        description='机械臂串口设备路径')
    declare_driver_baud = DeclareLaunchArgument(
        'driver_baud', default_value='115200',
        description='机械臂串口波特率')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='是否使用仿真时钟 (真实硬件为 false)')
    declare_use_jsp = DeclareLaunchArgument(
        'use_jsp', default_value='true',
        description='是否启动无头 joint_state_publisher (为轮子关节补零位, '
                    '机械臂关节沿用 JSB 实际状态, 保证 RViz 中整车完整显示)')

    use_rviz = LaunchConfiguration('use_rviz')
    use_move_group = LaunchConfiguration('use_move_group')
    driver_port = LaunchConfiguration('driver_port')
    driver_baud = LaunchConfiguration('driver_baud')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_jsp = LaunchConfiguration('use_jsp')

    # --- 通过 xacro 构建聚合机器人描述 (use_gazebo:=false, use_gripper:=true) ---
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare('wheeltec_mycobot_description'),
        'urdf',
        'wheeltec_mycobot.urdf.xacro',
    ])
    robot_description_content = ParameterValue(
        Command([
            'xacro', ' ', urdf_xacro, ' ',
            'use_gazebo:=false', ' ',
            'use_gripper:=true',
        ]),
        value_type=str,
    )

    # 控制器配置文件 (惰性求值)
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('mycobot280_moveit2_control'),
        'config',
        'mycobot_280_real_controllers.yaml',
    ])

    # --- 节点 --------------------------------------------------------------

    # 1) robot_state_publisher - 发布 /robot_description 与 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content,
        }],
    )

    # 2) controller_manager (ros2_control_node) - 加载硬件接口与控制器
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_content},
            controllers_yaml,
        ],
        output='screen',
        remappings=[
            ('/mycobot/cmd_joint_pos', '/mycobot/cmd_joint_pos'),
            ('/mycobot/current_joint_pos', '/mycobot/current_joint_pos'),
        ],
    )

    # 3) mycobot Python 驱动节点, 与机械臂通信
    driver_node = Node(
        package='mycobot280_moveit2_control',
        executable='mycobot_driver_node.py',
        name='mycobot_driver',
        output='screen',
        parameters=[{
            'port': driver_port,
            'baudrate': driver_baud,
        }],
    )

    # 4) 依次 spawn 控制器: joint_state_broadcaster -> arm_controller
    #    -> gripper_action_controller (使用 TimerAction + RegisterEventHandler 串联)
    spawn_jsb_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')
    spawn_arm_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen')
    spawn_gripper_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_action_controller'],
        output='screen')
    # 串联: joint_state_broadcaster -> arm_controller -> gripper_action_controller
    load_arm_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_jsb_cmd,
            on_start=[TimerAction(period=2.0, actions=[spawn_arm_cmd])]))
    load_gripper_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_arm_cmd,
            on_start=[TimerAction(period=1.0, actions=[spawn_gripper_cmd])]))
    # 延迟 3s spawn 控制器, 给硬件接口初始化时间
    spawn_controllers_timer = TimerAction(
        period=3.0, actions=[spawn_jsb_cmd, load_arm_handler, load_gripper_handler])

    # 5) joint_state_publisher (无头, 默认启动)
    #    ros2_control 的 joint_state_broadcaster 只发布机械臂 6+夹爪关节,
    #    4 个轮子关节不受其管理; JSP 从 /joint_states 合并 JSB 的真实机械臂
    #    状态, 并为轮子等关节补零位, 使整车 TF 与 RViz 显示完整。
    #    关键: source_list 必须指向 JSB 发布的 /joint_states 才能正确合并
    #    实际关节值。若为空或指向错误话题，JSP 会回退到 URDF 默认值，导致
    #    RViz 中机械臂在 home 位姿和实际位姿之间来回跳动。
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(use_jsp),
        parameters=[{
            'source_list': ['/joint_states'],
        }],
    )

    # 6) MoveIt2 move_group (使用 mycobot_moveit_config)
    #    规划坐标系 = base_footprint（聚合 URDF 根 link），避免 world 帧缺失
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mycobot_moveit_config'),
                'launch',
                'move_group.launch.py',
            ])
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
            'robot_name': 'mycobot_280',
        }.items(),
        condition=IfCondition(use_move_group),
    )

    # --- 组装 -------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_move_group)
    ld.add_action(declare_driver_port)
    ld.add_action(declare_driver_baud)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_jsp)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(driver_node)
    ld.add_action(spawn_controllers_timer)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(move_group_launch)
    return ld
