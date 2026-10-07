#!/usr/bin/env python3
"""
启动 move_group + static_camera_clound 节点。

本 launch 文件同时启动：

1. mycobot_moveit_config/move_group.launch.py —— MoveIt 2 的 move_group 节点。
   它内部的 PlanningSceneMonitor (PSM) 会订阅 /planning_scene、合并场景后
   转发到 /monitored_planning_scene 供 RViz 的 MotionPlanning 插件显示。

2. mycobot_perceptions/static_camera_clound 节点 —— 订阅
   /camera/depth_registered/points 点云，做 PCL 区域增长聚类，把每个类
   的轴对齐包围盒 (AABB) 作为 CollisionObject(BOX) 通过 PlanningScene 消息
   发布。话题默认发布到 /test_planning_scene，本 launch 自动 remap 到
   /planning_scene，使 move_group 的 PSM 直接消费。

启动后效果：RViz 的 MotionPlanning 插件场景里会出现相机视野内物体的
碰撞立方体，可参与碰撞检测与运动规划。

启动参数：
    use_sim_time (bool, default: true) —— 是否使用仿真时钟
    robot_name   (str,  default: 'mycobot_280') —— move_group 的机器人名
    use_rviz     (bool, default: true) —— 是否由 move_group.launch.py 启动 RViz
    publish_rate (double, default: 1.0) —— 兼容参数（事件驱动模式下已无实际作用）

节点本身的 PCL 参数（CropBox / 聚类 / 包围盒等）从
config/camera_perception.yaml 加载，修改 yaml 后重新启动 launch 即可生效。

示例：
    ros2 launch mycobot_perceptions static_scene.launch.py
    ros2 launch mycobot_perceptions static_scene.launch.py use_sim_time:=false

:author: hl
:date: September 2026
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- 包名 ----
    package_name = 'mycobot_perceptions'
    moveit_config_pkg = 'mycobot_moveit_config'

    # 加载 config/camera_perception.yaml 作为 static_camera_clound 节点的参数源。
    # yaml 顶层用 `/**:` 通配符，会被加载到此节点上；其中 CropBox / 聚类 / 包围盒
    # 等参数会被节点在 declare_parameter 时通过 yaml 覆盖默认值。
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    config_file = os.path.join(pkg_share, 'config', 'camera_perception.yaml')

    # ---- Launch 参数 ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    publish_rate = LaunchConfiguration('publish_rate')

    # ---- 声明 Launch 参数 ----
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='mycobot_280',
        description='Robot name passed to move_group.launch.py')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Let move_group.launch.py start RViz (its RViz config already '
                     'has MotionPlanning + PlanningScene displays configured).')

    declare_publish_rate_cmd = DeclareLaunchArgument(
        name='publish_rate', default_value='1.0',
        description='[Deprecated] Rate (Hz) for periodic re-publish. The node is '
                    'now event-driven (publishes right after each cloud is '
                    'processed), so this parameter has no effect. Kept for '
                    'backward compatibility.')

    # ---- 1. Include move_group.launch.py ----
    # 透传 use_sim_time / robot_name / use_rviz。move_group 的 PSM 默认订阅
    # /planning_scene 并把合并后的场景以 /monitored_planning_scene 发布，
    # RViz 的 MotionPlanning 插件默认即订阅此话题。
    move_group_share = FindPackageShare(package=moveit_config_pkg).find(moveit_config_pkg)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(move_group_share, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_name': robot_name,
            'use_rviz': use_rviz,
        }.items(),
    )

    # ---- 2. static_camera_clound 节点 ----
    # 把节点默认的 /test_planning_scene 重映射到 /planning_scene，使 move_group
    # 的 PSM 直接消费；QoS (transient_local + reliable) 与 PSM 期望匹配，故
    # 节点启动后即使 RViz 还没起来，PSM 一就绪也会立即拿到 latched 的那一帧。
    # parameters 顺序：先加载 yaml（提供 CropBox / 聚类 / 包围盒等大量参数），
    # 再用 dict 显式覆盖话题 / 仿真时钟等少量参数。
    static_camera_clound_node = Node(
        package=package_name,
        executable='static_camera_clound',
        name='static_camera_clound',
        output='screen',
        parameters=[
            config_file,                              # 先加载 yaml 全部参数
            {                                         # 再用 dict 覆盖少量参数
                'use_sim_time': use_sim_time,
                'point_cloud_topic': '/camera/depth_registered/points',
                'publish_rate': publish_rate,
            },
        ],
        remappings=[
            ('/test_planning_scene', '/planning_scene'),
        ],
    )

    # ---- 组装 ----
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_publish_rate_cmd)

    ld.add_action(move_group_launch)
    ld.add_action(static_camera_clound_node)

    return ld
