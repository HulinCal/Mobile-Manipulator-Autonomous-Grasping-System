import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt 配置（让节点能看到 robot_description / kinematics.yaml 等）
    # 注意：mycobot_moveit_config 包把所有 yaml 放在 config/mycobot_280/ 子目录下
    # MoveItConfigsBuilder 默认在 config/ 下查找，找不到会报错
    # 所以必须显式指定每个 file_path（和 move_group.launch.py 一致）
    pkg_share = get_package_share_directory("mycobot_moveit_config")
    config_path = os.path.join(pkg_share, "config", "mycobot_280")

    kinematics_file = os.path.join(config_path, "kinematics.yaml")
    srdf_file = os.path.join(config_path, "mycobot_280.srdf")
    joint_limits_file = os.path.join(config_path, "joint_limits.yaml")
    moveit_controllers_file = os.path.join(config_path, "moveit_controllers.yaml")
    pilz_cartesian_limits_file = os.path.join(config_path, "pilz_cartesian_limits.yaml")

    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="mycobot_moveit_config")
        .trajectory_execution(file_path=moveit_controllers_file)
        .robot_description_semantic(file_path=srdf_file)
        .joint_limits(file_path=joint_limits_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file)
        .to_moveit_configs()
    )

    # 用户可传入的目标位姿参数
    declare_goal_x = DeclareLaunchArgument("goal_x", default_value="0.2")
    declare_goal_y = DeclareLaunchArgument("goal_y", default_value="0.0")
    declare_goal_z = DeclareLaunchArgument("goal_z", default_value="0.2")
    declare_goal_roll = DeclareLaunchArgument("goal_roll", default_value="0.0")
    declare_goal_pitch = DeclareLaunchArgument("goal_pitch", default_value="0.0")
    declare_goal_yaw = DeclareLaunchArgument("goal_yaw", default_value="0.0")
    # position_only=true 时只约束末端 XYZ 位置，姿态由 planner 自由选择
    declare_position_only = DeclareLaunchArgument(
        "position_only", default_value="true"
    )

    planner_node = Node(
        package="moveit2_ompl_demo",
        executable="ompl_planner_node",
        output="screen",
        parameters=[
            # 让节点能看到完整的 robot_description / semantic / kinematics
            moveit_config.to_dict(),
            # 用户传入的目标位姿
            {"goal_x": LaunchConfiguration("goal_x")},
            {"goal_y": LaunchConfiguration("goal_y")},
            {"goal_z": LaunchConfiguration("goal_z")},
            {"goal_roll": LaunchConfiguration("goal_roll")},
            {"goal_pitch": LaunchConfiguration("goal_pitch")},
            {"goal_yaw": LaunchConfiguration("goal_yaw")},
            {"position_only": LaunchConfiguration("position_only")},
        ],
    )

    return LaunchDescription(
        [
            declare_goal_x,
            declare_goal_y,
            declare_goal_z,
            declare_goal_roll,
            declare_goal_pitch,
            declare_goal_yaw,
            declare_position_only,
            planner_node,
        ]
    )
