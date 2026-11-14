#!/usr/bin/env python3
"""启动时将机械臂移动到 SRDF 里定义的 pick_ready 预抓取位姿.

用 FollowJointTrajectory 直接下发, 不走 MoveIt 规划——因为 pick_ready 是实机
已验证可达的位姿, 我们只需要从启动当前姿态平滑过渡过去即可.

用法 (launch 里调用):
    ros2 run wheeltec_mycobot_pick_place_aruco move_to_pick_ready --ros-args \
        -p action:=/arm_controller/follow_joint_trajectory
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 与 mycobot_280.srdf group_state name="pick_ready" 保持一致 (2026-11-13 实机读数)
PICK_READY_JOINTS = [
    ('link1_to_link2',           -0.015184),
    ('link2_to_link3',            1.164135),
    ('link3_to_link4',           -1.504823),
    ('link4_to_link5',            0.453960),
    ('link5_to_link6',           -0.049044),
    ('link6_to_link6_flange',    -0.711745),
]


class MoveToPickReady(Node):
    def __init__(self):
        super().__init__('move_to_pick_ready')
        self.declare_parameter('action', '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('duration', 3.0)   # 轨迹执行时长 (秒)
        action_name = self.get_parameter('action').value
        self._cli = ActionClient(self, FollowJointTrajectory, action_name)
        self.get_logger().info(f'Waiting for action server: {action_name}')

    def run(self):
        if not self._cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('FollowJointTrajectory action server not ready within 10s')
            return 1

        dur = self.get_parameter('duration').value
        traj = JointTrajectory()
        traj.joint_names = [n for n, _ in PICK_READY_JOINTS]
        pt = JointTrajectoryPoint()
        pt.positions = [v for _, v in PICK_READY_JOINTS]
        pt.time_from_start = rclpy.duration.Duration(seconds=dur).to_msg()
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info(f'Sending trajectory to pick_ready (duration={dur}s)')

        future = self._cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Goal rejected')
            return 1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        if result_future.result() is None:
            self.get_logger().warn('No result received (timeout), assuming reached')
            return 0
        status = result_future.result().result.error_code
        if status == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Reached pick_ready successfully')
            return 0
        self.get_logger().error(f'Failed with error_code={status}')
        return 1


def main():
    rclpy.init()
    node = MoveToPickReady()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
