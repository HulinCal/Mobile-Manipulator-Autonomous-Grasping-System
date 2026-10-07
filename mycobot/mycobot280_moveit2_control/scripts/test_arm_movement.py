#!/usr/bin/env python3
"""
Send a FollowJointTrajectory goal to the real mycobot280 arm and a
GripperCommand goal to the gripper_action_controller.

This script is a minimal end-to-end test for the
mycobot280_moveit2_control/ros2_control hardware bridge. It sends a single
trajectory goal to /arm_controller/follow_joint_trajectory (exactly like the
MoveIt2 move_group node would do during planning) and a GripperCommand goal to
/gripper_action_controller/gripper_cmd, letting the user verify that the real
arm and gripper move.

It is the real-hardware counterpart of mycobot_system_tests'
arm_gripper_loop_controller.py (which targets the Gazebo simulation). The
gripper command position is in the 0..100 open-ratio scale used by pymycobot's
set_gripper_value()/get_gripper_value() (0 = closed, 100 = fully open).

Usage:
    # After starting mycobot_280_real_moveit.launch.py
    ros2 run mycobot280_moveit2_control test_arm_movement.py
"""

import sys
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmMovementTest(Node):
    """Sends a FollowJointTrajectory + GripperCommand goal to the real arm."""

    JOINT_NAMES = [
        'link1_to_link2',
        'link2_to_link3',
        'link3_to_link4',
        'link4_to_link5',
        'link5_to_link6',
        'link6_to_link6_flange',
    ]

    # Small, safe joint positions (radians). Home is all zeros.
    HOME_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    TARGET_POS = [0.5, -0.4, 0.3, -0.3, 0.5, -0.5]

    # Gripper open ratio in 0..100 (pymycobot native units).
    GRIPPER_OPEN = 80.0
    GRIPPER_CLOSE = 0.0

    def __init__(self):
        super().__init__('arm_movement_test')
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_action_controller/gripper_cmd')

        self.get_logger().info('Waiting for /arm_controller/follow_joint_trajectory action...')
        if not self.arm_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Arm action server not available, is the launch file running?')
            sys.exit(1)
        self.get_logger().info('Arm action server connected.')

        self.get_logger().info('Waiting for /gripper_action_controller/gripper_cmd action...')
        if not self.gripper_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().warn(
                'Gripper action server not available; gripper part of the test will be skipped.')
            self.gripper_client = None
        else:
            self.get_logger().info('Gripper action server connected.')

    def send_arm_goal(self, positions, duration_sec=4):
        """Send a single-point trajectory goal and block until it finishes."""
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=duration_sec)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.JOINT_NAMES
        goal.trajectory.points = [point]

        self.get_logger().info(f'Sending arm goal: {positions}')
        future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.result() or not future.result().accepted:
            self.get_logger().error('Arm goal rejected by the arm_controller.')
            return False

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)
        status = result_future.result().status
        self.get_logger().info(f'Arm goal finished with status={status}')
        return status == FollowJointTrajectory.Result.SUCCESSFUL

    def send_gripper_goal(self, position, max_effort=100.0):
        """Send a GripperCommand goal (0..100 open ratio) and block until done."""
        if self.gripper_client is None:
            return False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self.get_logger().info(f'Sending gripper goal: position={position}')
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.result() or not future.result().accepted:
            self.get_logger().error('Gripper goal rejected.')
            return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        status = result_future.result().status
        self.get_logger().info(f'Gripper goal finished with status={status}')
        return status == GripperCommand.Result.SUCCESSFUL

    def run(self):
        # 1) Open the gripper before moving (so the arm has nothing in hand).
        self.get_logger().info('Opening gripper...')
        self.send_gripper_goal(self.GRIPPER_OPEN)
        time.sleep(0.5)

        # 2) Move the arm to the target pose.
        self.get_logger().info('Moving arm to TARGET position...')
        ok_target = self.send_arm_goal(self.TARGET_POS, duration_sec=4)
        time.sleep(0.5)

        # 3) Close the gripper at the target pose.
        self.get_logger().info('Closing gripper...')
        self.send_gripper_goal(self.GRIPPER_CLOSE)
        time.sleep(0.5)

        # 4) Move back to home with the gripper closed.
        self.get_logger().info('Moving arm back to HOME position...')
        ok_home = self.send_arm_goal(self.HOME_POS, duration_sec=4)
        time.sleep(0.5)

        # 5) Re-open the gripper at home.
        self.get_logger().info('Re-opening gripper at home...')
        self.send_gripper_goal(self.GRIPPER_OPEN)

        if ok_target and ok_home:
            self.get_logger().info('Test PASSED: arm moved to target and back to home.')
        else:
            self.get_logger().warn('Test finished but at least one arm goal did not succeed.')


def main(args=None):
    rclpy.init(args=args)
    node = ArmMovementTest()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
