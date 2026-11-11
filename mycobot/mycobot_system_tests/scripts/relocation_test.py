#!/usr/bin/env python3
"""
mycobot280 重定位（重复定位）精度测试节点。

功能：
    1. 订阅 /joint_states，读到 6 个关节角后打印当前位置（度 + 弧度），便于人工量取；
    2. 启动时随机生成 N（默认 10）个关节空间内可达的目标位置并打印；
    3. 订阅 ~/target (std_msgs/Float64MultiArray)，收到 6 个关节角后通过
       /arm_controller/follow_joint_trajectory 让机械臂运动到该位置，
       到位后打印实测关节角与指令值的偏差，用于重复定位精度评估。

依赖（需先启动）：/arm_controller/follow_joint_trajectory 动作服务器、/joint_states。
"""

import math
import random

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclDuration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

# 来自 mycobot_description/urdf/mech/mycobot_280_arm.urdf.xacro 的关节限位（弧度）
JOINT_LIMITS_RAD = [
    (-2.879793, 2.879793),  # link1_to_link2
    (-2.879793, 2.879793),  # link2_to_link3
    (-2.879793, 2.879793),  # link3_to_link4
    (-2.879793, 2.879793),  # link4_to_link5
    (-2.879793, 2.879793),  # link5_to_link6
    (-3.05, 3.05),          # link6_to_link6_flange
]


class RelocationTest(Node):
    """重定位精度测试节点。"""

    def __init__(self):
        super().__init__('relocation_test')
        self._declare_parameters()
        self._create_interfaces()
        self._print_random_positions()

    # ------------------------------------------------------------------ 参数
    def _declare_parameters(self):
        self.joint_names = list(self.declare_parameter(
            'joint_names',
            ['link1_to_link2', 'link2_to_link3', 'link3_to_link4',
             'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange']).value)
        self.arm_action = str(self.declare_parameter(
            'arm_action', '/arm_controller/follow_joint_trajectory').value)
        self.joint_states_topic = str(
            self.declare_parameter('joint_states_topic', '/joint_states').value)
        # 实测来源：控制器状态（直接来自硬件状态接口）。
        # 不能用 /joint_states 当实测：arm_controller.launch.py 里 JSP 的
        # source_list 指向 /joint_states 且自身也发布 /joint_states，形成
        # 自反馈，JSB 真实值会被 JSP 锁存的旧值淹没。
        self.controller_state_topic = str(self.declare_parameter(
            'controller_state_topic', '/arm_controller/controller_state').value)
        self.angle_unit = str(self.declare_parameter('angle_unit', 'rad').value).lower()
        self.num_positions = int(
            self.declare_parameter('num_random_positions', 10).value)
        self.margin_deg = float(
            self.declare_parameter('joint_margin_deg', 20.0).value)
        self.min_separation_deg = float(
            self.declare_parameter('min_joint_separation_deg', 20.0).value)
        self.move_duration = float(
            self.declare_parameter('move_duration_sec', 4.0).value)
        self.settle_time = float(
            self.declare_parameter('settle_time_sec', 1.0).value)
        self.seed = int(self.declare_parameter('seed', -1).value)

        if self.angle_unit not in ('rad', 'deg'):
            self.get_logger().warn(
                f'angle_unit="{self.angle_unit}" 无效，改用 rad（可选 rad / deg）')
            self.angle_unit = 'rad'

        self.target_topic = f'/{self.get_name()}/target'

    def _create_interfaces(self):
        self._current = None          # 最新实测 6 关节角（弧度），来源见下
        self._state_from_controller = False
        self._current_printed = False
        self._goal_active = False
        self._last_target = None      # 最近一次下发的目标（弧度）
        self._settle_timer = None

        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action)
        self.create_subscription(
            JointTrajectoryControllerState, self.controller_state_topic,
            self._on_controller_state, 10)
        self.create_subscription(
            JointState, self.joint_states_topic, self._on_joint_state, 10)
        self.create_subscription(
            Float64MultiArray, self.target_topic, self._on_target, 10)

        self.get_logger().info(
            f'重定位测试已启动：目标话题 {self.target_topic}，'
            f'输入单位 {self.angle_unit}，动作 {self.arm_action}')
        if not self.arm_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(
                f'动作服务器 {self.arm_action} 暂不可用，收到目标时会再次尝试连接')

    # -------------------------------------------------------------- 当前位置
    def _on_controller_state(self, msg):
        """控制器状态为权威实测来源（feedback.positions 即实际关节角）。"""
        index = {name: i for i, name in enumerate(msg.joint_names)}
        if not all(name in index for name in self.joint_names):
            return
        positions = msg.feedback.positions
        if len(positions) != len(msg.joint_names):
            return
        self._current = [positions[index[name]] for name in self.joint_names]
        self._state_from_controller = True
        self._print_current_once()

    def _on_joint_state(self, msg):
        # 仅当控制器状态尚不可用时作为回退（RViz 链路的值可能滞后，仅参考）
        if self._state_from_controller:
            return
        index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in index for name in self.joint_names):
            return
        self._current = [msg.position[index[name]] for name in self.joint_names]
        self._print_current_once()

    def _print_current_once(self):
        if not self._current_printed:
            self._current_printed = True
            source = (self.controller_state_topic if self._state_from_controller
                      else f'{self.joint_states_topic}（回退，可能被 JSP 锁存）')
            self.get_logger().info(f'当前关节角来源: {source}')
            self._log_angles('当前关节角', self._current)

    # ---------------------------------------------------------- 随机目标生成
    def _random_limits(self):
        margin = math.radians(self.margin_deg)
        limits = []
        for lower, upper in JOINT_LIMITS_RAD:
            lo, hi = lower + margin, upper - margin
            if lo >= hi:  # 余量过大时退回原始限位
                lo, hi = lower, upper
            limits.append((lo, hi))
        return limits

    def _generate_random_positions(self):
        rng = random.Random(None if self.seed < 0 else self.seed)
        limits = self._random_limits()
        min_sep = math.radians(self.min_separation_deg)
        positions = []
        attempts = 0
        while len(positions) < self.num_positions and attempts < 2000:
            attempts += 1
            candidate = [rng.uniform(lo, hi) for lo, hi in limits]
            if all(max(abs(c - p) for c, p in zip(candidate, pos)) >= min_sep
                   for pos in positions):
                positions.append(candidate)
        return positions

    def _print_random_positions(self):
        positions = self._generate_random_positions()
        self.get_logger().info(
            f'已在关节限位内随机生成 {len(positions)} 个可达目标（度 / 弧度）：')
        for i, pos in enumerate(positions):
            self.get_logger().info(f'[{i:2d}] {self._format_deg(pos)}')
            self.get_logger().info(f'     {self._format_rad(pos)}')
        if not positions:
            self.get_logger().error('未生成任何目标，请检查随机参数')
            return
        self.get_logger().info(
            f'发布目标示例：ros2 topic pub --once {self.target_topic} '
            f'std_msgs/msg/Float64MultiArray "{{data: {self._format_publish(positions[0])}}}"')

    # ------------------------------------------------------------ 目标执行
    def _on_target(self, msg):
        values = list(msg.data)
        if len(values) != len(self.joint_names):
            self.get_logger().warn(
                f'目标需要 {len(self.joint_names)} 个关节角，收到 {len(values)} 个，忽略')
            return

        target = [math.radians(v) for v in values] if self.angle_unit == 'deg' else values
        if not self._within_limits(target):
            self.get_logger().warn(
                '目标超出关节限位，已忽略（限位见 JOINT_LIMITS_RAD）')
            return
        if self._goal_active:
            self.get_logger().warn('上一个目标仍在执行，忽略本次目标')
            return

        self._log_angles('下发目标', target)
        self._send_goal(target)

    def _within_limits(self, positions):
        return all(
            lower - 1e-6 <= value <= upper + 1e-6
            for value, (lower, upper) in zip(positions, JOINT_LIMITS_RAD))

    def _send_goal(self, positions):
        if not self.arm_client.server_is_ready():
            self.get_logger().info('等待动作服务器...')
            if not self.arm_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(
                    f'动作服务器 {self.arm_action} 不可用，目标未发送')
                return

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = RclDuration(seconds=self.move_duration).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.points.append(point)

        self._goal_active = True
        self._last_target = positions
        self.arm_client.send_goal_async(goal).add_done_callback(
            self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_active = False
            self.get_logger().error('目标被控制器拒绝')
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self._goal_active = False
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'目标执行失败，状态码 {result.status}')
            return
        # SUCCEEDED 按控制器轨迹时钟返回，早于机械臂经串口真正停稳；
        # 延迟 settle_time 再采样，否则实测会读到上一个位置。
        self.get_logger().info(
            f'到位完成，{self.settle_time:.1f}s 后采样实测值...')
        self._settle_timer = self.create_timer(
            self.settle_time, self._print_measured)

    def _print_measured(self):
        self._settle_timer.cancel()
        if self._current is None:
            self.get_logger().warn('尚无实测数据，无法评估到位误差')
            return
        self._log_angles('实测关节角', self._current)
        if self._last_target is not None:
            errors_deg = [math.degrees(t - c)
                          for t, c in zip(self._last_target, self._current)]
            err_str = ' '.join(f'{e:+8.2f}' for e in errors_deg)
            max_err = max(abs(e) for e in errors_deg)
            self.get_logger().info(
                f'到位误差(目标-实测) deg = [{err_str}]  最大 {max_err:.2f}°')

    # -------------------------------------------------------------- 打印工具
    def _format_deg(self, positions_rad):
        values = ' '.join(f'{math.degrees(v):8.2f}' for v in positions_rad)
        return f'deg = [{values}]'

    def _format_rad(self, positions_rad):
        values = ' '.join(f'{v:8.4f}' for v in positions_rad)
        return f'rad = [{values}]'

    def _format_publish(self, positions_rad):
        if self.angle_unit == 'deg':
            values = [round(math.degrees(v), 2) for v in positions_rad]
        else:
            values = [round(v, 4) for v in positions_rad]
        return '[' + ', '.join(str(v) for v in values) + ']'

    def _log_angles(self, prefix, positions_rad):
        self.get_logger().info(f'{prefix} {self._format_deg(positions_rad)}')
        self.get_logger().info(f'{prefix} {self._format_rad(positions_rad)}')


def main(args=None):
    rclpy.init(args=args)
    node = RelocationTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
