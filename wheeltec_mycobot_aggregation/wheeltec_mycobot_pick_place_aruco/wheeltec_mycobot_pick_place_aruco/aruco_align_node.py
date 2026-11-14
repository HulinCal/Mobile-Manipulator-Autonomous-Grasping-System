#!/usr/bin/env python3
"""机械臂初始化到 pick_ready 后, 用 J1~J5 让法兰 Z 轴对准 ArUco(id=1).

流程:
  1. 启动后下发绝对关节角轨迹到 pick_ready
  2. 订阅 /aruco_single/pose, 由手眼静态 TF 转到 link1 系, 打印 marker 与夹爪位置
  3. 用 controller_state 的真实关节角做 URDF 正解(不用 /joint_states / 机械臂 TF,
     因为它们被 joint_state_publisher 的旧值污染), 把 marker 位置换算到法兰系 P
  4. 一次性解算: 在 URDF 模型里用牛顿迭代直接求出使 r 收敛的 J1~J5 绝对角,
     再把整段角度单条轨迹下发(不再小步伺服, 不再动一下停一下)
  5. 每条轨迹执行成功并静置 settle_time 后才做下一次控制, 仍不满足容差就再解算一次
  6. marker_offset_x/y/z 在 link1 系给相机解算位置加常向量补偿,
     用于抵消手眼外参的系统偏差(方向固定、约几厘米的对准偏差)
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import TransformException


# 臂 6 关节名, 顺序与 /arm_controller 一致
ARM_JOINTS = [
    'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
    'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange',
]

# 与 mycobot_280.srdf group_state name="pick_ready" 一致 (2026-11-13 实机读数)
# 顺序同 ARM_JOINTS
PICK_READY_JOINT_VALUES = [
    -0.015184,  1.164135, -1.504823,
     0.453960, -0.049044, -0.711745,
]

# link1 -> link6_flange 的 URDF 关节链: (origin xyz, origin rpy, 关节名)
# 数据来自 mycobot_description/urdf/mech/mycobot_280_arm.urdf.xacro
JOINT_CHAIN = [
    ((0.0, 0.0, 0.13156), (0.0, 0.0, math.pi / 2), 'link1_to_link2'),
    ((0.0, 0.0, -0.001), (0.0, math.pi / 2, -math.pi / 2), 'link2_to_link3'),
    ((-0.1104, 0.0, 0.0), (0.0, 0.0, 0.0), 'link3_to_link4'),
    ((-0.096, 0.0, 0.06062), (0.0, 0.0, -math.pi / 2), 'link4_to_link5'),
    ((0.0, -0.07318, 0.0), (math.pi / 2, -math.pi / 2, 0.0), 'link5_to_link6'),
    ((0.0, 0.0456, 0.0), (-math.pi / 2, 0.0, 0.0), 'link6_to_link6_flange'),
]

# 参与对准的关节: J1~J5 (J6 不动)
ALIGN_JOINTS = [
    'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
    'link4_to_link5', 'link5_to_link6',
]
JOINT_LIMIT = 2.879793   # URDF: ±165°

RETRY_INTERVAL = 1.0   # [s] 初始化下发失败后的重试间隔


# ----------------------------------------------------------------------
# 3x3 旋转 / 平移 小工具 (避免额外依赖)
# ----------------------------------------------------------------------
def _rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _mat_vec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _mat_t(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


class ArucoAlignNode(Node):

    def __init__(self):
        super().__init__('aruco_align_node')
        self.cbg = ReentrantCallbackGroup()

        # ---- 参数 ----
        self.declare_parameter('trajectory_action',
                               '/arm_controller/follow_joint_trajectory')
        self.declare_parameter('controller_state_topic',
                               '/arm_controller/controller_state')
        self.declare_parameter('init_duration', 3.0)     # [s] 初始化轨迹时长
        self.declare_parameter('marker_pose_topic', '/aruco_single/pose')
        self.declare_parameter('arm_base_frame', 'link1')
        self.declare_parameter('tcp_frame', 'link6_flange')  # 法兰/夹爪 frame
        self.declare_parameter('print_throttle', 1.0)    # [s] 打印节流
        self.declare_parameter('marker_timeout', 0.5)    # [s] marker 新鲜度
        self.declare_parameter('target_tolerance', 0.01)  # [m] 垂直距离目标
        self.declare_parameter('release_tolerance', 0.02)  # [m] 对准后重新触发阈值
        self.declare_parameter('max_total_step_deg', 60.0)  # [deg] 单次轨迹最大转角
        self.declare_parameter('max_joint_rate', 0.4)    # [rad/s] 关节转速上限
        self.declare_parameter('min_move_time', 0.5)     # [s] 轨迹最短时长
        self.declare_parameter('settle_time', 1.0)       # [s] 到位后静置时间
                                                         # (串口反馈滞后约 1s)
        self.declare_parameter('cmd_hold_time', 3.0)     # [s] 同一目标角保持期内不重复下发
        # 相机外参系统偏差补偿: marker 在 link1 系下的位置修正量 [m],
        # 修正后位置 = 相机解算值 + 该偏移; 用于抵消手眼标定/安装的恒定偏差
        self.declare_parameter('marker_offset_x', 0.0)
        self.declare_parameter('marker_offset_y', 0.0)
        self.declare_parameter('marker_offset_z', 0.0)

        p = self.get_parameter
        self.init_duration = p('init_duration').value
        self.base_frame = p('arm_base_frame').value
        self.tcp_frame = p('tcp_frame').value
        self.print_throttle = p('print_throttle').value
        self.marker_timeout = p('marker_timeout').value
        self.tolerance = p('target_tolerance').value
        self.release_tolerance = p('release_tolerance').value
        self.max_total_step = math.radians(p('max_total_step_deg').value)
        self.max_joint_rate = p('max_joint_rate').value
        self.min_move_time = p('min_move_time').value
        self.settle_time = p('settle_time').value
        self.cmd_hold_time = p('cmd_hold_time').value
        self.marker_offset = (p('marker_offset_x').value,
                              p('marker_offset_y').value,
                              p('marker_offset_z').value)

        # ---- 状态 ----
        self._joints = {}            # name -> position (controller 反馈)
        self._marker_link1 = None    # link1 系 marker 位姿
        self._marker_time = 0.0
        self._busy = False           # 有轨迹在执行
        self._init_done = False      # 已成功到达 pick_ready
        self._last_try = 0.0         # 上次初始化下发时刻(重试节流)
        self._settle_until = 0.0     # 到位后的静置截止时刻
        self._goal_kind = 'init'     # 'init' | 'align'
        self._aligned = False        # 已进入容差带(滞回, 防检测噪声抖动)
        self._last_cmd = None        # (j4, j5, 时刻) 上次下发的目标角

        # ---- TF: 只用于 相机/手眼 静态链路, 不用于机械臂关节 ----
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        # ---- 订阅 ----
        self.create_subscription(
            PoseStamped, p('marker_pose_topic').value,
            self._on_marker, 10, callback_group=self.cbg)
        # 注意: 不能订阅 /joint_states —— joint_state_publisher 也在该话题上
        # 发布旧值, 同一话题两个源会导致关节角读数被污染.
        self.create_subscription(
            JointTrajectoryControllerState, p('controller_state_topic').value,
            self._on_controller_state, 10, callback_group=self.cbg)

        # ---- 轨迹 action ----
        self.traj_cli = ActionClient(
            self, FollowJointTrajectory,
            p('trajectory_action').value,
            callback_group=self.cbg)

        # 10Hz 主循环: 初始化 -> 视觉伺服
        self.create_timer(0.1, self._tick, callback_group=self.cbg)
        self.get_logger().info(
            'ArucoAlignNode ready (pick_ready -> J1~J5 align), '
            f'marker offset (link1) = {self.marker_offset} m')

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------
    def _on_controller_state(self, msg: JointTrajectoryControllerState):
        """机械臂真实关节反馈 (硬件接口读数), 不用 /joint_states."""
        for n, v in zip(msg.joint_names, msg.feedback.positions):
            if n in ARM_JOINTS:
                self._joints[n] = v

    def _on_marker(self, msg: PoseStamped):
        """marker 位姿只转到 link1 系 (相机->link1 的静态手眼链)."""
        try:
            tf_l1 = self.tf_buf.lookup_transform(
                self.base_frame, msg.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            pose_l1 = do_transform_pose_stamped(msg, tf_l1)
        except TransformException as e:
            self.get_logger().warn(
                f'TF {msg.header.frame_id} -> {self.base_frame} failed: {e}',
                throttle_duration_sec=2.0)
            return
        # 加系统偏差补偿 (link1 系常向量, 与机械臂姿态无关)
        ox, oy, oz = self.marker_offset
        if ox or oy or oz:
            pose_l1.pose.position.x += ox
            pose_l1.pose.position.y += oy
            pose_l1.pose.position.z += oz
        self._marker_link1 = pose_l1
        self._marker_time = time.time()

    # ------------------------------------------------------------------
    # URDF 正解: link1 -> link6_flange
    # ------------------------------------------------------------------
    def _fk(self, q=None):
        """返回 (R_flange, t_flange); 关节不全返回 None.

        q 为关节角字典, 缺省用最新反馈; 求解时传入候选角度, 不动机械臂.
        """
        joints = self._joints if q is None else q
        if any(n not in joints for n in ARM_JOINTS):
            return None
        rot = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        pos = (0.0, 0.0, 0.0)
        for (xyz, rpy, name) in JOINT_CHAIN:
            pos = tuple(pos[i] + sum(rot[i][k] * xyz[k] for k in range(3))
                        for i in range(3))
            rot = _mat_mul(_mat_mul(rot, _rpy_matrix(*rpy)),
                           _rpy_matrix(0.0, 0.0, joints[name]))
        return rot, pos

    def _aim_residual(self, m, q):
        """给定候选关节角 q, 返回 marker 在法兰系下的 (Px, Py) 误差."""
        fk = self._fk(q)
        if fk is None:
            return None
        rot_fe, t_fe = fk
        rel = (m.x - t_fe[0], m.y - t_fe[1], m.z - t_fe[2])
        P = _mat_vec(_mat_t(rot_fe), rel)
        return P[0], P[1]

    def _solve_aim(self, m):
        """模型内迭代求出 J1~J5 目标角, 让法兰 Z 轴指向 marker m.

        2 维误差 / 5 个关节, 用阻尼最小二乘取最小关节变化解;
        只做计算, 不下发轨迹, 收敛后一次性把角度发下去.
        """
        # q 里保留 J6 (正解需要完整 6 关节), 只迭代 J1~J5
        q = dict(self._joints)
        h = 1e-3
        lam2 = 1e-6   # 阻尼项: marker 落在法兰 Z 轴上时雅可比退化
        for _ in range(40):
            res = self._aim_residual(m, q)
            if res is None:
                return None
            if math.hypot(res[0], res[1]) <= 0.5 * self.tolerance:
                break
            # 2x5 数值雅可比
            jac = []
            for n in ALIGN_JOINTS:
                qp = dict(q)
                qp[n] += h
                rp = self._aim_residual(m, qp)
                if rp is None:
                    return None
                jac.append(((rp[0] - res[0]) / h, (rp[1] - res[1]) / h))
            # dq = J^T * (J*J^T + lam^2*I)^-1 * (-res)
            a11 = sum(j[0] ** 2 for j in jac) + lam2
            a12 = sum(j[0] * j[1] for j in jac)
            a22 = sum(j[1] ** 2 for j in jac) + lam2
            det = a11 * a22 - a12 * a12
            if abs(det) < 1e-12:
                break
            y0 = (-res[0] * a22 + res[1] * a12) / det
            y1 = (res[0] * a12 - res[1] * a11) / det
            dq = {n: jac[i][0] * y0 + jac[i][1] * y1
                  for i, n in enumerate(ALIGN_JOINTS)}
            # 单次迭代限幅, 防止线性化失效
            step = max(abs(v) for v in dq.values())
            if step > 0.4:
                s = 0.4 / step
                dq = {n: v * s for n, v in dq.items()}
            moved = False
            for n, v in dq.items():
                nv = max(-JOINT_LIMIT, min(JOINT_LIMIT, q[n] + v))
                if abs(nv - q[n]) > 1e-6:
                    moved = True
                q[n] = nv
            if not moved:
                break
        return {n: q[n] for n in ALIGN_JOINTS}

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def _tick(self):
        if self._busy:
            return
        if not self._init_done:
            self._init_pick_ready()
            return
        if time.time() < self._settle_until:
            return
        self._align_step()

    # ------------------------------------------------------------------
    # 初始化: 回 pick_ready
    # ------------------------------------------------------------------
    def _init_pick_ready(self):
        now = time.time()
        if now - self._last_try < RETRY_INTERVAL:
            return
        self._last_try = now
        self.get_logger().info(
            f'Init: moving to pick_ready in {self.init_duration:.1f}s')
        self._goal_kind = 'init'
        self._send_positions(PICK_READY_JOINT_VALUES, self.init_duration)

    # ------------------------------------------------------------------
    # 视觉对准: J1~J5 (marker 位置 P 在法兰系)
    # ------------------------------------------------------------------
    def _align_step(self):
        if self._marker_link1 is None:
            return
        if time.time() - self._marker_time > self.marker_timeout:
            return
        fk = self._fk()
        if fk is None:
            self.get_logger().warn(
                'waiting for arm joints in controller_state',
                throttle_duration_sec=2.0)
            return
        rot_fe, t_fe = fk

        # marker 在法兰系的位置: P = R_fe^T * (p_link1 - t_fe)
        m = self._marker_link1.pose.position
        rel = (m.x - t_fe[0], m.y - t_fe[1], m.z - t_fe[2])
        P = _mat_vec(_mat_t(rot_fe), rel)
        dist = math.sqrt(P[0] ** 2 + P[1] ** 2 + P[2] ** 2)
        r = math.hypot(P[0], P[1])

        self.get_logger().info(
            f'marker id=1 in {self.base_frame}: '
            f'({m.x:.4f}, {m.y:.4f}, {m.z:.4f}) | '
            f'gripper {self.tcp_frame} in {self.base_frame}: '
            f'({t_fe[0]:.4f}, {t_fe[1]:.4f}, {t_fe[2]:.4f}) | '
            f'P in {self.tcp_frame}: ({P[0]:.4f}, {P[1]:.4f}, {P[2]:.4f}) '
            f'r={r:.4f}m',
            throttle_duration_sec=self.print_throttle)

        if dist < 0.05 or P[2] <= 0.02:
            self.get_logger().warn(
                f'marker not in front of flange '
                f'(P=({P[0]:.3f},{P[1]:.3f},{P[2]:.3f})), skip',
                throttle_duration_sec=2.0)
            return
        if r <= self.tolerance:
            if not self._aligned:
                # 刚进入容差带: 把残留误差完整算出来并打印
                ang_err = math.degrees(math.atan2(r, P[2]))
                self.get_logger().info(
                    f'target reached, residual error: '
                    f'P=({P[0]:+.4f}, {P[1]:+.4f}, {P[2]:+.4f})m | '
                    f'perpendicular r={r:.4f}m (tol {self.tolerance:.3f}m) | '
                    f'angular={ang_err:+.2f}deg | distance={dist:.4f}m')
            self._aligned = True
            self.get_logger().info(
                f'aligned: r={r:.4f}m <= {self.tolerance:.3f}m',
                throttle_duration_sec=2.0)
            return
        # 滞回: 已对准后, 误差回到 release_tolerance 以内不再动作,
        # 避免检测噪声让伺服来回小幅摆动
        if self._aligned and r <= self.release_tolerance:
            return
        self._aligned = False

        # 一次性解算: 在模型里迭代出 J1~J5 目标角, 再单条轨迹下发
        target = self._solve_aim(m)
        if target is None:
            self.get_logger().warn('aim solve failed',
                                   throttle_duration_sec=2.0)
            return
        dq = {n: target[n] - self._joints[n] for n in ALIGN_JOINTS}
        m_step = max(abs(v) for v in dq.values())
        if m_step > self.max_total_step:
            s = self.max_total_step / m_step
            dq = {n: v * s for n, v in dq.items()}
            target = {n: self._joints[n] + dq[n] for n in ALIGN_JOINTS}
        if max(abs(v) for v in dq.values()) < math.radians(0.3):
            self.get_logger().warn('no feasible J1~J5 increment',
                                   throttle_duration_sec=2.0)
            return

        # 同一目标角在 hold 期内不重复下发: 串口执行比轨迹时长慢,
        # 反馈要好几秒才追上, 重复下发相同角度只会让臂"动一下停一下"
        now = time.time()
        if self._last_cmd is not None:
            last, lt = self._last_cmd
            if (max(abs(target[n] - last[n]) for n in ALIGN_JOINTS)
                    < math.radians(0.5) and now - lt < self.cmd_hold_time):
                return
        self._last_cmd = (dict(target), now)

        positions = [target[n] if n in target else self._joints[n]
                     for n in ARM_JOINTS]
        duration = max(self.min_move_time,
                       max(abs(v) for v in dq.values()) / self.max_joint_rate)
        detail = ' '.join(
            f'J{i + 1} {math.degrees(self._joints[n]):+.2f}->'
            f'{math.degrees(target[n]):+.2f}deg'
            for i, n in enumerate(ALIGN_JOINTS))
        self.get_logger().info(
            f'aim: P=({P[0]:.4f},{P[1]:.4f},{P[2]:.4f}) r={r:.4f}m '
            f'{detail} in {duration:.2f}s')
        self._goal_kind = 'align'
        if not self._send_positions(positions, duration):
            self._settle_until = time.time() + RETRY_INTERVAL

    # ------------------------------------------------------------------
    # 轨迹下发 (无规划器)
    # ------------------------------------------------------------------
    def _send_positions(self, positions, duration: float) -> bool:
        if not self.traj_cli.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('trajectory action server not ready',
                                   throttle_duration_sec=2.0)
            return False

        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._busy = True
        future = self.traj_cli.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('trajectory goal rejected, will retry')
            self._busy = False
            self._settle_until = time.time() + RETRY_INTERVAL
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._busy = False
        result = future.result()
        code = result.result.error_code if result is not None else -1
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(
                f'trajectory error_code={code}, will retry')
            self._settle_until = time.time() + RETRY_INTERVAL
            return
        if self._goal_kind == 'init':
            self._init_done = True
            self.get_logger().info(
                'pick_ready reached, start J1~J5 align')
        else:
            # 到位后静置, 等关节反馈/TF 稳定再做下一次控制
            self._settle_until = time.time() + self.settle_time


def main():
    rclpy.init()
    node = ArucoAlignNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
