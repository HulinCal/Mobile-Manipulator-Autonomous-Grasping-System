#!/usr/bin/env python3
"""ArUco marker block pick-and-place test node.

检测 ArUco marker (ID=0) 并在其稳定出现 2 秒后, 用机械臂抓取 0.03m
方块, 放到右侧 60° 位置, 然后回 home 等待下一次。不启动小车。

话题:
  订阅  /aruco_single/pose (geometry_msgs/PoseStamped) - aruco_ros single
  发布  /pick_place/status (std_msgs/String) - 状态文本

TF 链路:
  link1 -> camera_color_optical_frame (easy_handeye2 手眼标定发布)
  camera_color_optical_frame -> aruco_marker_frame (aruco_ros 发布)
"""

import copy
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, Quaternion

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints, PositionConstraint,
    OrientationConstraint, JointConstraint)
from moveit_msgs.msg import BoundingVolume
from shape_msgs.msg import SolidPrimitive

from control_msgs.action import GripperCommand

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import LookupException, ExtrapolationException, TransformException


# 与 mycobot_280.srdf group_state name="pick_ready" 保持一致 (2026-11-13 实机读数)
# link1_to_link2, link2_to_link3, link3_to_link4,
# link4_to_link5, link5_to_link6, link6_to_link6_flange
PICK_READY_JOINT_VALUES = [
    -0.015184,  1.164135, -1.504823,
     0.453960, -0.049044, -0.711745,
]


class ArucoGraspNode(Node):

    def __init__(self, node_name='aruco_grasp_node'):
        super().__init__(node_name)

        # ---- aruco marker ----
        self.declare_parameter('marker_id', 1)
        self.declare_parameter('marker_size', 0.026)        # [m] marker black square
        self.declare_parameter('block_size', 0.030)         # [m] cube side
        self.declare_parameter('marker_on_top', True)       # marker on block top face
        self.declare_parameter('stable_duration', 2.0)      # [s] stable detect before grasp
        self.declare_parameter('stable_pos_tol', 0.015)     # [m] position variance tol
        self.declare_parameter('stable_ori_tol', 0.15)      # [rad] orientation variance tol
        self.declare_parameter('detect_timeout', 30.0)      # [s] max wait for stable detect

        # ---- frames ----
        self.declare_parameter('arm_base_frame', 'link1')
        self.declare_parameter('eef_link', 'link6_flange')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_frame', 'aruco_marker_frame')

        # ---- grasp geometry ----
        # link6_flange +z 到夹爪两指夹持中心的距离 [m]
        self.declare_parameter('tcp_flange_z', 0.06)
        # marker_thickness: ArUco 检测的 marker 表面到物理方块表面的距离 [m]
        self.declare_parameter('marker_thickness', 0.0)     # marker 直接在方块表面
        # grasp_depth_from_marker: 从 marker 中心下探多少到方块中央 [m]
        # 方块 0.03m, marker 在顶面, 则方块中心在 marker 下方 0.015m
        self.declare_parameter('grasp_depth_from_marker', 0.015)

        # ---- hand-eye calibration correction offsets [m] ----
        # 在手眼标定 TF 转换到 link1 系后, 对 marker 位置做附加修正.
        # 用于补偿手眼标定系统偏差, 例如实测 z 偏低需 +0.005m.
        self.declare_parameter('marker_pos_offset_x', 0.0)
        self.declare_parameter('marker_pos_offset_y', 0.0)
        self.declare_parameter('marker_pos_offset_z', 0.0)

        # ---- motion ----
        self.declare_parameter('approach_distance', 0.08)    # [m] pre-grasp above block
        self.declare_parameter('lift_distance', 0.10)       # [m] lift after grasp
        self.declare_parameter('rotate_yaw_deg', 60.0)      # [deg] rotate right after lift
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('explore_planning_time', 1.5)
        self.declare_parameter('arm_group', 'arm')
        self.declare_parameter('vel_scale_slow', 0.1)      # fine approach
        self.declare_parameter('vel_scale_fast', 0.3)      # home / lift

        # ---- gripper ----
        self.declare_parameter('gripper_open_value', 0.15)
        self.declare_parameter('gripper_close_value', -0.42)
        self.declare_parameter('gripper_settle_time', 1.5)  # [s] wait after close

        # ---- topics ----
        self.declare_parameter('marker_pose_topic', '/aruco_single/pose')
        self.declare_parameter('move_action', '/move_action')
        self.declare_parameter('gripper_action',
                               '/gripper_action_controller/gripper_cmd')

        # ---- 闭合前视觉修正 (id=2 修正标定板, 刚连在法兰上) ----
        # 修正板: 距法兰原点 correction_flange_dist, 在法兰 +z 轴上,
        #         板 z 轴与法兰 z 轴相反, 板 x,y 与法兰原点重合.
        # 仅在闭合夹爪前使用, 其他时候忽略.
        self.declare_parameter('correction_enabled', True)
        self.declare_parameter('correction_pose_topic', '/aruco_correction/pose')
        self.declare_parameter('correction_flange_dist', 0.0786)  # [m]
        self.declare_parameter('correction_max', 0.05)   # [m] 单次修正限幅
        self.declare_parameter('correction_timeout', 1.0)  # [s] 位姿新鲜度

        # ---- read params ----
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.block_size = float(self.get_parameter('block_size').value)
        self.marker_on_top = bool(self.get_parameter('marker_on_top').value)
        self.stable_duration = float(self.get_parameter('stable_duration').value)
        self.stable_pos_tol = float(self.get_parameter('stable_pos_tol').value)
        self.stable_ori_tol = float(self.get_parameter('stable_ori_tol').value)
        self.detect_timeout = float(self.get_parameter('detect_timeout').value)

        self.arm_base_frame = self.get_parameter('arm_base_frame').value
        self.eef_link = self.get_parameter('eef_link').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value

        self.tcp_flange_z = float(self.get_parameter('tcp_flange_z').value)
        self.marker_thickness = float(self.get_parameter('marker_thickness').value)
        self.grasp_depth_from_marker = float(
            self.get_parameter('grasp_depth_from_marker').value)
        self.offset_x = float(self.get_parameter('marker_pos_offset_x').value)
        self.offset_y = float(self.get_parameter('marker_pos_offset_y').value)
        self.offset_z = float(self.get_parameter('marker_pos_offset_z').value)

        self.approach_dist = float(self.get_parameter('approach_distance').value)
        self.lift_dist = float(self.get_parameter('lift_distance').value)
        self.rotate_yaw_deg = float(self.get_parameter('rotate_yaw_deg').value)
        self.planning_time = float(self.get_parameter('planning_time').value)
        self.explore_planning_time = float(
            self.get_parameter('explore_planning_time').value)
        self.arm_group = self.get_parameter('arm_group').value
        self.vel_slow = float(self.get_parameter('vel_scale_slow').value)
        self.vel_fast = float(self.get_parameter('vel_scale_fast').value)

        self.gripper_open_value = float(
            self.get_parameter('gripper_open_value').value)
        self.gripper_close_value = float(
            self.get_parameter('gripper_close_value').value)
        self.gripper_settle_time = float(
            self.get_parameter('gripper_settle_time').value)

        marker_pose_topic = self.get_parameter('marker_pose_topic').value
        move_action = self.get_parameter('move_action').value
        gripper_action = self.get_parameter('gripper_action').value

        self.correction_enabled = bool(
            self.get_parameter('correction_enabled').value)
        correction_pose_topic = self.get_parameter(
            'correction_pose_topic').value
        self.correction_flange_dist = float(
            self.get_parameter('correction_flange_dist').value)
        self.correction_max = float(self.get_parameter('correction_max').value)
        self.correction_timeout = float(
            self.get_parameter('correction_timeout').value)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- state ----
        self._latest_marker_pose = None    # PoseStamped in camera frame
        self._stable_marker_pose = None    # PoseStamped in arm_base_frame (link1)
        self._stable_start_time = None     # first stable timestamp
        self._is_stable = False
        self._running = False
        self._ref_pose = None              # reference pose for stability check
        self._correction_msg = None        # 最新 id=2 修正板位姿 (camera 系)
        self._correction_recv_time = None  # 接收时刻 (time.time())

        # ---- pub/sub ----
        self.status_pub = self.create_publisher(String, '/pick_place/status', 10)
        self.arm_stop_pub = self.create_publisher(Empty, '/mycobot/stop', 5)

        # aruco_ros single publishes /aruco_single/pose
        self.create_subscription(
            PoseStamped, marker_pose_topic, self._on_marker_pose, 10)
        # id=2 修正标定板 (仅闭合夹爪前使用, 其他时候忽略)
        self.create_subscription(
            PoseStamped, correction_pose_topic,
            self._on_correction_pose, 10)

        # ---- action clients ----
        self.cbg = ReentrantCallbackGroup()
        self.move_cli = ActionClient(
            self, MoveGroup, move_action, callback_group=self.cbg)
        self.gripper_cli = ActionClient(
            self, GripperCommand, gripper_action, callback_group=self.cbg)

        self._move_goal_handle = None

        self.get_logger().info(
            f'aruco_grasp_node started: marker_id={self.marker_id}, '
            f'stable={self.stable_duration}s, arm_base={self.arm_base_frame}')

    # --------------------------------------------------------------
    def _publish_status(self, text: str):
        msg = String(); msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    # --------------------------------------------------------------
    # 四元数小工具 (x, y, z, w 元组), 避免引入 tf_transformations
    # --------------------------------------------------------------
    @staticmethod
    def _q_mul(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def _q_conj(q):
        return (-q[0], -q[1], -q[2], q[3])

    @classmethod
    def _q_rotate(cls, q, v):
        """用四元数 q 旋转向量 v."""
        qv = (v[0], v[1], v[2], 0.0)
        r = cls._q_mul(cls._q_mul(q, qv), cls._q_conj(q))
        return (r[0], r[1], r[2])

    def _on_correction_pose(self, msg: PoseStamped):
        """只缓存最新修正板位姿, 不做任何处理 (仅闭合夹爪前读取)."""
        self._correction_msg = msg
        self._correction_recv_time = time.time()

    def _on_marker_pose(self, msg: PoseStamped):
        """接收 aruco_ros /aruco_single/pose, 稳定性判定。

        当 marker 位姿在允许容差内连续出现 stable_duration 秒, 判定为稳定。
        一旦稳定, _stable_marker_pose 被锁存, 直到 grasp 完成后重置。
        """
        self._latest_marker_pose = msg

        if self._running or self._stable_marker_pose is not None:
            return  # 抓取进行中或已有稳定锁存, 不重复判定

        now = time.time()
        if self._stable_start_time is None:
            self._stable_start_time = now
            return

        # 尝试转换到 arm_base_frame 做稳定判断 (需要手眼标定 TF 已就绪)
        try:
            pose_base = self._to_frame(msg, self.arm_base_frame)
        except TransformException:
            # TF 还没就绪, 重置计时
            self._stable_start_time = now
            return

        if self._check_stable(pose_base, now):
            elapsed = now - self._stable_start_time
            if elapsed >= self.stable_duration:
                self._stable_marker_pose = pose_base
                self._is_stable = True
                # 同时打印相机原始位姿 + link1 转换后位姿
                mp = msg.pose.position
                lp = pose_base.pose.position
                self.get_logger().info(
                    f'===== Marker STABLE for {elapsed:.1f}s =====')
                self.get_logger().info(
                    f'[camera] frame={msg.header.frame_id} '
                    f'pos=({mp.x:.3f}, {mp.y:.3f}, {mp.z:.3f}) m')
                self.get_logger().info(
                    f'[link1]  frame={pose_base.header.frame_id} '
                    f'pos=({lp.x:.3f}, {lp.y:.3f}, {lp.z:.3f}) m')
                self.get_logger().info(
                    f'==========================================')
        else:
            self._stable_start_time = now  # 不稳, 重置

    def _check_stable(self, pose_base: PoseStamped, now: float) -> bool:
        """简单稳定判定: 位置变化在容差内。"""
        if self._ref_pose is None:
            self._ref_pose = pose_base
            self._ref_time = now
            return True
        p0 = self._ref_pose.pose.position
        p1 = pose_base.pose.position
        dist = math.sqrt((p0.x - p1.x) ** 2 + (p0.y - p1.y) ** 2 +
                         (p0.z - p1.z) ** 2)
        if dist > self.stable_pos_tol:
            self._ref_pose = pose_base
            self._ref_time = now
            return False
        return True

    # --------------------------------------------------------------
    def _to_frame(self, pose: PoseStamped, frame: str) -> PoseStamped:
        if pose.header.frame_id == frame:
            return pose
        t = self.tf_buffer.lookup_transform(
            frame, pose.header.frame_id, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=2.0))
        return do_transform_pose_stamped(pose, t)

    @staticmethod
    def _spin_until(fut, timeout: float) -> bool:
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.done()

    # --------------------------------------------------------------
    def _send_move_goal(self, goal) -> bool:
        if not self.move_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('move_action server not available')
            return False
        gh_fut = self.move_cli.send_goal_async(goal)
        if not self._spin_until(gh_fut, 10.0):
            return False
        gh = gh_fut.result()
        if not gh.accepted:
            self.get_logger().warn('Move goal rejected')
            return False
        self._move_goal_handle = gh
        res_fut = gh.get_result_async()
        if not self._spin_until(res_fut, 60.0):
            return False
        res = res_fut.result().result
        ok = res.error_code.val == 1
        if ok:
            time.sleep(0.2)
        return ok

    def _move_to_pose(self, pose: PoseStamped,
                      ori_tol: float = 0.15,
                      vel_scale: float = 0.3,
                      planning_time_override: float = None,
                      retries: int = 1) -> bool:
        p = pose.pose.position
        o = pose.pose.orientation

        pc = PositionConstraint()
        pc.header.frame_id = pose.header.frame_id
        pc.link_name = self.eef_link
        pc.weight = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        from geometry_msgs.msg import Pose as GPose
        cpose = GPose()
        cpose.position.x = p.x; cpose.position.y = p.y; cpose.position.z = p.z
        cpose.orientation.w = 1.0
        bv = BoundingVolume()
        bv.primitives = [box]
        bv.primitive_poses = [cpose]
        pc.constraint_region = bv

        oc = OrientationConstraint()
        oc.header.frame_id = pose.header.frame_id
        oc.link_name = self.eef_link
        oc.orientation = o
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        cons = Constraints()
        cons.position_constraints = [pc]
        cons.orientation_constraints = [oc]

        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.num_planning_attempts = 10
        req.allowed_planning_time = (
            planning_time_override if planning_time_override is not None
            else self.planning_time)
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = vel_scale
        req.goal_constraints = [cons]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        for attempt in range(retries + 1):
            if self._send_move_goal(goal):
                return True
            if attempt < retries:
                self.get_logger().warn('Move failed, retrying once ...')
                time.sleep(0.5)
        return False

    def _move_to_joint_values(self, values) -> bool:
        names = ['link1_to_link2', 'link2_to_link3', 'link3_to_link4',
                 'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange']
        jcs = []
        for n, v in zip(names, values):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            jcs.append(jc)
        cons = Constraints(); cons.joint_constraints = jcs

        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.num_planning_attempts = 10
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_fast
        req.max_acceleration_scaling_factor = self.vel_fast
        req.goal_constraints = [cons]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        return self._send_move_goal(goal)

    def _move_gripper(self, value: float) -> bool:
        if not self.gripper_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('gripper action server not available')
            return False
        g = GripperCommand.Goal()
        g.command.position = value
        g.command.max_effort = 50.0
        gh_fut = self.gripper_cli.send_goal_async(g)
        if not self._spin_until(gh_fut, 10.0):
            return False
        gh = gh_fut.result()
        if not gh.accepted:
            return False
        res_fut = gh.get_result_async()
        if not self._spin_until(res_fut, 30.0):
            return False
        result = res_fut.result().result
        return bool(result.reached_goal or result.stalled)

    def _wait_flange_settled(self, target: PoseStamped,
                              pos_tol: float = 0.012,
                              stationary: float = 0.002,
                              timeout: float = 6.0) -> bool:
        frame = target.header.frame_id
        goal_p = target.pose.position
        deadline = time.time() + timeout
        prev = None
        while time.time() < deadline:
            try:
                tf_st = self.tf_buffer.lookup_transform(
                    frame, self.eef_link, rclpy.time.Time())
            except Exception:
                time.sleep(0.05)
                continue
            p = tf_st.transform.translation
            err = math.sqrt((p.x - goal_p.x) ** 2 +
                            (p.y - goal_p.y) ** 2 +
                            (p.z - goal_p.z) ** 2)
            if err < pos_tol and prev is not None:
                drift = math.sqrt((p.x - prev[0]) ** 2 +
                                  (p.y - prev[1]) ** 2 +
                                  (p.z - prev[2]) ** 2)
                if drift < stationary:
                    return True
            prev = (p.x, p.y, p.z)
            time.sleep(0.05)
        self.get_logger().warn(f'Flange not settled within {timeout}s')
        return False

    # --------------------------------------------------------------
    # Geometry helpers
    # --------------------------------------------------------------
    @staticmethod
    def _with(src: PoseStamped, quat: Quaternion,
              z_extra: float = 0.0, x_extra: float = 0.0,
              y_extra: float = 0.0) -> PoseStamped:
        out = PoseStamped()
        out.header = src.header
        out.pose.position.x = src.pose.position.x + x_extra
        out.pose.position.y = src.pose.position.y + y_extra
        out.pose.position.z = src.pose.position.z + z_extra
        out.pose.orientation = quat
        return out

    def _vertical_down_quat(self) -> Quaternion:
        """link6_flange z 轴竖直朝下的四元数 (俯抓)."""
        q = Quaternion()
        q.y = 1.0; q.w = 0.0
        return q

    def _yaw_rotate_quat(self, base_q: Quaternion, yaw_deg: float) -> Quaternion:
        """在 base_q 基础上绕 Z 轴再转 yaw_deg 度。"""
        # base_q 表示 link6_flange 竖直朝下, 即机械臂"对准 marker"时的姿态
        # 我们保持法兰朝向不变, 仅改变位置 (先上升到 marker 上方, 再水平移动)
        # 所以这个函数暂时不直接用, 实际旋转通过位置 + 同样姿态实现
        return base_q

    # --------------------------------------------------------------
    def _get_block_center_pose(self, marker_pose: PoseStamped) -> PoseStamped:
        """marker center -> block center (方块几何中心).

        marker 在顶面: 方块中心 = marker - (0, 0, block_size/2)
        marker 在侧面: 方块中心 = marker + (marker 法线方向 * block_size/2)
        简化处理: marker_on_top=True 时 z 方向下偏 block_size/2
        """
        out = PoseStamped()
        out.header = marker_pose.header
        out.pose.position.x = marker_pose.pose.position.x
        out.pose.position.y = marker_pose.pose.position.y
        if self.marker_on_top:
            # marker 在顶面 -> 方块中心在 marker 下方 block_size/2
            out.pose.position.z = marker_pose.pose.position.z - self.block_size / 2.0
        else:
            out.pose.position.z = marker_pose.pose.position.z
        # marker 朝向不变
        out.pose.orientation = marker_pose.pose.orientation

        # 应用手眼标定系统偏差修正 (link1 坐标系下)
        out.pose.position.x += self.offset_x
        out.pose.position.y += self.offset_y
        out.pose.position.z += self.offset_z
        return out

    # --------------------------------------------------------------
    def _correct_grasp_before_close(self,
                                    grasp_pose: PoseStamped) -> PoseStamped:
        """闭合夹爪前的视觉修正.

        id=2 修正板刚连在法兰上:
          T_flange_corr = Trans(0, 0, correction_flange_dist) * Rx(pi)
          (板在法兰 +z 轴上, 板 z 与法兰 z 相反, x,y 与法兰原点重合)
        相机测得 T_link1_corr 后反推法兰实际位姿:
          T_link1_flange = T_link1_corr * inv(T_flange_corr)
        再算实际 TCP = p_flange + R_flange*(0,0,tcp_flange_z),
        与指令期望 TCP 比较得偏差 delta, 把 grasp_pose 平移 delta.
        姿态保持不变.

        检测不到 / 数据不新鲜 / 偏差超限时跳过修正, 返回原 grasp_pose.
        """
        # 1. 取最新修正板位姿并检查新鲜度
        msg = self._correction_msg
        if msg is None or self._correction_recv_time is None:
            self.get_logger().warn(
                'Correction: no id=2 marker pose, skip correction')
            return grasp_pose
        age = time.time() - self._correction_recv_time
        if age > self.correction_timeout:
            self.get_logger().warn(
                f'Correction: id=2 pose stale ({age:.2f}s), skip correction')
            return grasp_pose

        # 2. 转到 link1 系
        try:
            corr = self._to_frame(msg, self.arm_base_frame)
        except TransformException as e:
            self.get_logger().warn(f'Correction TF failed: {e}, skip')
            return grasp_pose

        q_lc = (corr.pose.orientation.x, corr.pose.orientation.y,
                corr.pose.orientation.z, corr.pose.orientation.w)
        p_lc = (corr.pose.position.x, corr.pose.position.y,
                corr.pose.position.z)

        # 3. T_flange_corr 的逆: R_fc = Rx(pi) -> q=(1,0,0,0)
        q_fc = (1.0, 0.0, 0.0, 0.0)
        t_fc = (0.0, 0.0, self.correction_flange_dist)
        q_cf = self._q_conj(q_fc)
        # inv 平移 = -R_cf * t_fc
        t_inv = tuple(-v for v in self._q_rotate(q_cf, t_fc))

        # 4. 法兰实际位姿 T_link1_flange = T_link1_corr * inv(T_flange_corr)
        q_lf = self._q_mul(q_lc, q_cf)
        rot = self._q_rotate(q_lc, t_inv)
        p_lf = (p_lc[0] + rot[0], p_lc[1] + rot[1], p_lc[2] + rot[2])

        # 5. 实际 TCP (法兰 +z 方向 tcp_flange_z)
        tcp_off = self._q_rotate(q_lf, (0.0, 0.0, self.tcp_flange_z))
        tcp_actual = (p_lf[0] + tcp_off[0], p_lf[1] + tcp_off[1],
                      p_lf[2] + tcp_off[2])

        # 6. 指令期望 TCP (按当前 grasp_pose 指令位姿推)
        q_cmd = (grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y,
                 grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w)
        cmd_off = self._q_rotate(q_cmd, (0.0, 0.0, self.tcp_flange_z))
        tcp_desired = (grasp_pose.pose.position.x + cmd_off[0],
                       grasp_pose.pose.position.y + cmd_off[1],
                       grasp_pose.pose.position.z + cmd_off[2])

        delta = (tcp_desired[0] - tcp_actual[0],
                 tcp_desired[1] - tcp_actual[1],
                 tcp_desired[2] - tcp_actual[2])
        norm = math.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)
        self.get_logger().info(
            f'Correction: tcp_actual=({tcp_actual[0]:.3f},'
            f'{tcp_actual[1]:.3f},{tcp_actual[2]:.3f}) '
            f'tcp_desired=({tcp_desired[0]:.3f},{tcp_desired[1]:.3f},'
            f'{tcp_desired[2]:.3f})')
        self.get_logger().info(
            f'Correction delta: dx={delta[0]*1000:+.1f}mm '
            f'dy={delta[1]*1000:+.1f}mm dz={delta[2]*1000:+.1f}mm '
            f'(|d|={norm*1000:.1f}mm)')

        if norm > self.correction_max:
            self.get_logger().warn(
                f'Correction |d|={norm:.3f}m > max={self.correction_max}m, '
                'skip correction (suspect bad detection)')
            return grasp_pose

        if norm < 0.001:
            self.get_logger().info('Correction < 1mm, no move needed')
            return grasp_pose

        # 7. 平移修正 grasp_pose, 姿态不变, 慢速移动
        corrected = PoseStamped()
        corrected.header = grasp_pose.header
        corrected.pose = copy.deepcopy(grasp_pose.pose)
        corrected.pose.position.x += delta[0]
        corrected.pose.position.y += delta[1]
        corrected.pose.position.z += delta[2]

        self._publish_status('Step 2.5: applying visual correction')
        if not self._move_to_pose(corrected, vel_scale=self.vel_slow):
            self.get_logger().warn(
                'Correction move failed, try loose ori')
            if not self._move_to_pose(corrected, vel_scale=self.vel_slow,
                                      ori_tol=0.5):
                self.get_logger().warn(
                    'Correction move failed, close at current position')
                return grasp_pose
        return corrected

    # --------------------------------------------------------------
    def _flange_target_vertical(self, block_center: PoseStamped,
                                z_offset: float = 0.0,
                                x_offset: float = 0.0,
                                y_offset: float = 0.0) -> PoseStamped:
        """竖抓目标: 法兰 z 轴朝下, 法兰在方块中心上方 tcp_flange_z + z_offset."""
        q = self._vertical_down_quat()
        out = PoseStamped()
        out.header = block_center.header
        out.pose.position.x = block_center.pose.position.x + x_offset
        out.pose.position.y = block_center.pose.position.y + y_offset
        # 法兰 z 轴朝下, 所以法兰应在 block_center.z + tcp_flange_z (TCP 在 -z 方向)
        # 但 marker 定位的是方块顶面, 夹爪应从侧面夹取
        # 简化: 法兰 z = block_center.z + tcp_flange_z + z_offset
        out.pose.position.z = block_center.pose.position.z + \
                              self.tcp_flange_z + z_offset
        out.pose.orientation = q
        return out

    # --------------------------------------------------------------
    def run_pick_place_cycle(self) -> bool:
        """执行一次完整的 pick-and-place 循环。"""
        # 先取本地变量, 再清状态锁存 (顺序不能反!)
        marker_pose = self._stable_marker_pose
        if marker_pose is None:
            self.get_logger().warn('run_pick_place_cycle called without stable marker')
            return False

        self._running = True
        self._stable_marker_pose = None
        self._stable_start_time = None
        self._ref_pose = None

        # marker 中心在 link1 系
        # 方块中心 = marker - (0, 0, block_size/2) (marker 在顶面)
        block_center = self._get_block_center_pose(marker_pose)

        # 同时打印相机原始位姿 + link1 转换后位姿 + 方块中心
        cam_pose = self._latest_marker_pose
        if cam_pose is not None:
            cp = cam_pose.pose.position
            self.get_logger().info(
                f'[camera] aruco_marker raw: '
                f'({cp.x:.3f}, {cp.y:.3f}, {cp.z:.3f}) m')
        mp = marker_pose.pose.position
        bc = block_center.pose.position
        self.get_logger().info(
            f'[link1]  marker:    ({mp.x:.3f}, {mp.y:.3f}, {mp.z:.3f}) m')
        self.get_logger().info(
            f'[link1]  block_cen: ({bc.x:.3f}, {bc.y:.3f}, {bc.z:.3f}) m')
        if self.offset_x or self.offset_y or self.offset_z:
            self.get_logger().info(
                f'Hand-eye correction offset: '
                f'dx={self.offset_x*1000:+.1f}mm '
                f'dy={self.offset_y*1000:+.1f}mm '
                f'dz={self.offset_z*1000:+.1f}mm')

        # ----------------------------------------------------------
        # Step 0: pick_ready + open gripper (parallel)
        # ----------------------------------------------------------
        self._publish_status('Step 0: moving to pick_ready + opening gripper')
        results = {}

        def _do_home():
            results['home'] = self._move_to_joint_values(PICK_READY_JOINT_VALUES)

        def _do_open():
            results['open'] = self._move_gripper(self.gripper_open_value)

        t_home = threading.Thread(target=_do_home)
        t_open = threading.Thread(target=_do_open)
        t_home.start(); t_open.start()
        t_home.join(); t_open.join()

        if not results.get('home'):
            self.get_logger().error('Cannot reach home')
            self._running = False
            return False
        if not results.get('open'):
            self.get_logger().error('Cannot open gripper')
            self._running = False
            return False
        time.sleep(1.0)

        # ----------------------------------------------------------
        # Step 1: pre-grasp (above block)
        # ----------------------------------------------------------
        self._publish_status('Step 1: pre-grasp above block')
        pre_grasp = self._flange_target_vertical(
            block_center, z_offset=self.approach_dist)
        self.get_logger().info(
            f'Pre-grasp target: ({pre_grasp.pose.position.x:.3f},'
            f'{pre_grasp.pose.position.y:.3f},'
            f'{pre_grasp.pose.position.z:.3f})')

        if not self._move_to_pose(pre_grasp, vel_scale=self.vel_fast,
                                  ori_tol=0.5):
            # try tucked (move inward a bit) if direct fails
            self.get_logger().warn('Direct pre-grasp failed, trying tucked')
            pre_grasp_tucked = self._flange_target_vertical(
                block_center, z_offset=self.approach_dist, y_offset=-0.08)
            if not self._move_to_pose(pre_grasp_tucked, vel_scale=self.vel_fast,
                                      ori_tol=1.0):
                self.get_logger().error('Cannot reach pre-grasp')
                self._running = False
                return False

        # ----------------------------------------------------------
        # Step 2: slow approach (vertical down to grasp position)
        # ----------------------------------------------------------
        self._publish_status('Step 2: slow vertical approach')
        # grasp 位置: 法兰在 block_center.z + tcp_flange_z
        # 但要让夹爪刚好夹住方块侧面, 实际需要再下探一点
        # 这里直接到 block_center 上方 tcp_flange_z 处
        grasp_pose = self._flange_target_vertical(
            block_center, z_offset=0.0)

        # 如果 pre-grasp 用了 tucked, 这里也 tucked
        if 'pre_grasp_tucked' in dir():
            pass  # already tried above
        if not self._move_to_pose(grasp_pose, vel_scale=self.vel_slow):
            self.get_logger().warn('Direct grasp failed, trying tucked')
            grasp_pose = self._flange_target_vertical(
                block_center, z_offset=0.0, y_offset=-0.06)
            if not self._move_to_pose(grasp_pose, vel_scale=self.vel_slow):
                self.get_logger().error('Cannot reach grasp position')
                self._running = False
                return False

        # wait settle
        if not self._wait_flange_settled(grasp_pose):
            self.get_logger().warn('Flange not settled, re-commanding')
            self._move_to_pose(grasp_pose, vel_scale=self.vel_slow)

        # ----------------------------------------------------------
        # Step 2.5: 闭合前视觉修正 (id=2 修正板反推法兰实际位姿)
        # ----------------------------------------------------------
        if self.correction_enabled:
            grasp_pose = self._correct_grasp_before_close(grasp_pose)

        # ----------------------------------------------------------
        # Step 3: close gripper
        # ----------------------------------------------------------
        self._publish_status('Step 3: closing gripper')
        if not self._move_gripper(self.gripper_close_value):
            self.get_logger().error('Cannot close gripper')
            self._running = False
            return False
        time.sleep(self.gripper_settle_time)

        # ----------------------------------------------------------
        # Step 4: lift up ~0.10m
        # ----------------------------------------------------------
        self._publish_status(f'Step 4: lifting {self.lift_dist:.2f}m')
        lift_pose = self._flange_target_vertical(
            block_center, z_offset=self.lift_dist)
        # lift 是竖直抬升, 无需严格保持垂直姿态 (机械臂自然姿态即可)
        # ori_tol=0.5rad≈28° 足够宽松, 大幅提高规划成功率
        if not self._move_to_pose(lift_pose, vel_scale=self.vel_fast,
                                  ori_tol=0.5):
            self.get_logger().warn('Lift strict failed, trying tucked + loose ori')
            lift_t = self._flange_target_vertical(
                block_center, z_offset=self.lift_dist, y_offset=-0.08)
            if not self._move_to_pose(lift_t, vel_scale=self.vel_fast,
                                      ori_tol=1.0):
                self.get_logger().warn('Lift tucked failed, try more tucked')
                lift_t2 = self._flange_target_vertical(
                    block_center, z_offset=self.lift_dist,
                    y_offset=-0.12, x_offset=-0.05)
                if not self._move_to_pose(lift_t2, vel_scale=self.vel_fast,
                                          ori_tol=1.0):
                    self.get_logger().error('Cannot lift')
                    self._running = False
                    return False

        # ----------------------------------------------------------
        # Step 5: open gripper (drop block at lifted height)
        # ----------------------------------------------------------
        self._publish_status('Step 5: opening gripper (drop block)')
        if not self._move_gripper(self.gripper_open_value):
            self.get_logger().error('Cannot open gripper')
            self._running = False
            return False
        time.sleep(1.0)

        # ----------------------------------------------------------
        # Step 6: return pick_ready
        # ----------------------------------------------------------
        self._publish_status('Step 6: returning pick_ready')
        if not self._move_to_joint_values(PICK_READY_JOINT_VALUES):
            self.get_logger().warn('Cannot return pick_ready (non-fatal)')

        self._publish_status('Cycle COMPLETE. Waiting for next marker ...')
        self._running = False
        self._stable_marker_pose = None  # reset for next detection
        self._stable_start_time = None
        self._ref_pose = None
        return True

    # --------------------------------------------------------------
    def run(self):
        """主循环: 等待 marker 稳定 -> 执行抓取循环."""
        self._publish_status('Waiting for stable ArUco marker ...')
        while rclpy.ok():
            if self._stable_marker_pose is not None and not self._running:
                self.run_pick_place_cycle()
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoGraspNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)

    # 在另一个线程上跑主循环
    def _main_loop():
        node.run()

    t = threading.Thread(target=_main_loop, daemon=True)
    t.start()

    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
