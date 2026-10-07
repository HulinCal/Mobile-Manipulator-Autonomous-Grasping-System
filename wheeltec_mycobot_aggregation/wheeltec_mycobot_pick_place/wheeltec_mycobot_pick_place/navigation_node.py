#!/usr/bin/env python3
"""Navigation node for the WHEELTEC mobile base.

Uses Nav2 ``BasicNavigator`` to drive the car to a target pose. The node
exposes a single service ``/pick_place/navigate_to`` that takes a target
pose (PoseStamped) and drives the car there. Dynamic obstacle avoidance
is handled by Nav2's local costmap (which fuses lidar + camera point
cloud), so no custom avoidance logic is needed here.

The node also exposes ``/pick_place/navigate_near`` which takes an
*object* pose (e.g. the apple) and computes an *approach pose* in front
of the object so the car's head (where the arm is mounted) faces the
object at ``approach_distance``.

Services
--------
* ``/pick_place/navigate_to``   (pick_place_srvs/NavigateTo)
    - request.target  (PoseStamped)  - desired car pose
    - response.success (bool)
* ``/pick_place/navigate_near`` (pick_place_srvs/NavigateNear)
    - request.object_pose (PoseStamped) - object the car should approach
    - response.car_pose    (PoseStamped) - the pose the car drove to
    - response.success     (bool)
* ``/pick_place/record_start``  (std_srvs/Trigger)
    - record the current car pose as the "start" pose (used for return)
* ``/pick_place/go_home``       (std_srvs/Trigger)
    - navigate back to the recorded start pose

Modes
-----
* ``sim_mode:=true`` : does not require Nav2. Each navigation request
  completes after ``sim_duration`` seconds and returns success. The
  car's current pose is tracked from TF (or assumed identity).
* ``sim_mode:=false``: uses the real Nav2 stack (must be running).
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, \
    DurabilityPolicy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import (PoseStamped, Pose, Point, PointStamped,
                               Quaternion, Twist)
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

import tf2_ros
from tf2_geometry_msgs import (do_transform_pose_stamped,
                               do_transform_point)


# ---- lightweight custom service messages ------------------------------
# We avoid a separate srv package by defining simple request/response
# helpers using built-in message types. The service interfaces are
# created dynamically from geometry_msgs/PoseStamped + a bool.
# To keep things simple and dependency-free, we use ROS2 built-in
# std_srvs/SetBool + a PoseStamped topic for passing the goal, OR we
# define the services via the message package. To avoid creating a new
# message package, we use a topic-based goal + Trigger service pattern.
#
# Implementation: goals are published to /pick_place/nav_goal
# (PoseStamped), and the node runs the navigation when the
# /pick_place/navigate service is called. For "navigate_near", the
# object pose is published to /pick_place/nav_object and the service
# computes the approach pose.
# ------------------------------------------------------------------------


def _yaw_from_quat(q: Quaternion) -> float:
    """Convert quaternion to yaw (rotation about Z)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _normalize_yaw(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        # ---- parameters --------------------------------------------------
        self.declare_parameter('sim_mode', True)
        self.declare_parameter('approach_distance', 0.35)
        self.declare_parameter('goal_tolerance_xy', 0.05)
        self.declare_parameter('goal_tolerance_yaw', 0.15)
        self.declare_parameter('sim_duration', 2.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # cmd_vel 兜底驱动: 无 Nav2/map 时用 odom TF 里程反馈直接发 /cmd_vel
        self.declare_parameter('odom_frame', 'odom_combined')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_lin_vel', 0.05)
        self.declare_parameter('max_ang_vel', 0.05)
        self.declare_parameter('nav_timeout', 30.0)
        # 安全限制: 本次行驶离起点的最大位移 (超限强制停车), 以及雷达
        # 前方最小障碍距离 (低于则立即停车)。
        self.declare_parameter('max_travel_dist', 0.5)
        self.declare_parameter('min_obstacle_dist', 0.1)
        self.declare_parameter('scan_topic', '/scan')
        # 视觉伺服 (navigate_near): 行驶中保持苹果在视野中央, 低频更新
        # 位置; 进入 final_fix_dist 精度带锁定最终位姿 (astra 结构光
        # <0.5m 失效, 0.5-1.5m 最可信), 之后用里程计走完最后一段。
        # 注意: detect ~3s/次, 观测间隔内车用旧方位角转向, 角速度上限
        # 必须与观测周期匹配 (0.1 rad/s × 3s = 0.17 rad/周期), 否则
        # 转过头导致弧线震荡。
        self.declare_parameter('servo_max_ang_vel', 0.1)
        self.declare_parameter('servo_timeout', 90.0)
        self.declare_parameter('servo_max_travel', 1.0)
        self.declare_parameter('final_fix_dist', 0.55)
        # 最后保险: 接近阶段 (视觉估计 < final_fix_dist) 雷达前方
        # 距离 < lidar_stop_dist 时立即停车, 判定已到达苹果前。
        # (苹果近处桌面/支撑物必被雷达扫到, 里程推算误差不再致命)
        self.declare_parameter('lidar_stop_dist', 0.2)
        # CPU 上单次 detect 服务 ~3s, 请求周期必须 ≥ 检测耗时, 丢失
        # 保护窗口要容纳 2 次以上的检测失败。
        self.declare_parameter('detect_period', 1.0)
        self.declare_parameter('servo_stale_timeout', 8.0)
        self.declare_parameter('detect_topic', '/pick_place/detect_apple')

        self.sim_mode = self.get_parameter('sim_mode').value
        self.approach_distance = float(
            self.get_parameter('approach_distance').value)
        self.tol_xy = float(self.get_parameter('goal_tolerance_xy').value)
        self.tol_yaw = float(self.get_parameter('goal_tolerance_yaw').value)
        self.sim_duration = float(self.get_parameter('sim_duration').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.max_lin_vel = float(self.get_parameter('max_lin_vel').value)
        self.max_ang_vel = float(self.get_parameter('max_ang_vel').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.max_travel_dist = float(
            self.get_parameter('max_travel_dist').value)
        self.min_obstacle_dist = float(
            self.get_parameter('min_obstacle_dist').value)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.servo_max_ang_vel = float(
            self.get_parameter('servo_max_ang_vel').value)
        self.servo_timeout = float(self.get_parameter('servo_timeout').value)
        self.servo_max_travel = float(
            self.get_parameter('servo_max_travel').value)
        self.final_fix_dist = float(
            self.get_parameter('final_fix_dist').value)
        self.lidar_stop_dist = float(
            self.get_parameter('lidar_stop_dist').value)
        self.detect_period = float(self.get_parameter('detect_period').value)
        self.servo_stale_timeout = float(
            self.get_parameter('servo_stale_timeout').value)
        self.detect_topic = self.get_parameter('detect_topic').value

        # ---- state -------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        # spin_thread=True: TF 订阅在独立线程更新 buffer, 服务回调里的
        # lookup/sleep 循环不会阻塞 TF 数据接收。
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=True)
        self.navigator = None
        self.start_pose = None  # recorded start pose in map frame
        self.pending_goal = None  # PoseStamped from /pick_place/nav_goal
        self.pending_object = None  # PoseStamped from /pick_place/nav_object
        self.latest_scan = None  # LaserScan for obstacle safety check

        # ---- publishers --------------------------------------------------
        self.status_pub = self.create_publisher(String, '/pick_place/status', 10)
        self.nav_result_pub = self.create_publisher(
            Bool, '/pick_place/nav_result', 10)
        self.approach_pose_pub = self.create_publisher(
            PoseStamped, '/pick_place/approach_pose', 10)
        # 停车后发布"最终苹果位姿"到专用话题 (TRANSIENT_LOCAL latched),
        # 供 grasp_node 使用。不能与检测节点共用 /pick_place/apple_pose:
        # 近距离苹果出画后检测节点会持续重发起步时的陈旧远距位姿,
        # 把 grasp 缓存的精确最终位姿覆盖掉。
        final_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.final_apple_pub = self.create_publisher(
            PoseStamped, '/pick_place/apple_pose_final', final_qos)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)

        # ---- subscribers (goal / object inputs) --------------------------
        self.create_subscription(
            PoseStamped, '/pick_place/nav_goal', self._on_goal, 10)
        self.create_subscription(
            PoseStamped, '/pick_place/nav_object', self._on_object, 10)
        # 雷达: cmd_vel 驱动的前方障碍安全检查。放可重入回调组 +
        # MultiThreadedExecutor, 服务回调阻塞期间也能持续更新。
        self.scan_cbg = ReentrantCallbackGroup()
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan,
            rclpy.qos.qos_profile_sensor_data,
            callback_group=self.scan_cbg)
        # 视觉伺服: 检测节点 latched 的最新苹果位姿 + detect 服务客户端。
        # 必须放独立可重入组: 服务回调 (伺服循环) 运行时默认组被独占,
        # 否则 client 响应/订阅回调永远无法执行 (future 永不完成)。
        self.srv_cbg = ReentrantCallbackGroup()
        latch_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.latest_apple_pose = None
        self.apple_pose_time = None
        self.create_subscription(
            PoseStamped, '/pick_place/apple_pose', self._on_apple_pose,
            latch_qos, callback_group=self.srv_cbg)
        self.detect_client = self.create_client(
            Trigger, self.detect_topic, callback_group=self.srv_cbg)

        # ---- services ----------------------------------------------------
        self.create_service(Trigger, '/pick_place/navigate', self._on_navigate)
        self.create_service(
            Trigger, '/pick_place/navigate_near', self._on_navigate_near)
        self.create_service(
            Trigger, '/pick_place/record_start', self._on_record_start)
        self.create_service(Trigger, '/pick_place/go_home', self._on_go_home)

        # ---- Nav2 BasicNavigator (real mode only) ------------------------
        if not self.sim_mode:
            try:
                from nav2_simple_commander.robot_navigator import \
                    BasicNavigator
                self.navigator = BasicNavigator()
                self.get_logger().info('Nav2 BasicNavigator initialized')
            except Exception as e:
                self.get_logger().error(
                    f'Failed to create Nav2 BasicNavigator: {e}. '
                    f'Navigation will fail. Is nav2_bringup running?')

        self.get_logger().info(
            f'navigation_node started (sim_mode={self.sim_mode})')

    # ------------------------------------------------------------------
    def _publish_status(self, text: str):
        msg = String(); msg.data = text
        self.status_pub.publish(msg)

    def _frame_available(self, frame: str, timeout: float = 0.2) -> bool:
        """Cheaply check whether ``frame`` exists in TF (no long blocking)."""
        try:
            return self.tf_buffer.can_transform(
                frame, self.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception:
            return False

    def _current_pose(self) -> PoseStamped:
        """Get current car pose via TF.

        Prefers the map frame; falls back to the odom frame (cmd_vel
        fallback mode). Returns an identity pose in odom frame if neither
        is available (sim mode / TF not up yet).
        """
        for frame in (self.map_frame, self.odom_frame):
            try:
                t = self.tf_buffer.lookup_transform(
                    frame, self.base_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                p = PoseStamped()
                p.header.frame_id = frame
                p.header.stamp = self.get_clock().now().to_msg()
                p.pose.position = Point(
                    x=t.transform.translation.x,
                    y=t.transform.translation.y,
                    z=t.transform.translation.z)
                p.pose.orientation = t.transform.rotation
                return p
            except Exception:
                continue
        if not self.sim_mode:
            self.get_logger().warn(
                f'Failed to get current pose in {self.map_frame} '
                f'or {self.odom_frame}')
        # Fallback: identity pose (car at origin)
        p = PoseStamped()
        p.header.frame_id = self.odom_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.orientation.w = 1.0
        return p

    def _on_goal(self, msg: PoseStamped):
        self.pending_goal = msg

    def _on_object(self, msg: PoseStamped):
        self.pending_object = msg

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def _on_apple_pose(self, msg: PoseStamped):
        self.latest_apple_pose = msg
        self.apple_pose_time = self.get_clock().now()

    def _front_obstacle_distance(self):
        """Min valid range in the front +/-0.5 rad sector, or None."""
        hit = self._front_obstacle_hit()
        return None if hit is None else hit[0]

    def _front_obstacle_hit(self):
        """前方 ±0.5 rad 扇区内最近点 (range, angle), 无则 None。"""
        scan = self.latest_scan
        if scan is None:
            return None
        best = None
        a = scan.angle_min
        for r in scan.ranges:
            if -0.5 <= a <= 0.5 and scan.range_min < r < scan.range_max:
                if best is None or r < best[0]:
                    best = (r, a)
            a += scan.angle_increment
        return best

    # ------------------------------------------------------------------
    def _compute_approach_pose(self, object_pose: PoseStamped) -> PoseStamped:
        """Compute a car pose so the car's front faces the object.

        The car should be ``approach_distance`` away from the object in
        the XY plane, with its +X (forward) pointing at the object.
        Works in the object's frame if the map frame is unavailable.
        """
        # Prefer map frame (Nav2 mode); fall back to odom frame (cmd_vel
        # fallback mode), then to the object's own frame.
        if self._frame_available(self.map_frame):
            work_frame = self.map_frame
        else:
            work_frame = self.odom_frame
        obj = self._to_frame(object_pose, work_frame)
        if obj is None:
            work_frame = object_pose.header.frame_id
            obj = object_pose

        car = self._current_pose()
        if car.header.frame_id != work_frame:
            car_t = self._to_frame(car, work_frame)
            if car_t is not None:
                car = car_t
            else:
                # Assume car at origin of work_frame
                car = PoseStamped()
                car.header.frame_id = work_frame
                car.header.stamp = self.get_clock().now().to_msg()
                car.pose.orientation.w = 1.0

        dx = obj.pose.position.x - car.pose.position.x
        dy = obj.pose.position.y - car.pose.position.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            yaw = _yaw_from_quat(car.pose.orientation)
            ax = car.pose.position.x
            ay = car.pose.position.y
        else:
            yaw = math.atan2(dy, dx)
            ax = obj.pose.position.x - \
                self.approach_distance * math.cos(yaw)
            ay = obj.pose.position.y - \
                self.approach_distance * math.sin(yaw)

        approach = PoseStamped()
        approach.header.frame_id = work_frame
        approach.header.stamp = self.get_clock().now().to_msg()
        approach.pose.position.x = ax
        approach.pose.position.y = ay
        approach.pose.position.z = 0.0
        approach.pose.orientation = _quat_from_yaw(yaw)
        return approach

    def _to_frame(self, pose: PoseStamped, frame: str):
        if pose.header.frame_id == frame:
            return pose
        try:
            t = self.tf_buffer.lookup_transform(
                frame, pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            return do_transform_pose_stamped(pose, t)
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return None

    # ------------------------------------------------------------------
    def _run_navigation(self, goal: PoseStamped) -> bool:
        """Drive to goal. Returns True on success.

        Route selection:
        - sim_mode: fake navigation after sim_duration seconds.
        - Nav2 available (navigator + map frame in TF): use BasicNavigator.
        - Otherwise: built-in cmd_vel fallback driver in the odom frame.
        """
        if goal is None:
            self.get_logger().warn('No goal provided')
            return False

        if self.sim_mode:
            goal_map = self._to_frame(goal, self.map_frame)
            if goal_map is None:
                goal_map = goal
            self._publish_status(
                f'Navigating to x={goal_map.pose.position.x:.2f} '
                f'y={goal_map.pose.position.y:.2f} '
                f'yaw={_yaw_from_quat(goal_map.pose.orientation):.2f}')
            self.get_logger().info(
                f'[SIM] Navigating to '
                f'({goal_map.pose.position.x:.2f}, '
                f'{goal_map.pose.position.y:.2f}) ...')
            time.sleep(self.sim_duration)
            self.get_logger().info('[SIM] Navigation succeeded')
            return True

        # ---- Nav2 route (requires navigator + map frame) -----------------
        if self.navigator is not None \
                and self._frame_available(self.map_frame):
            goal_map = self._to_frame(goal, self.map_frame)
            if goal_map is None:
                return False
            self._publish_status(
                f'Navigating to x={goal_map.pose.position.x:.2f} '
                f'y={goal_map.pose.position.y:.2f} '
                f'yaw={_yaw_from_quat(goal_map.pose.orientation):.2f}')
            try:
                self.navigator.goToPose(goal_map)
                # The node is spun by the main executor; poll for completion.
                deadline = time.time() + 300.0  # 5 min max
                while rclpy.ok() and not self.navigator.isTaskComplete() \
                        and time.time() < deadline:
                    time.sleep(0.2)
                result = self.navigator.getResult()
                ok = str(result) == 'TaskResult.SUCCEEDED' or \
                    (hasattr(result, 'name') and result.name == 'SUCCEEDED')
                self.get_logger().info(
                    f'Navigation result: {result} (ok={ok})')
                return ok
            except Exception as e:
                self.get_logger().error(f'Navigation error: {e}')
                return False

        # ---- cmd_vel fallback (no Nav2 / no map frame) -------------------
        if self._frame_available(self.odom_frame, 1.0):
            self.get_logger().info(
                'Nav2/map not available, using built-in cmd_vel driver '
                f'(odom frame: {self.odom_frame})')
            return self._cmdvel_drive(goal)

        self.get_logger().error(
            f'Neither {self.map_frame} nor {self.odom_frame} available '
            'in TF; cannot navigate')
        return False

    def _cmdvel_drive(self, goal: PoseStamped) -> bool:
        """Built-in fallback driver: P-control on /cmd_vel.

        The goal is transformed into the odom frame once, then the car is
        driven with feedback from odom->base TF: turn towards the goal,
        translate, stop at goal, align to the goal yaw. Returns True when
        the car is within ``goal_tolerance_xy`` of the goal and aligned
        within ``goal_tolerance_yaw``.
        """
        goal_odom = self._to_frame(goal, self.odom_frame)
        if goal_odom is None:
            self.get_logger().warn(
                'cmd_vel driver: cannot transform goal to '
                f'{self.odom_frame}')
            return False
        gx = goal_odom.pose.position.x
        gy = goal_odom.pose.position.y
        goal_yaw = _yaw_from_quat(goal_odom.pose.orientation)
        self._publish_status(
            f'Navigating (cmd_vel) to x={gx:.2f} y={gy:.2f} '
            f'yaw={goal_yaw:.2f}')
        self.get_logger().info(
            f'cmd_vel driver: goal ({gx:.2f}, {gy:.2f}) yaw={goal_yaw:.2f}')

        def _send(lin: float, ang: float):
            t = Twist()
            t.linear.x = lin
            t.angular.z = ang
            self.cmd_vel_pub.publish(t)

        def _cur_odom():
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            return (t.transform.translation.x, t.transform.translation.y,
                    _yaw_from_quat(t.transform.rotation))

        try:
            # 起点位姿: 用于限程 (max_travel_dist)
            try:
                sx, sy, _ = _cur_odom()
            except Exception as e:
                self.get_logger().warn(
                    f'cmd_vel driver: no odom TF at start: {e}')
                return False

            deadline = time.time() + self.nav_timeout
            while rclpy.ok() and time.time() < deadline:
                # ---- safety: lidar front obstacle ----
                obs = self._front_obstacle_distance()
                if obs is not None and obs < self.min_obstacle_dist:
                    self.get_logger().warn(
                        f'cmd_vel driver: obstacle at {obs:.2f} m '
                        f'(< {self.min_obstacle_dist:.2f} m), stopping')
                    return False

                # ---- safety: max travel distance from start ----
                try:
                    cx, cy, cyaw = _cur_odom()
                except Exception as e:
                    self.get_logger().warn(
                        f'cmd_vel driver: lost odom TF: {e}')
                    return False
                traveled = math.hypot(cx - sx, cy - sy)
                if traveled > self.max_travel_dist:
                    self.get_logger().warn(
                        f'cmd_vel driver: traveled {traveled:.2f} m exceeds '
                        f'limit {self.max_travel_dist:.2f} m, stopping')
                    return False

                dx = gx - cx
                dy = gy - cy
                dist = math.hypot(dx, dy)

                if dist > self.tol_xy:
                    e_yaw = _normalize_yaw(math.atan2(dy, dx) - cyaw)
                    if abs(e_yaw) > 0.5:
                        # turn towards the goal in place
                        _send(0.0, max(-self.max_ang_vel,
                                       min(self.max_ang_vel, 1.5 * e_yaw)))
                    else:
                        lin = min(self.max_lin_vel, 1.2 * dist)
                        # slow down on sharp heading error
                        lin *= max(0.3, 1.0 - abs(e_yaw))
                        _send(lin, max(-self.max_ang_vel,
                                       min(self.max_ang_vel, 2.0 * e_yaw)))
                    time.sleep(0.1)
                    continue

                # position reached: align final yaw
                e_final = _normalize_yaw(goal_yaw - cyaw)
                if abs(e_final) > self.tol_yaw:
                    _send(0.0, max(-self.max_ang_vel,
                                   min(self.max_ang_vel, 1.5 * e_final)))
                    time.sleep(0.1)
                    continue

                self.get_logger().info(
                    f'cmd_vel driver: goal reached '
                    f'(traveled {traveled:.2f} m, dist err {dist:.2f} m, '
                    f'yaw err {e_final:.2f} rad)')
                return True

            self.get_logger().warn(
                f'cmd_vel driver: timeout after {self.nav_timeout:.0f}s')
            return False
        finally:
            _send(0.0, 0.0)

    # ------------------------------------------------------------------
    def _lidar_hit_to_base_pose(self, hit, apple_z: float) -> PoseStamped:
        """雷达最近点 (range, angle, laser 系) → 最终苹果位姿 (base_link)。

        x 取障碍点变换到 base_link 后的 x; y 强制 0 (行驶中已对齐);
        z 用可信带观测值。
        """
        r, ang = hit
        scan_frame = self.latest_scan.header.frame_id
        ps = PointStamped()
        ps.header.frame_id = scan_frame
        ps.header.stamp = rclpy.time.Time().to_msg()
        ps.point.x = r * math.cos(ang)
        ps.point.y = r * math.sin(ang)
        ps.point.z = 0.0
        t = self.tf_buffer.lookup_transform(
            self.base_frame, scan_frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5))
        tp = do_transform_point(ps, t)
        out = PoseStamped()
        out.header.frame_id = self.base_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = tp.point.x
        out.pose.position.y = 0.0
        out.pose.position.z = apple_z
        out.pose.orientation.w = 1.0
        return out

    # ------------------------------------------------------------------
    def _visual_servo_drive(self, object_pose: PoseStamped):
        """视觉伺服接近苹果。

        行驶中保持苹果在视野中央 (方位角 P 控制), 异步调用检测服务更新
        苹果位置。停车时给出最终苹果位姿 (base_link):
          x = 停车点位置: 雷达停车时取雷达障碍点变换到 base_link 的 x;
              里程计停车时取 est_dist
          y = 0 (行驶中已左右对齐)
          z = 首次检测值 (固定, 不被途中噪声检测覆盖)
        返回 (success, message, final_pose|None)。
        """
        def _send(lin: float, ang: float):
            t = Twist()
            t.linear.x = lin
            t.angular.z = ang
            self.cmd_vel_pub.publish(t)

        def _cur_odom():
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            return (t.transform.translation.x, t.transform.translation.y,
                    _yaw_from_quat(t.transform.rotation))

        try:
            # ---- 初始观测 (base_link 系) + 第一个锚点 ----
            obs = self._to_frame(object_pose, self.base_frame)
            if obs is None:
                return False, 'no TF for initial object pose', None
            ax = obs.pose.position.x
            ay = obs.pose.position.y
            dist = math.hypot(ax, ay)
            # z 初始值 = 首次检测值, 之后只在 0.5-1.5m 可信带内更新
            apple_z = obs.pose.position.z
            try:
                cx, cy, cyaw = _cur_odom()
            except Exception as e:
                return False, f'no odom TF at start: {e}', None
            # 锚点: 每次有效观测时记录 (odom 位置, 苹果方位, 视觉距离)。
            # 观测间隔内用位移模长折减距离 —— 即使深度失效 (<0.5m
            # 结构光不可靠) 也能保证在 approach 处停车。
            anchor = {
                'x': cx, 'y': cy,
                'bearing': _normalize_yaw(cyaw + math.atan2(ay, ax)),
                'dist': dist,
            }
            last_obs_time = time.time()

            # 限程: 保证能走到苹果前 approach_distance 处
            travel_limit = max(
                self.servo_max_travel,
                dist - self.approach_distance + 0.25)
            sx, sy = cx, cy

            final_locked = False  # 已获得可信带内最终位姿, 停发检测
            warned_stale = False
            fut = None
            next_detect = 0.0
            deadline = time.time() + self.servo_timeout
            self.get_logger().info(
                f'visual servo: start, apple dist {dist:.2f} m, '
                f'bearing {math.atan2(ay, ax):.2f} rad')

            while rclpy.ok() and time.time() < deadline:
                # ---- 里程推算当前与苹果的距离 ----
                try:
                    cx, cy, cyaw = _cur_odom()
                except Exception as e:
                    return False, f'lost odom TF: {e}', None
                # 用位移模长折减 (非投影): EKF 航向与实际运动方向存在
                # 偏差时, 投影会严重低估接近量导致永不停车; 车朝苹果开
                # 时模长≈接近量, 最多略微保守 (提前停), 不会失效。
                moved = math.hypot(cx - anchor['x'], cy - anchor['y'])
                est_dist = max(0.0, anchor['dist'] - moved)
                traveled = math.hypot(cx - sx, cy - sy)

                # ---- 雷达: 减速 / 硬停 / 最后保险停车 ----
                hit = self._front_obstacle_hit()
                obs_d = None if hit is None else hit[0]
                slow = 1.0
                if obs_d is not None:
                    if obs_d < self.min_obstacle_dist:
                        msg = (f'obstacle at {obs_d:.2f} m '
                               f'(< {self.min_obstacle_dist:.2f} m), stopped')
                        self.get_logger().warn(f'visual servo: {msg}')
                        return False, msg, None
                    if obs_d < self.lidar_stop_dist:
                        # 最后保险: 已驶出起步区 (traveled > 0.15 m) 或
                        # 视觉估计已接近 → 视为到达苹果旁。
                        # 起步阶段就遇障才是真障碍, 判失败。
                        if (est_dist <= self.final_fix_dist
                                or traveled >= 0.15):
                            final_pose = self._lidar_hit_to_base_pose(
                                hit, apple_z)
                            self.get_logger().info(
                                f'visual servo: lidar stop at {obs_d:.2f} m '
                                f'(est dist {est_dist:.2f} m, traveled '
                                f'{traveled:.2f} m) -> reached apple, '
                                f'final pose x={final_pose.pose.position.x:.2f}'
                                f' z={final_pose.pose.position.z:.2f}')
                            return True, '', final_pose
                        msg = (f'obstacle at {obs_d:.2f} m right at start '
                               f'(est {est_dist:.2f} m), stopped')
                        self.get_logger().warn(f'visual servo: {msg}')
                        return False, msg, None
                    if obs_d < self.lidar_stop_dist + 0.1:
                        slow = 0.5

                # ---- 限程保护 ----
                if traveled > travel_limit:
                    msg = (f'traveled {traveled:.2f} m exceeds limit '
                           f'{travel_limit:.2f} m, stopped')
                    self.get_logger().warn(f'visual servo: {msg}')
                    return False, msg, None

                # ---- 检测请求/收割 (未锁定最终位姿前, 异步不阻塞) ----
                if (not final_locked and fut is None
                        and time.time() >= next_detect
                        and self.detect_client.service_is_ready()):
                    fut = self.detect_client.call_async(Trigger.Request())
                    next_detect = time.time() + self.detect_period
                if fut is not None and fut.done():
                    fut = None
                    ap = self.latest_apple_pose
                    if ap is not None:
                        apb = self._to_frame(ap, self.base_frame)
                        if apb is not None:
                            nax = apb.pose.position.x
                            nay = apb.pose.position.y
                            ndist = math.hypot(nax, nay)
                            anchor = {
                                'x': cx, 'y': cy,
                                'bearing': _normalize_yaw(
                                    cyaw + math.atan2(nay, nax)),
                                'dist': ndist,
                            }
                            last_obs_time = time.time()
                            warned_stale = False
                            # z 不再更新: 固定取首次检测值 (途中带内检测
                            # 的 z 可能落到桌面 0.02m 等坏值)。
                            # 进入结构光可信精度带: 锁定, 停发检测
                            if ndist <= self.final_fix_dist:
                                final_locked = True
                                self.get_logger().info(
                                    f'visual servo: final pose locked at '
                                    f'dist {ndist:.2f} m '
                                    f'(x={nax:.2f} y={nay:.2f})')

                if (not warned_stale
                        and time.time() - last_obs_time
                        > self.servo_stale_timeout):
                    # 深度在近距离不可靠属预期: 里程推算保证停车
                    self.get_logger().warn(
                        'visual servo: observation stale, '
                        'continuing on odom dead-reckoning')
                    warned_stale = True

                # ---- 到达判定: 用推算距离, 不依赖观测更新 ----
                if est_dist <= self.approach_distance + self.tol_xy:
                    final_pose = PoseStamped()
                    final_pose.header.frame_id = self.base_frame
                    final_pose.header.stamp = self.get_clock().now().to_msg()
                    final_pose.pose.position.x = est_dist
                    final_pose.pose.position.y = 0.0
                    final_pose.pose.position.z = apple_z
                    final_pose.pose.orientation.w = 1.0
                    self.get_logger().info(
                        f'visual servo: reached apple '
                        f'(est dist {est_dist:.2f} m, '
                        f'final pose {"locked" if final_locked else "NOT locked"}'
                        f', z={apple_z:.2f})')
                    return True, '', final_pose

                # ---- 方向 P 控制: 保持苹果在视野中央 ----
                # 死区 0.12 rad: 观测 ~3s 才更新一次, 小偏差持续转向
                # 会在观测延迟下震荡。
                e_yaw = _normalize_yaw(anchor['bearing'] - cyaw)
                w = 0.0 if abs(e_yaw) < 0.12 else max(
                    -self.servo_max_ang_vel,
                    min(self.servo_max_ang_vel, 2.5 * e_yaw))
                v = min(self.max_lin_vel,
                        max(0.02, 0.8 * (est_dist - self.approach_distance)))
                if abs(e_yaw) > 0.35:
                    v *= 0.3   # 大角度偏差时先转向
                _send(v * slow, w)
                time.sleep(0.1)

            self.get_logger().warn(
                f'visual servo: timeout after {self.servo_timeout:.0f}s')
            return False, 'visual servo timeout', None
        finally:
            _send(0.0, 0.0)

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------
    def _on_navigate(self, request, response):
        if self.pending_goal is None:
            response.success = False
            response.message = 'No goal set on /pick_place/nav_goal'
            return response
        ok = self._run_navigation(self.pending_goal)
        response.success = ok
        response.message = 'Navigation succeeded' if ok else 'Navigation failed'
        r = Bool(); r.data = ok
        self.nav_result_pub.publish(r)
        return response

    def _on_navigate_near(self, request, response):
        self.get_logger().info(
            f'navigate_near called (pending_object='
            f'{"set" if self.pending_object is not None else "None"})')
        if self.pending_object is None:
            response.success = False
            response.message = 'No object set on /pick_place/nav_object'
            return response
        approach = self._compute_approach_pose(self.pending_object)
        if approach is None:
            response.success = False
            response.message = 'Failed to compute approach pose'
            return response
        self.approach_pose_pub.publish(approach)
        # 视觉伺服: 行驶中保持苹果在视野中央并持续更新位置
        ok, msg, final_pose = self._visual_servo_drive(self.pending_object)
        if final_pose is not None:
            # latched 发布最终苹果位姿, grasp_node 直接使用
            self.final_apple_pub.publish(final_pose)
        response.success = ok
        response.message = (
            f'Approached apple at '
            f'({approach.pose.position.x:.2f}, '
            f'{approach.pose.position.y:.2f}) '
            f'{"success" if ok else "failed: " + msg}')
        r = Bool(); r.data = ok
        self.nav_result_pub.publish(r)
        return response

    def _on_record_start(self, request, response):
        self.get_logger().info('record_start called')
        self.start_pose = self._current_pose()
        response.success = True
        response.message = (
            f'Start pose recorded: '
            f'x={self.start_pose.pose.position.x:.2f}, '
            f'y={self.start_pose.pose.position.y:.2f}')
        self.get_logger().info(response.message)
        return response

    def _on_go_home(self, request, response):
        if self.start_pose is None:
            response.success = False
            response.message = 'No start pose recorded; call record_start first'
            return response
        # 回程 (用户要求): cmd_vel 兜底场景只倒退不转弯 — 掉头角速度
        # 被限在 0.05 rad/s, 转 180° 约需 63s, 必然超时; 且狭小场地
        # 掉头易刮碰。Nav2 可用时仍走正常导航。
        nav2_usable = self.navigator is not None \
            and self._frame_available(self.map_frame, 1.0)
        if not self.sim_mode and not nav2_usable \
                and self._frame_available(self.odom_frame, 1.0):
            ok = self._cmdvel_reverse_home()
        else:
            ok = self._run_navigation(self.start_pose)
        response.success = ok
        response.message = 'Returned home' if ok else 'Return home failed'
        r = Bool(); r.data = ok
        self.nav_result_pub.publish(r)
        return response

    def _cmdvel_reverse_home(self) -> bool:
        """倒车回起点: 不转弯, 直接负线速度沿来路退回。

        去程是视觉伺服对准苹果的近似直线, 回程直线倒车即可回到起点
        附近 (tol_xy 内)。横向偏差由去程的伺服对准保证很小; 行驶距离
        用 odom 反馈闭环, 到位即停。
        """
        goal_odom = self._to_frame(self.start_pose, self.odom_frame)
        if goal_odom is None:
            self.get_logger().warn(
                'reverse home: cannot transform start pose to '
                f'{self.odom_frame}')
            return False
        gx = goal_odom.pose.position.x
        gy = goal_odom.pose.position.y
        self._publish_status(f'Reversing home to x={gx:.2f} y={gy:.2f}')
        self.get_logger().info(
            f'reverse home: goal ({gx:.2f}, {gy:.2f})')

        def _send(lin: float):
            t = Twist()
            t.linear.x = lin
            self.cmd_vel_pub.publish(t)

        def _cur_odom():
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            return (t.transform.translation.x, t.transform.translation.y)

        try:
            try:
                sx, sy = _cur_odom()
            except Exception as e:
                self.get_logger().warn(f'reverse home: no odom TF: {e}')
                return False

            deadline = time.time() + self.nav_timeout
            while rclpy.ok() and time.time() < deadline:
                try:
                    cx, cy = _cur_odom()
                except Exception as e:
                    self.get_logger().warn(
                        f'reverse home: lost odom TF: {e}')
                    return False
                dist = math.hypot(gx - cx, gy - cy)
                if dist <= self.tol_xy:
                    self.get_logger().info(
                        f'reverse home: reached start (dist err '
                        f'{dist:.2f} m)')
                    return True
                # 与去程相同的限程: 倒退距离不超过来程 + 余量
                if math.hypot(cx - sx, cy - sy) > \
                        self.max_travel_dist + 0.3:
                    self.get_logger().warn(
                        'reverse home: travel limit exceeded, stopping')
                    return False
                # P 控制倒车, 接近时减速; 不修正横向 (不转弯)
                _send(-min(self.max_lin_vel, 1.2 * dist))
                time.sleep(0.1)

            self.get_logger().warn(
                f'reverse home: timeout after {self.nav_timeout:.0f}s')
            return False
        finally:
            _send(0.0)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    # MultiThreadedExecutor: cmd_vel 驱动循环阻塞在服务回调里时,
    # 雷达回调 (可重入组) 仍能持续更新障碍数据。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
