#!/usr/bin/env python3
"""Coordinator node - state machine orchestrating the pick-and-place.

States
------
IDLE -> DETECTING -> NAV_TO_APPLE -> GRASPING -> NAV_TO_DELIVERY
      -> DONE (back to IDLE, apple kept held)

On receiving a trigger (from /pick_place/trigger or manual service), the
coordinator drives the full pipeline:

  1. DETECTING     : call /pick_place/detect_apple  -> apple pose
  2. RECORD_START  : call /pick_place/record_start  -> save home pose
  3. NAV_TO_APPLE  : pub apple pose -> /pick_place/nav_object
                     call /pick_place/navigate_near
  4. GRASPING      : (apple pose already on /pick_place/apple_pose)
                     call /pick_place/pick (含夹取后收臂回 home)
  5. NAV_TO_DELIVERY: call /pick_place/go_home 倒车返回起点
  6. DONE          : 到起点即结束, 不放置、不张爪, 苹果保持夹持

注: /pick_place/place 服务仍存在, 需要时可单独手动调用, 但自动流水线不再调用。

Services
--------
* ``/pick_place/start`` (std_srvs/Trigger) - manually start the pipeline
  (equivalent to receiving a trigger).

Topics
------
* Sub: ``/pick_place/trigger`` (std_msgs/Bool)
* Pub: ``/pick_place/state`` (std_msgs/String) - current state name
* Pub: ``/pick_place/status`` (std_msgs/String) - human readable status
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_srvs.srv import Trigger


def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _yaw_from_quat(q: Quaternion) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# State names
S_IDLE = 'IDLE'
S_DETECTING = 'DETECTING'
S_NAV_TO_APPLE = 'NAV_TO_APPLE'
S_GRASPING = 'GRASPING'
S_NAV_TO_DELIVERY = 'NAV_TO_DELIVERY'
S_PLACING = 'PLACING'
S_NAV_HOME = 'NAV_HOME'
S_DONE = 'DONE'
S_ERROR = 'ERROR'


class CoordinatorNode(Node):

    def __init__(self):
        super().__init__('coordinator_node')

        # ---- parameters --------------------------------------------------
        self.declare_parameter('sim_mode', True)
        self.declare_parameter('delivery_offset_x', 0.5)
        self.declare_parameter('delivery_offset_y', 0.5)
        self.declare_parameter('delivery_offset_yaw', 0.0)
        self.declare_parameter('auto_return', True)
        self.declare_parameter('state_publish_rate', 2.0)

        self.sim_mode = self.get_parameter('sim_mode').value
        self.delivery_offset_x = float(
            self.get_parameter('delivery_offset_x').value)
        self.delivery_offset_y = float(
            self.get_parameter('delivery_offset_y').value)
        self.delivery_offset_yaw = float(
            self.get_parameter('delivery_offset_yaw').value)
        self.auto_return = self.get_parameter('auto_return').value

        # ---- state -------------------------------------------------------
        self.state = S_IDLE
        self.running = False
        self.start_pose = None  # PoseStamped (map frame)
        self.apple_pose = None

        # ---- publishers --------------------------------------------------
        self.state_pub = self.create_publisher(String, '/pick_place/state', 10)
        self.status_pub = self.create_publisher(String, '/pick_place/status', 10)
        self.nav_object_pub = self.create_publisher(
            PoseStamped, '/pick_place/nav_object', 10)
        self.nav_goal_pub = self.create_publisher(
            PoseStamped, '/pick_place/nav_goal', 10)
        # 与 apple_detection_node 的 pose_pub 保持一致的 TRANSIENT_LOCAL,
        # 否则同话题不同 Durability 会让 subscriber 降级到 VOLATILE,
        # 导致 latched 失效。
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.apple_pose_pub = self.create_publisher(
            PoseStamped, '/pick_place/apple_pose', latched_qos)
        self.place_pose_pub = self.create_publisher(
            PoseStamped, '/pick_place/place_pose', 10)

        # ---- subscribers -------------------------------------------------
        self.create_subscription(
            Bool, '/pick_place/trigger', self._on_trigger, 10)

        # ---- service clients ---------------------------------------------
        self.detect_cli = self.create_client(Trigger, '/pick_place/detect_apple')
        self.nav_near_cli = self.create_client(
            Trigger, '/pick_place/navigate_near')
        self.nav_to_cli = self.create_client(Trigger, '/pick_place/navigate')
        self.record_cli = self.create_client(Trigger, '/pick_place/record_start')
        self.go_home_cli = self.create_client(Trigger, '/pick_place/go_home')
        self.pick_cli = self.create_client(Trigger, '/pick_place/pick')
        self.place_cli = self.create_client(Trigger, '/pick_place/place')

        # ---- service -----------------------------------------------------
        self.create_service(Trigger, '/pick_place/start', self._on_start)

        # ---- timer for state publishing ----------------------------------
        rate = float(self.get_parameter('state_publish_rate').value)
        self.create_timer(1.0 / max(rate, 0.1), self._publish_state)

        self.get_logger().info(
            f'coordinator_node started (sim_mode={self.sim_mode})')
        self._set_state(S_IDLE, 'Ready. Waiting for trigger on '
                                '/pick_place/trigger (say "把苹果拿过来" '
                                'or call /pick_place/start).')

    # ------------------------------------------------------------------
    def _set_state(self, state: str, status: str = ''):
        self.state = state
        self.get_logger().info(f'== STATE: {state} == {status}')
        if status:
            self._publish_status(f'[{state}] {status}')
        s = String(); s.data = state
        self.state_pub.publish(s)

    def _publish_status(self, text: str):
        msg = String(); msg.data = text
        self.status_pub.publish(msg)

    def _publish_state(self):
        s = String(); s.data = self.state
        self.state_pub.publish(s)

    # ------------------------------------------------------------------
    def _on_trigger(self, msg: Bool):
        if msg.data and not self.running:
            self.get_logger().info('Trigger received! Starting pipeline.')
            import threading
            threading.Thread(target=self._start_pipeline, daemon=True).start()

    def _on_start(self, request, response):
        if self.running:
            response.success = False
            response.message = 'Pipeline already running'
            return response
        import threading
        threading.Thread(target=self._start_pipeline, daemon=True).start()
        response.success = True
        response.message = 'Pipeline started'
        return response

    # ------------------------------------------------------------------
    def _call_service(self, client, timeout=30.0):
        """Call a Trigger service and return (success, message).

        Uses call_async + polling so it can be invoked from a worker
        thread while the node is being spun by the main executor.
        """
        if not client.wait_for_service(timeout_sec=5.0):
            return False, f'Service {client.srv_name} not available'
        req = Trigger.Request()
        future = client.call_async(req)
        deadline = time.time() + timeout
        while rclpy.ok() and not future.done() and time.time() < deadline:
            time.sleep(0.1)
        if not future.done():
            return False, f'Service {client.srv_name} timed out'
        try:
            resp = future.result()
            return resp.success, resp.message
        except Exception as e:
            return False, str(e)

    # ------------------------------------------------------------------
    def _start_pipeline(self):
        """Run the full pipeline (blocking, called from a service/trigger)."""
        self.running = True
        try:
            # ---- 1. DETECTING --------------------------------------------
            self._set_state(S_DETECTING, 'Detecting apple ...')
            ok, msg = self._call_service(self.detect_cli, timeout=15.0)
            if not ok:
                self._fail('Apple detection failed', msg)
                return
            # The detection node publishes the apple pose; we also capture
            # the latest via a short subscription (or re-read from topic).
            # Here we rely on the published pose being latched by the
            # detection node. To be safe, we re-fetch via the service's
            # message which contains the pose string.
            self.get_logger().info(f'Detection result: {msg}')
            # Give the pose a moment to be published
            time.sleep(0.5)

            # ---- 2. RECORD START -----------------------------------------
            self._set_state(S_DETECTING, 'Recording start pose ...')
            ok, msg = self._call_service(self.record_cli, timeout=10.0)
            if not ok:
                self._fail('Failed to record start pose', msg)
                return
            self.get_logger().info(f'Start recorded: {msg}')

            # ---- 3. NAV TO APPLE -----------------------------------------
            self._set_state(S_NAV_TO_APPLE, 'Navigating near the apple ...')
            # The apple pose is on /pick_place/apple_pose; re-publish to
            # /pick_place/nav_object for the navigation node.
            # We get the pose by subscribing briefly.
            apple = self._wait_for_pose('/pick_place/apple_pose', 5.0)
            if apple is None:
                self._fail('No apple pose available', '')
                return
            self.nav_object_pub.publish(apple)
            time.sleep(0.2)
            ok, msg = self._call_service(self.nav_near_cli, timeout=120.0)
            if not ok:
                self._fail('Navigation to apple failed', msg)
                return

            # ---- 4. GRASPING ---------------------------------------------
            self._set_state(S_GRASPING, 'Picking up the apple ...')
            # 不再重新检测: navigate_near 停车时已在
            # /pick_place/apple_pose_final 发布最终位姿 (x=雷达/里程计
            # 停车位置, y=0, z=可信带值)。此时 <0.5m 结构光不可靠,
            # 重检测只会发布坏位姿。
            ok, msg = self._call_service(self.pick_cli, timeout=120.0)
            if not ok:
                self._fail('Pick failed', msg)
                return

            # ---- 5. NAV BACK TO START ------------------------------------
            # 用户要求: 抓取成功后小车返回起点即结束, 不放置、不张开夹爪
            # (苹果保持夹在机械臂上, 由现场人工取走)。
            # 直接调用 go_home (navigation_node 已记录起点), 避免
            # _compute_delivery_pose 用"当前位姿+偏移"导致目标过远。
            self._set_state(S_NAV_TO_DELIVERY,
                            'Returning to start pose (keep apple) ...')
            ok, msg = self._call_service(self.go_home_cli, timeout=120.0)
            if not ok:
                self._fail('Return to start failed', msg)
                return

            # 到起点后不执行 place: 保持夹爪夹紧、不松爪, 直接结束。
            self._set_state(S_DONE, 'Pick-and-return complete '
                                    '(apple still held)!')
            self.get_logger().info(
                'Pipeline finished successfully (returned with apple held)')

        except Exception as e:
            self._fail('Unexpected error', str(e))
        finally:
            self.running = False

    # ------------------------------------------------------------------
    def _fail(self, reason: str, detail: str):
        self._set_state(S_ERROR, f'{reason}: {detail}')
        self.get_logger().error(f'Pipeline FAILED: {reason} - {detail}')

    def _wait_for_pose(self, topic: str, timeout: float) -> PoseStamped:
        """Subscribe to a PoseStamped topic and return the first message.

        The node is already being spun by the main executor, so we just
        create a subscription and poll for the stored message.
        """
        result = {}

        def cb(msg):
            result['pose'] = msg

        # 用 TRANSIENT_LOCAL 订阅才能拿到 latched 的历史位姿
        # (publisher 也是 TRANSIENT_LOCAL; rclpy 默认 VOLATILE 不触发重放)
        latch_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.create_subscription(PoseStamped, topic, cb, latch_qos)
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(0.1)
            if 'pose' in result:
                break
        self.destroy_subscription(sub)
        return result.get('pose')


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
