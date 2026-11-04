"""里程计原地旋转漂移测试节点

测试内容:
  让小车原地自转 N 圈, 同时记录两路里程计位姿:
    /odom          —— 轮速里程计 (底盘原始输出)
    /odom_combined —— EKF 融合里程计
  理想情况下原地旋转的轨迹应收缩在一点; 若轮径/轮距标定不准,
  角速度与线速度换算不匹配, 轨迹会画出一个圆并逐圈漂移.

输出指标 (每路里程计各一份):
  1. 起点→终点位移           —— 漂移量
  2. 轨迹相对起点的最大半径   —— 画圈幅度
  3. 轨迹相对质心的 RMS 半径  —— 轨迹离散程度
  4. 累计旋转角 vs 目标角     —— 角度误差
  5. 每圈折算漂移             —— 便于横向比较

节点可自己发 /cmd_vel 控制小车, 也可只被动采样 (publish_cmd_vel:=false).
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

# ---------- 工具函数 ----------


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元数 → 偏航角 (弧度), 只取绕 z 轴的旋转."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    """把角度归一化到 [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ---------- 数据容器 ----------


@dataclass
class OdomSample:
    """一帧里程计位姿采样."""
    stamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass
class SpinReport:
    """一段旋转测试的漂移统计."""
    source: str                     # 数据来源标签, 如 odom / odom_combined
    frame_id: str = ''
    count: int = 0

    yaw_accum: float = 0.0          # 累计旋转角 (解缠绕)
    target_yaw: float = 0.0         # 目标旋转角
    yaw_error: float = 0.0          # 累计角 - 目标角

    dx: float = 0.0                 # 起点→终点位移分量
    dy: float = 0.0
    displacement: float = 0.0       # 起点→终点位移模长

    max_radius: float = 0.0         # 相对起点的最大半径
    rms_radius: float = 0.0         # 相对质心的 RMS 半径
    drift_per_rev: float = 0.0      # 每圈折算漂移

    path_points: List[OdomSample] = field(default_factory=list)


# ---------- 统计计算 ----------


def build_report(source: str, samples: List[OdomSample], frame_id: str,
                 target_yaw: float, revolutions: float) -> SpinReport:
    """根据采样序列计算漂移指标 (纯函数, 便于复用)."""
    report = SpinReport(source=source, frame_id=frame_id, count=len(samples),
                        target_yaw=target_yaw)
    report.path_points = samples
    if len(samples) < 2:
        return report

    report.yaw_accum = accumulate_yaw([s.yaw for s in samples])
    report.yaw_error = wrap_to_pi(report.yaw_accum - target_yaw)

    start, end = samples[0], samples[-1]
    report.dx = end.x - start.x
    report.dy = end.y - start.y
    report.displacement = math.hypot(report.dx, report.dy)

    report.max_radius = max(math.hypot(s.x - start.x, s.y - start.y)
                            for s in samples)

    cx = sum(s.x for s in samples) / len(samples)
    cy = sum(s.y for s in samples) / len(samples)
    report.rms_radius = math.sqrt(
        sum((s.x - cx) ** 2 + (s.y - cy) ** 2 for s in samples) / len(samples))

    if revolutions > 0:
        report.drift_per_rev = report.displacement / revolutions

    return report


def accumulate_yaw(yaws: List[float]) -> float:
    """把逐帧 yaw (会跳变) 解缠绕后累加, 得到总旋转角."""
    total = 0.0
    for i in range(1, len(yaws)):
        total += wrap_to_pi(yaws[i] - yaws[i - 1])
    return total


def drift_verdict(displacement: float) -> str:
    """根据位移给出结论."""
    if displacement < 0.05:
        return '定点旋转, 无明显漂移'
    if displacement < 0.15:
        return '轻微画圈漂移'
    return '明显画圈漂移, 需检查轮径/轮距标定'


# ---------- 节点 ----------


class OdomSpinTestNode(Node):

    def __init__(self):
        super().__init__('odom_spin_test_node')

        # ---------- 参数 ----------
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_fused_topic', '/odom_combined')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_cmd_vel', True)
        self.declare_parameter('control_source', 'fused')   # fused / raw
        self.declare_parameter('angular_speed', 0.4)        # rad/s
        self.declare_parameter('revolutions', 2.0)          # 原地旋转圈数
        self.declare_parameter('duration', 45.0)            # 超时兜底秒数
        self.declare_parameter('sample_rate', 50.0)         # Hz
        self.declare_parameter('wait_seconds_before_start', 3.0)
        self.declare_parameter('path_topic', '/odom_spin_test/path')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_fused_topic = self.get_parameter('odom_fused_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.publish_cmd_vel = self.get_parameter('publish_cmd_vel').value
        self.control_source = self.get_parameter('control_source').value.strip().lower()
        self.angular_speed = self.get_parameter('angular_speed').value
        self.revolutions = self.get_parameter('revolutions').value
        self.duration = self.get_parameter('duration').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.wait_seconds_before_start = self.get_parameter('wait_seconds_before_start').value
        self.path_topic = self.get_parameter('path_topic').value

        self.target_yaw = 2.0 * math.pi * self.revolutions

        # ---------- 订阅 / 发布 ----------
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self._sub_raw = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom_raw, qos_reliable)
        self._sub_fused = self.create_subscription(
            Odometry, self.odom_fused_topic, self._on_odom_fused, qos_reliable)
        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._path_pub = self.create_publisher(Path, self.path_topic, 10)

        # ---------- 状态 ----------
        self._raw_latest: Optional[OdomSample] = None
        self._fused_latest: Optional[OdomSample] = None
        self._raw_frame_id = ''
        self._fused_frame_id = ''

        self._seg_raw: List[OdomSample] = []
        self._seg_fused: List[OdomSample] = []
        self._accum_raw = 0.0
        self._accum_fused = 0.0
        self._last_yaw_raw: Optional[float] = None
        self._last_yaw_fused: Optional[float] = None

        self._test_active = False
        self._cmd = Twist()
        self._start_ts = 0.0

        self.get_logger().info(
            f'=== Odom Spin Test (组合整车) ===\n'
            f'  odom raw   : {self.odom_topic}\n'
            f'  odom fused : {self.odom_fused_topic}\n'
            f'  cmd_vel    : {self.cmd_vel_topic}\n'
            f'  publish    : {self.publish_cmd_vel}   控制源: {self.control_source}\n'
            f'  target     : {self.revolutions:.2f} 圈 ({math.degrees(self.target_yaw):.1f}°)\n'
            f'  angular    : {self.angular_speed} rad/s   超时: {self.duration:.0f}s\n'
            f'================================='
        )

        self.create_timer(1.0 / self.sample_rate, self._sample_tick)
        self.create_timer(1.0, self._heartbeat_tick)
        self._start_timer = self.create_timer(
            self.wait_seconds_before_start, self._start_test_once)

    # ---------- 回调 ----------

    def _on_odom_raw(self, msg: Odometry):
        self._raw_latest = self._to_sample(msg)
        self._raw_frame_id = msg.header.frame_id
        self._accum_raw, self._last_yaw_raw = self._accumulate(
            self._accum_raw, self._last_yaw_raw, self._raw_latest.yaw)

    def _on_odom_fused(self, msg: Odometry):
        self._fused_latest = self._to_sample(msg)
        self._fused_frame_id = msg.header.frame_id
        self._accum_fused, self._last_yaw_fused = self._accumulate(
            self._accum_fused, self._last_yaw_fused, self._fused_latest.yaw)

    @staticmethod
    def _to_sample(msg: Odometry) -> OdomSample:
        pose = msg.pose.pose
        return OdomSample(
            stamp_sec=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            x=pose.position.x,
            y=pose.position.y,
            yaw=quat_to_yaw(pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w),
        )

    @staticmethod
    def _accumulate(accum: float, last_yaw: Optional[float],
                    yaw: float) -> tuple:
        """累加解缠绕后的角度增量, 返回 (新累计角, 新 last_yaw)."""
        if last_yaw is None:
            return accum, yaw
        return accum + wrap_to_pi(yaw - last_yaw), yaw

    # ---------- 状态机 ----------

    def _start_test_once(self):
        """延迟启动一次, 避免 timer 反复触发."""
        self._start_timer.cancel()
        if self._latest_control_sample() is None:
            self.get_logger().warn(
                '尚无里程计数据, 5s 后重试; 若持续无数据请检查底盘驱动/EKF')
            self._start_timer = self.create_timer(5.0, self._start_test_once)
            return

        self._seg_raw.clear()
        self._seg_fused.clear()
        self._accum_raw = 0.0
        self._accum_fused = 0.0
        self._last_yaw_raw = None
        self._last_yaw_fused = None

        self._cmd = Twist()
        self._cmd.angular.z = self.angular_speed
        self._start_ts = self._time_now()
        self._test_active = True

        self.get_logger().info(
            f'>>> START 原地旋转 {self.revolutions:.1f} 圈, '
            f'wz={self.angular_speed:.2f} rad/s, 超时 {self.duration:.0f}s')

    def _sample_tick(self):
        """定时采样: 发速度指令, 记录位姿, 判断是否结束."""
        if self._test_active and self.publish_cmd_vel:
            self._cmd_pub.publish(self._cmd)

        self._record_segment()

        if not self._test_active:
            return

        if (self._time_now() - self._start_ts) >= self.duration:
            self.get_logger().info(f'   超时 {self.duration:.0f}s, 停止旋转')
            self._finish_test()
            return

        if not self.publish_cmd_vel:
            return

        accum = self._control_accum()
        remaining = self.target_yaw - abs(accum)

        # 临近目标时线性降速, 减小惯性 overshoot
        if 0 < remaining < math.radians(10.0):
            k = max(0.1, remaining / math.radians(10.0))
            self._cmd.angular.z = math.copysign(self.angular_speed * k,
                                                self.angular_speed)
            self._cmd_pub.publish(self._cmd)

        if abs(accum) >= self.target_yaw:
            self.get_logger().info(
                f'   目标达成: 累计旋转 {math.degrees(accum):.1f}°')
            self._finish_test()

    def _record_segment(self):
        """把最新一帧位姿滚入本段采样缓冲."""
        if self._raw_latest:
            self._seg_raw.append(self._raw_latest)
        if self._fused_latest:
            self._seg_fused.append(self._fused_latest)

    def _finish_test(self):
        self._test_active = False
        if self.publish_cmd_vel:
            self._cmd_pub.publish(Twist())

        raw_report = build_report('odom(轮速)', self._seg_raw, self._raw_frame_id,
                                  self.target_yaw, self.revolutions)
        fused_report = build_report('odom_combined(EKF)', self._seg_fused,
                                    self._fused_frame_id, self.target_yaw,
                                    self.revolutions)
        self._print_report(raw_report)
        self._print_report(fused_report)
        self._publish_path(fused_report)

    def _heartbeat_tick(self):
        if not self._test_active:
            return
        elapsed = self._time_now() - self._start_ts
        accum = self._control_accum()
        self.get_logger().info(
            f'   running ... 累计 {math.degrees(accum):.1f}/'
            f'{math.degrees(self.target_yaw):.1f}°, 已用 {elapsed:.1f}s')

    # ---------- 输出 ----------

    def _print_report(self, r: SpinReport):
        log = self.get_logger()
        log.info('')
        log.info(f'====== 漂移结果 [{r.source}] ======')
        if r.count < 2:
            log.info('  采样不足, 无法统计 (检查里程计是否发布)')
            log.info('==================================')
            return
        log.info(f'  采样点数     : {r.count}')
        log.info(f'  累计旋转角   : {math.degrees(r.yaw_accum):8.2f} °')
        log.info(f'  目标旋转角   : {math.degrees(r.target_yaw):8.2f} °')
        log.info(f'  角度误差     : {math.degrees(r.yaw_error):8.2f} °')
        log.info(f'  起点→终点位移: {r.displacement:.4f} m '
                 f'(dx={r.dx:+.4f}, dy={r.dy:+.4f})')
        log.info(f'  最大半径     : {r.max_radius:.4f} m   (相对起点)')
        log.info(f'  RMS 半径     : {r.rms_radius:.4f} m   (相对质心)')
        log.info(f'  每圈漂移     : {r.drift_per_rev:.4f} m/圈')
        log.info(f'  结论         : {drift_verdict(r.displacement)}')
        log.info('==================================')

    def _publish_path(self, r: SpinReport):
        """把轨迹发布成 nav_msgs/Path, 便于在 RViz 中查看."""
        if not r.path_points:
            return
        path = Path()
        path.header.frame_id = r.frame_id or 'odom_combined'
        path.header.stamp = self.get_clock().now().to_msg()
        for s in r.path_points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = s.x
            ps.pose.position.y = s.y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)
        self.get_logger().info(f'  轨迹已发布到 {self.path_topic} ({len(path.poses)} 点)')

    # ---------- 辅助 ----------

    def _latest_control_sample(self) -> Optional[OdomSample]:
        if self.control_source == 'raw':
            return self._raw_latest
        return self._fused_latest if self._fused_latest else self._raw_latest

    def _control_accum(self) -> float:
        if self.control_source == 'raw':
            return self._accum_raw
        if self._fused_latest:
            return self._accum_fused
        return self._accum_raw

    def _time_now(self) -> float:
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        return sec + nsec * 1e-9


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = OdomSpinTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('收到退出信号, 停止旋转')
    finally:
        # 中途退出也输出一次统计, 避免手动采样模式下拿不到结果
        if node._test_active:
            node._finish_test()
        # 持续发零速约 1s, 确保底盘确实收到停车指令
        if node.publish_cmd_vel:
            import time as _t
            t_stop = _t.time() + 1.0
            zero = Twist()
            while _t.time() < t_stop:
                try:
                    node._cmd_pub.publish(zero)
                except Exception:
                    pass
                _t.sleep(0.05)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
