"""IMU 误差测试节点

测试内容:
  1. rotate   —— 原地旋转: 对比 IMU yaw 角 / yaw rate vs 轮速里程计 yaw / yaw rate
  2. forward  —— 前进:     对比 IMU 姿态漂移 (roll/pitch) vs 线速度一致性
  3. backward —— 后退:     同上
  4. all      —— 依次执行上述三项 (默认)

节点既可自己发 /cmd_vel 控制小车运动, 也可只被动采样 (由用户用键盘/遥控器控制).
"""

import math
import time
import statistics
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# ---------- 工具函数 ----------

def quat_to_euler(x: float, y: float, z: float, w: float):
    """四元数 → 欧拉角 (roll, pitch, yaw), 单位弧度."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def wrap_to_pi(angle: float) -> float:
    """把角度归一化到 [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ---------- 数据容器 ----------

@dataclass
class ImuSample:
    stamp_sec: float           # 时间戳 (秒)
    roll: float
    pitch: float
    yaw: float                 # 从 orientation 提取
    yaw_rate: float            # angular_velocity.z
    accel_x: float
    accel_y: float
    accel_z: float


@dataclass
class OdomSample:
    stamp_sec: float
    x: float
    y: float
    yaw: float
    vx: float                  # 线速度 x
    vy: float
    yaw_rate: float            # angular velocity z


@dataclass
class SegmentStats:
    """一段动作的误差统计."""
    label: str
    count: int = 0

    # === 旋转测试指标 ===
    imu_yaw_start: float = 0.0
    imu_yaw_end: float = 0.0
    imu_yaw_delta: float = 0.0          # IMU 角度变化
    odom_yaw_delta: float = 0.0         # 里程计角度变化
    yaw_delta_error: float = 0.0        # 角度误差 (imu - odom)

    imu_yaw_rate_mean: float = 0.0
    imu_yaw_rate_std: float = 0.0
    imu_yaw_rate_max: float = 0.0
    odom_yaw_rate_mean: float = 0.0
    imu_vs_odom_rate_bias: float = 0.0  # 均值差

    # === 直线测试指标 ===
    imu_roll_mean: float = 0.0
    imu_roll_std: float = 0.0
    imu_roll_range: float = 0.0         # max - min
    imu_pitch_mean: float = 0.0
    imu_pitch_std: float = 0.0
    imu_pitch_range: float = 0.0

    imu_yaw_drift: float = 0.0          # 直线段 IMU yaw 漂移 (首尾差)
    odom_yaw_drift: float = 0.0         # 里程计 yaw 漂移
    vx_mean: float = 0.0                # 实际线速度均值


class ImuErrorTestNode(Node):

    def __init__(self):
        super().__init__('imu_error_test_node')

        # ---------- 参数 ----------
        self.declare_parameter('imu_topic', '/mobile_base/sensors/imu_data')
        self.declare_parameter('odom_topic', '/odom')               # wheeltec_robot 原始轮速里程计
        self.declare_parameter('odom_fused_topic', '/odom_combined') # EKF 融合里程计
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_cmd_vel', True)             # True=节点自己发速度, False=只采样
        self.declare_parameter('test_mode', 'all')                  # rotate / forward / backward / all
        self.declare_parameter('duration', 30.0)                    # 超时兜底秒数 (运动目标优先)
        self.declare_parameter('target_distance', 1.0)              # 前进/后退目标距离 (m)
        self.declare_parameter('target_rotation_deg', 90.0)         # 原地旋转目标角度 (°)
        self.declare_parameter('angular_speed', 0.5)                # rad/s, 原地旋转 —— 降速减少惯性
        self.declare_parameter('linear_speed', 0.15)                # m/s, 前进/后退 —— 降速减少惯性
        self.declare_parameter('sample_rate', 50.0)                 # Hz, 采样频率
        self.declare_parameter('wait_seconds_before_start', 3.0)    # 启动后等待底盘稳定

        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_fused_topic = self.get_parameter('odom_fused_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.publish_cmd_vel = self.get_parameter('publish_cmd_vel').value
        self.test_mode = self.get_parameter('test_mode').value
        self.duration = self.get_parameter('duration').value
        self.target_distance = self.get_parameter('target_distance').value
        self.target_rotation_deg = self.get_parameter('target_rotation_deg').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.wait_seconds_before_start = self.get_parameter('wait_seconds_before_start').value

        # ---------- 订阅 ----------
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos_best_effort)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos_reliable)
        self.create_subscription(Odometry, self.odom_fused_topic, self._on_odom_fused, qos_reliable)

        # ---------- 发布 ----------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ---------- 状态 ----------
        self._imu_buffer: List[ImuSample] = []
        self._odom_buffer: List[OdomSample] = []
        self._odom_fused_buffer: List[OdomSample] = []
        self._segment_imu: List[ImuSample] = []
        self._segment_odom: List[OdomSample] = []
        self._segment_odom_fused: List[OdomSample] = []

        self._test_active = False
        self._current_label = ''
        self._current_cmd = Twist()
        self._tests_scheduled: List[str] = []
        self._test_index = 0
        self._segment_start_ts = 0.0
        self._segment_start_odom: Optional[OdomSample] = None   # 本段起点 odom, 用于距离/角度驱动
        self._all_results: List[SegmentStats] = []
        self._heartbeat_count = 0

        self.get_logger().info(
            f'=== IMU Error Test ===\n'
            f'  IMU topic        : {self.imu_topic}\n'
            f'  Odom topic       : {self.odom_topic}\n'
            f'  Odom fused topic : {self.odom_fused_topic}\n'
            f'  cmd_vel topic    : {self.cmd_vel_topic}\n'
            f'  publish_cmd_vel  : {self.publish_cmd_vel}\n'
            f'  test_mode        : {self.test_mode}\n'
            f'  target_distance  : {self.target_distance} m   (直线)\n'
            f'  target_rotation  : {self.target_rotation_deg} °  (旋转)\n'
            f'  duration         : {self.duration} s   (超时兜底)\n'
            f'  angular_speed    : {self.angular_speed} rad/s\n'
            f'  linear_speed     : {self.linear_speed} m/s\n'
            f'========================='
        )

        # 定时采样 + 状态机
        self._sample_timer = self.create_timer(1.0 / self.sample_rate, self._sample_tick)
        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat_tick)

        # 延迟启动测试, 让底盘先准备好
        self._start_timer = self.create_timer(
            self.wait_seconds_before_start,
            self._schedule_tests_once,
        )

    # ---------- 回调 ----------

    def _on_imu(self, msg: Imu):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        r, p, y = quat_to_euler(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w,
        )
        self._imu_buffer.append(ImuSample(
            stamp_sec=t_sec,
            roll=r, pitch=p, yaw=y,
            yaw_rate=msg.angular_velocity.z,
            accel_x=msg.linear_acceleration.x,
            accel_y=msg.linear_acceleration.y,
            accel_z=msg.linear_acceleration.z,
        ))
        # 限制 buffer 长度, 避免长时间运行爆内存
        if len(self._imu_buffer) > int(self.sample_rate * 120):
            self._imu_buffer = self._imu_buffer[-int(self.sample_rate * 60):]

    def _on_odom(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = msg.pose.pose
        r, p, y = quat_to_euler(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        twist = msg.twist.twist
        self._odom_buffer.append(OdomSample(
            stamp_sec=t_sec,
            x=pose.position.x,
            y=pose.position.y,
            yaw=y,
            vx=twist.linear.x,
            vy=twist.linear.y,
            yaw_rate=twist.angular.z,
        ))
        if len(self._odom_buffer) > int(self.sample_rate * 120):
            self._odom_buffer = self._odom_buffer[-int(self.sample_rate * 60):]

    def _on_odom_fused(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = msg.pose.pose
        r, p, y = quat_to_euler(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        twist = msg.twist.twist
        self._odom_fused_buffer.append(OdomSample(
            stamp_sec=t_sec,
            x=pose.position.x,
            y=pose.position.y,
            yaw=y,
            vx=twist.linear.x,
            vy=twist.linear.y,
            yaw_rate=twist.angular.z,
        ))
        if len(self._odom_fused_buffer) > int(self.sample_rate * 120):
            self._odom_fused_buffer = self._odom_fused_buffer[-int(self.sample_rate * 60):]

    def _heartbeat_tick(self):
        self._heartbeat_count += 1
        if self._test_active:
            elapsed = self._time_now() - self._segment_start_ts
            if self._current_label in ('forward', 'backward'):
                cur = self._odom_fused_buffer[-1] if self._odom_fused_buffer else (
                    self._odom_buffer[-1] if self._odom_buffer else None
                )
                start = self._segment_start_odom
                if cur and start:
                    dist = math.hypot(cur.x - start.x, cur.y - start.y)
                    self.get_logger().info(
                        f'[{self._current_label}] running ... '
                        f'{dist:.2f}/{self.target_distance:.2f} m  ({elapsed:.1f}s)'
                    )
                else:
                    self.get_logger().info(
                        f'[{self._current_label}] running ... (waiting odom)  ({elapsed:.1f}s)'
                    )
            elif self._current_label == 'rotate':
                cur = self._odom_fused_buffer[-1] if self._odom_fused_buffer else (
                    self._odom_buffer[-1] if self._odom_buffer else None
                )
                start = self._segment_start_odom
                if cur and start:
                    ddeg = abs(math.degrees(wrap_to_pi(cur.yaw - start.yaw)))
                    self.get_logger().info(
                        f'[{self._current_label}] running ... '
                        f'{ddeg:.1f}/{self.target_rotation_deg:.1f} °  ({elapsed:.1f}s)'
                    )
                else:
                    self.get_logger().info(
                        f'[{self._current_label}] running ... (waiting odom)  ({elapsed:.1f}s)'
                    )
        elif self._tests_scheduled:
            self.get_logger().info(
                f'  idle. next = {self._tests_scheduled[self._test_index] if self._test_index < len(self._tests_scheduled) else "DONE"}'
            )
        elif self._heartbeat_count == 1:
            self.get_logger().info(
                f'Waiting sensors... imu={len(self._imu_buffer)} odom={len(self._odom_buffer)}. '
                f'Test will auto start in {self.wait_seconds_before_start}s. '
                f'(publish_cmd_vel={self.publish_cmd_vel})'
            )

    # ---------- 状态机 ----------

    def _schedule_tests_once(self):
        """只能触发一次, 避免 timer 重复调用."""
        self._start_timer.cancel()
        mode = self.test_mode.strip().lower()
        if mode == 'rotate':
            self._tests_scheduled = ['rotate']
        elif mode == 'forward':
            self._tests_scheduled = ['forward']
        elif mode == 'backward':
            self._tests_scheduled = ['backward']
        elif mode == 'all':
            self._tests_scheduled = ['forward', 'backward', 'rotate']
        else:
            self.get_logger().error(f'Unknown test_mode={self.test_mode}, expected rotate/forward/backward/all')
            return

        # 至少要有 IMU 数据才开始 (cmd_vel 不依赖里程计, 但距离/角度停止条件需要)
        if len(self._imu_buffer) == 0:
            self.get_logger().warn(f'No IMU data on {self.imu_topic} yet. Waiting 5s more...')
            self._start_timer = self.create_timer(5.0, self._schedule_tests_once)
            return

        if self.publish_cmd_vel and len(self._odom_buffer) == 0 and len(self._odom_fused_buffer) == 0:
            self.get_logger().warn(
                f'No odom data yet (odom={len(self._odom_buffer)}, '
                f'odom_fused={len(self._odom_fused_buffer)}). '
                f'Test will still start — auto-stop will rely on duration timeout, '
                f'and distance/angle stats will be skipped.'
            )

        self._test_index = 0
        self._all_results = []
        self._run_next_test()

    def _run_next_test(self):
        # 防重入: 上一轮还没 finish 就 return
        if self._test_active:
            return
        if self._test_index >= len(self._tests_scheduled):
            self._print_summary()
            return
        label = self._tests_scheduled[self._test_index]
        self._current_label = label

        # 清空 segment buffer
        self._segment_imu.clear()
        self._segment_odom.clear()
        self._segment_odom_fused.clear()

        # 构造 cmd
        self._current_cmd = Twist()
        if label == 'rotate':
            self._current_cmd.angular.z = self.angular_speed
        elif label == 'forward':
            self._current_cmd.linear.x = self.linear_speed
        elif label == 'backward':
            self._current_cmd.linear.x = -self.linear_speed

        # 段起点 odom: 每次 START 取当前位置, 独立量本段运动距离
        # forward 从位置A出发, backward 从位置B出发, 各自量自己走了多少 —— 不叠加
        self._segment_start_odom = (
            self._odom_fused_buffer[-1] if self._odom_fused_buffer else
            (self._odom_buffer[-1] if self._odom_buffer else None)
        )
        if self._segment_start_odom is None:
            self._segment_start_odom = OdomSample(
                stamp_sec=self._time_now(),
                x=0.0, y=0.0, yaw=0.0,
                vx=0.0, vy=0.0, yaw_rate=0.0,
            )
            self.get_logger().info('  [anchor] no odom yet, using (0,0,0)')
        else:
            self.get_logger().info(
                f'  [anchor] x={self._segment_start_odom.x:.3f} '
                f'y={self._segment_start_odom.y:.3f} '
                f'yaw={math.degrees(self._segment_start_odom.yaw):.1f}°'
            )

        self._segment_start_ts = self._time_now()
        self._test_active = True

        # 显示目标
        if label in ('forward', 'backward'):
            target_str = f'dist≥{self.target_distance:.2f} m'
        else:
            target_str = f'rot≥{self.target_rotation_deg:.1f}°'
        self.get_logger().info(
            f'>>> START [{label}] target={target_str}  timeout={self.duration:.0f}s  '
            f'cmd(vx={self._current_cmd.linear.x:.2f}, wz={self._current_cmd.angular.z:.2f})'
        )

    def _sample_tick(self):
        """采样定时器: 持续把最新数据滚入 segment buffer, 并在需要时发 cmd."""
        if self._test_active and self.publish_cmd_vel:
            self.cmd_pub.publish(self._current_cmd)

        # 把最新一条拷进 segment buffer
        if self._imu_buffer:
            self._segment_imu.append(self._imu_buffer[-1])
        if self._odom_buffer:
            self._segment_odom.append(self._odom_buffer[-1])
        if self._odom_fused_buffer:
            self._segment_odom_fused.append(self._odom_fused_buffer[-1])

        if not self._test_active:
            return

        # 停止条件 1: 超时兜底
        if (self._time_now() - self._segment_start_ts) >= self.duration:
            self.get_logger().info(f'   timeout {self.duration:.0f}s reached, stopping.')
            self._finish_current_test()
            return

        # 停止条件 2/3: 距离 / 角度 达标 (publish_cmd_vel=True 才检查自动停止)
        if not self.publish_cmd_vel:
            return

        cur = (
            self._odom_fused_buffer[-1] if self._odom_fused_buffer else
            (self._odom_buffer[-1] if self._odom_buffer else None)
        )
        start = self._segment_start_odom
        if cur is None or start is None:
            return

        if self._current_label in ('forward', 'backward'):
            # 沿车头朝向投影计算前进距离 (全局 dx 会有投影损失)
            dx = cur.x - start.x
            dy = cur.y - start.y
            cos_y = math.cos(start.yaw)
            sin_y = math.sin(start.yaw)
            projection = dx * cos_y + dy * sin_y   # 位移向量在车头朝向 (start.yaw) 上的投影

            if self._current_label == 'forward':
                signed_dist = projection
            else:
                signed_dist = -projection

            dist = abs(signed_dist)

            # 渐进减速: 剩余 < 0.15m 时 vx 按比例线性降到 0, 减少惯性 overshoot
            BRAKE_BAND = 0.15
            remaining = self.target_distance - dist
            if remaining > 0 and remaining < BRAKE_BAND and self.publish_cmd_vel:
                k = max(0.05, remaining / BRAKE_BAND)
                sign = 1.0 if self._current_label == 'forward' else -1.0
                self._current_cmd.linear.x = sign * self.linear_speed * k
                self.cmd_pub.publish(self._current_cmd)

            if dist >= self.target_distance:
                dir_note = '方向OK' if signed_dist > 0 else '注意: 方向偏了!'
                self.get_logger().info(
                    f'   target reached: moved {dist:.2f}/{self.target_distance:.2f} m '
                    f'(dx={dx:+.3f}, dy={dy:+.3f}, yaw={math.degrees(start.yaw):.1f}°) [{dir_note}]'
                )
                self._finish_current_test()
        elif self._current_label == 'rotate':
            ddeg = abs(math.degrees(wrap_to_pi(cur.yaw - start.yaw)))
            # 旋转同样渐进减速: 剩余 < 10° 时降速
            ROT_BRAKE_DEG = 10.0
            remaining_deg = self.target_rotation_deg - ddeg
            if remaining_deg > 0 and remaining_deg < ROT_BRAKE_DEG and self.publish_cmd_vel:
                k = max(0.1, remaining_deg / ROT_BRAKE_DEG)
                self._current_cmd.angular.z = math.copysign(self.angular_speed * k, self._current_cmd.angular.z)
                self.cmd_pub.publish(self._current_cmd)

            if ddeg >= self.target_rotation_deg:
                self.get_logger().info(
                    f'   target reached: rotated {ddeg:.1f}/{self.target_rotation_deg:.1f} °'
                )
                self._finish_current_test()

    def _finish_current_test(self):
        self._test_active = False
        # 发停止指令
        if self.publish_cmd_vel:
            self.cmd_pub.publish(Twist())
        stats = self._compute_stats(self._current_label)
        self._print_stats(stats)
        self._all_results.append(stats)

        self._test_index += 1
        # 间隔 2 秒再跑下一个, 让小车停下来
        if self._test_index < len(self._tests_scheduled):
            self.get_logger().info(f'   resting 2s before next test...')
            self._start_timer = self.create_timer(2.0, self._run_next_test)
        else:
            # 全部结束: 取消可能还在跑的旧 timer, 打印一次总评
            try:
                self._start_timer.cancel()
            except Exception:
                pass
            self._print_summary()

    # ---------- 统计 ----------

    def _compute_stats(self, label: str) -> SegmentStats:
        s = SegmentStats(label=label, count=len(self._segment_imu))

        imu = self._segment_imu
        odom = self._segment_odom

        if len(imu) < 5:
            self.get_logger().warn(f'[{label}] too few imu samples ({len(imu)}), stats may be unreliable')
            return s

        s.imu_yaw_start = imu[0].yaw
        s.imu_yaw_end = imu[-1].yaw
        s.imu_yaw_delta = wrap_to_pi(s.imu_yaw_end - s.imu_yaw_start)

        imu_yaw_rates = [x.yaw_rate for x in imu]
        s.imu_yaw_rate_mean = statistics.fmean(imu_yaw_rates)
        s.imu_yaw_rate_std = statistics.pstdev(imu_yaw_rates) if len(imu_yaw_rates) > 1 else 0.0
        s.imu_yaw_rate_max = max(abs(x) for x in imu_yaw_rates)

        rolls = [x.roll for x in imu]
        pitches = [x.pitch for x in imu]
        yaws = [x.yaw for x in imu]
        s.imu_roll_mean = statistics.fmean(rolls)
        s.imu_roll_std = statistics.pstdev(rolls) if len(rolls) > 1 else 0.0
        s.imu_roll_range = max(rolls) - min(rolls)
        s.imu_pitch_mean = statistics.fmean(pitches)
        s.imu_pitch_std = statistics.pstdev(pitches) if len(pitches) > 1 else 0.0
        s.imu_pitch_range = max(pitches) - min(pitches)

        # 直线段 yaw 漂移 = 首尾 yaw 差 (wrap 到 pi)
        s.imu_yaw_drift = wrap_to_pi(yaws[-1] - yaws[0])

        if odom:
            # 里程计角度变化 (首尾 yaw 差)
            s.odom_yaw_delta = wrap_to_pi(odom[-1].yaw - odom[0].yaw)
            s.yaw_delta_error = wrap_to_pi(s.imu_yaw_delta - s.odom_yaw_delta)

            odom_yaw_rates = [x.yaw_rate for x in odom]
            s.odom_yaw_rate_mean = statistics.fmean(odom_yaw_rates)
            s.imu_vs_odom_rate_bias = s.imu_yaw_rate_mean - s.odom_yaw_rate_mean

            s.odom_yaw_drift = wrap_to_pi(odom[-1].yaw - odom[0].yaw)
            vxs = [x.vx for x in odom]
            s.vx_mean = statistics.fmean(vxs)

        return s

    def _print_stats(self, s: SegmentStats):
        log = self.get_logger()
        log.info(f'')
        log.info(f'====== RESULT [{s.label}] ======')
        log.info(f'  samples imu={len(self._segment_imu)} odom={len(self._segment_odom)}')

        if s.label == 'rotate':
            log.info(f'  --- 旋转误差 ---')
            log.info(f'  IMU 角位移 : {math.degrees(s.imu_yaw_delta):8.2f} °')
            log.info(f'  Odom 角位移: {math.degrees(s.odom_yaw_delta):8.2f} °')
            log.info(f'  角度误差   : {math.degrees(s.yaw_delta_error):8.2f} °   (imu - odom)')
            log.info(f'  IMU yaw rate : mean={s.imu_yaw_rate_mean:+.4f} rad/s  '
                     f'std={s.imu_yaw_rate_std:.4f}  max|rate|={s.imu_yaw_rate_max:.4f}')
            log.info(f'  Odom yaw rate: mean={s.odom_yaw_rate_mean:+.4f} rad/s')
            log.info(f'  速率偏置    : {s.imu_vs_odom_rate_bias:+.4f} rad/s   '
                     f'({math.degrees(s.imu_vs_odom_rate_bias):+.2f} °/s)')
        else:
            log.info(f'  --- 直线误差 ---')
            log.info(f'  实际 vx 均值 : {s.vx_mean:+.3f} m/s')
            log.info(f'  IMU yaw 漂移 : {math.degrees(s.imu_yaw_drift):+.2f} °')
            log.info(f'  Odom yaw 漂移: {math.degrees(s.odom_yaw_drift):+.2f} °')
            log.info(f'  Roll  均值±std: {math.degrees(s.imu_roll_mean):+.2f}° ± {math.degrees(s.imu_roll_std):.2f}°'
                     f'   range={math.degrees(s.imu_roll_range):.2f}°')
            log.info(f'  Pitch 均值±std: {math.degrees(s.imu_pitch_mean):+.2f}° ± {math.degrees(s.imu_pitch_std):.2f}°'
                     f'   range={math.degrees(s.imu_pitch_range):.2f}°')
        log.info(f'=============================')

    def _print_summary(self):
        log = self.get_logger()
        log.info(f'')
        log.info(f'=========== ALL TESTS DONE ===========')
        for s in self._all_results:
            if s.label == 'rotate':
                log.info(f'  [rotate] 角度误差 = {math.degrees(s.yaw_delta_error):+.2f} °, '
                         f'速率偏置 = {s.imu_vs_odom_rate_bias:+.4f} rad/s')
            else:
                log.info(f'  [{s.label}] yaw漂移 = {math.degrees(s.imu_yaw_drift):+.2f} °, '
                         f'roll_range = {math.degrees(s.imu_roll_range):.2f} °')
        log.info(f'======================================')

    # ---------- 辅助 ----------

    def _time_now(self) -> float:
        now = self.get_clock().now()
        return now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ImuErrorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received, stopping...')
    finally:
        # 持续发零速 ~1 秒, 确保底盘收到足够多帧停车指令
        # (单次 publish 可能丢包, 或底盘控制频率不够)
        if node.publish_cmd_vel:
            node.get_logger().info('Sending stop commands for 1s...')
            import time as _t
            t_stop = _t.time() + 1.0
            zero = Twist()
            while _t.time() < t_stop:
                try:
                    node.cmd_pub.publish(zero)
                except Exception:
                    pass
                _t.sleep(0.05)
            node.get_logger().info('Stop sent.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
