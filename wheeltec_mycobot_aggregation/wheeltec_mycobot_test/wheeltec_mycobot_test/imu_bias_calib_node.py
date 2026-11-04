"""IMU 零偏标定节点

用途:
  车体完全静止时采集一段时间 IMU 数据, 计算三轴陀螺仪零偏与三轴加速度零偏,
  并给出结果在驱动中的可修改位置.

原理:
  静止时陀螺仪真实角速度为 0, 其输出均值即为零偏 (bias).
  加速度计静止时只感受重力, 若车体水平安装, 输出均值应为 (0, 0, +g),
  减去该期望值即为加速度零偏 (仅 assume_flat_mount=true 时按 |g| 扣除).

重要:
  采样期间小车必须完全静止 (不上电行驶、机械臂不动、人不碰车),
  否则均值里混入真实运动, 标定结果无效. 节点会检查静止性并给出警告.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu


# ---------- 基础统计 ----------


def axis_mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def axis_std(values: List[float]) -> float:
    """总体标准差, 用来衡量采样期间是否静止."""
    if len(values) < 2:
        return 0.0
    m = axis_mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / len(values))


# ---------- 数据容器 ----------


@dataclass
class ImuBiasReport:
    """一次零偏标定的结果."""
    count: int = 0
    duration: float = 0.0
    rate_hz: float = 0.0

    gyro_bias: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    gyro_std: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accel_mean: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accel_bias: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    accel_std: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    stationary: bool = True
    reason: str = ''


# ---------- 计算与格式化 (纯函数) ----------


def evaluate_bias(gyro: List[List[float]], accel: List[List[float]],
                  duration: float, gravity: float, flat_mount: bool,
                  max_gyro_std: float, max_accel_std: float) -> ImuBiasReport:
    """从采样序列计算零偏指标."""
    report = ImuBiasReport(count=len(gyro), duration=duration)
    if not gyro or not accel:
        return report

    report.rate_hz = len(gyro) / duration if duration > 0 else 0.0

    for i in range(3):
        g = [s[i] for s in gyro]
        a = [s[i] for s in accel]
        report.gyro_bias[i] = axis_mean(g)
        report.gyro_std[i] = axis_std(g)
        report.accel_mean[i] = axis_mean(a)
        report.accel_std[i] = axis_std(a)

    # 加速度零偏: 水平安装时 z 轴应等于 +g, xy 应为 0
    if flat_mount:
        report.accel_bias = [
            report.accel_mean[0],
            report.accel_mean[1],
            report.accel_mean[2] - gravity,
        ]
    else:
        # 安装姿态未知时无法把重力与零偏分离, 只能报告原始均值
        report.accel_bias = list(report.accel_mean)

    # 静止性判定
    reasons = []
    if max(report.gyro_std) > max_gyro_std:
        reasons.append(f'陀螺仪噪声/扰动过大 (max std={max(report.gyro_std):.4f} rad/s)')
    if max(report.accel_std) > max_accel_std:
        reasons.append(f'加速度噪声/扰动过大 (max std={max(report.accel_std):.3f} m/s²)')
    if reasons:
        report.stationary = False
        report.reason = '; '.join(reasons)

    return report


def format_report(r: ImuBiasReport, flat_mount: bool) -> List[str]:
    """把结果格式化成日志行."""
    deg = 180.0 / math.pi
    lines = [
        '',
        '====== IMU 零偏标定结果 ======',
        f'  采样     : {r.count} 帧 / {r.duration:.1f} s  ({r.rate_hz:.1f} Hz)',
        '',
        '  --- 陀螺仪 (静止时输出均值即零偏) ---',
        f'  零偏 rad/s : x={r.gyro_bias[0]:+.6f}  y={r.gyro_bias[1]:+.6f}  z={r.gyro_bias[2]:+.6f}',
        f'  零偏 °/s   : x={r.gyro_bias[0] * deg:+.4f}  '
        f'y={r.gyro_bias[1] * deg:+.4f}  z={r.gyro_bias[2] * deg:+.4f}',
        f'  噪声 std   : x={r.gyro_std[0]:.6f}  y={r.gyro_std[1]:.6f}  z={r.gyro_std[2]:.6f} rad/s',
        '',
        '  --- 加速度计 ---',
        f'  均值 m/s²  : x={r.accel_mean[0]:+.4f}  y={r.accel_mean[1]:+.4f}  '
        f'z={r.accel_mean[2]:+.4f}',
        f'  噪声 std   : x={r.accel_std[0]:.4f}  y={r.accel_std[1]:.4f}  z={r.accel_std[2]:.4f}',
    ]
    if flat_mount:
        lines.append(
            f'  零偏 m/s²  : x={r.accel_bias[0]:+.4f}  y={r.accel_bias[1]:+.4f}  '
            f'z={r.accel_bias[2]:+.4f}   (已按 |g| 扣除重力)')
    else:
        lines.append('  零偏 m/s²  : 未扣重力, 上表均值即含重力分量 (assume_flat_mount=false)')

    lines.append('')
    if r.stationary:
        lines.append('  静止性检查 : OK, 采样期间车体静止, 结果可用')
    else:
        lines.append(f'  静止性检查 : 不合格 —— {r.reason}')
        lines.append('               采样期间车体有运动, 请保持完全静止后重测')
    lines.append('')
    lines.append('  --- 结果应用位置 ---')
    lines.append('  1) 驱动解析: turn_on_wheeltec_robot/src/wheeltec_robot.cpp')
    lines.append('     Mpu6050_Data 原始值 → Mpu6050.angular_velocity / linear_acceleration')
    lines.append('  2) 比例常数: turn_on_wheeltec_robot/include/turn_on_wheeltec_robot/')
    lines.append('     wheeltec_robot.h  (GYROSCOPE_RATIO / ACCEl_RATIO)')
    lines.append('  3) 注意: 当前驱动和 yaml 均无零偏修正参数;')
    lines.append('     launch/base_serial.launch.py 只配置串口与坐标系,')
    lines.append('     ekf.yaml 的 imu0_differential 只做帧间差分, 不消除绝对零偏.')
    lines.append('     需要按零偏修正时, 应在驱动代码中减去 bias, 或在上位机/固件标定.')
    lines.append('==================================')
    return lines


# ---------- 节点 ----------


class ImuBiasCalibNode(Node):

    def __init__(self):
        super().__init__('imu_bias_calib_node')

        self.declare_parameter('imu_topic', '/mobile_base/sensors/imu_data')
        self.declare_parameter('duration', 20.0)          # 采样时长 (s)
        self.declare_parameter('min_samples', 200)        # 最少有效样本
        self.declare_parameter('assume_flat_mount', True) # 车体水平安装, z 轴扣重力
        self.declare_parameter('gravity', 9.80665)
        self.declare_parameter('max_gyro_std', 0.02)      # 静止判定: rad/s
        self.declare_parameter('max_accel_std', 0.10)     # 静止判定: m/s²
        self.declare_parameter('auto_exit', True)         # 打印结果后自动退出

        self.imu_topic = self.get_parameter('imu_topic').value
        self.duration = self.get_parameter('duration').value
        self.min_samples = self.get_parameter('min_samples').value
        self.assume_flat_mount = self.get_parameter('assume_flat_mount').value
        self.gravity = self.get_parameter('gravity').value
        self.max_gyro_std = self.get_parameter('max_gyro_std').value
        self.max_accel_std = self.get_parameter('max_accel_std').value
        self.auto_exit = self.get_parameter('auto_exit').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self._sub = self.create_subscription(Imu, self.imu_topic, self._on_imu, qos)

        self._gyro: List[List[float]] = []
        self._accel: List[List[float]] = []
        self._start_ts: Optional[float] = None
        self._done = False
        self._next_progress = 0.0

        self.get_logger().info(
            f'=== IMU Bias Calibration ===\n'
            f'  话题     : {self.imu_topic}\n'
            f'  采样时长 : {self.duration:.1f} s\n'
            f'  水平安装 : {self.assume_flat_mount}   (z 轴按 {self.gravity:.4f} m/s² 扣重力)\n'
            f'  静止阈值 : gyro std < {self.max_gyro_std} rad/s, '
            f'accel std < {self.max_accel_std} m/s²\n'
            f'  请保持小车完全静止, 不要触碰车体和机械臂\n'
            f'===========================')

        self._tick_timer = self.create_timer(0.1, self._tick)

    # ---------- 回调 ----------

    def _on_imu(self, msg: Imu):
        if self._done:
            return
        if self._start_ts is None:
            self._start_ts = self._time_now()
            self._next_progress = self._start_ts + 5.0
        self._gyro.append([msg.angular_velocity.x,
                           msg.angular_velocity.y,
                           msg.angular_velocity.z])
        self._accel.append([msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z])

    def _tick(self):
        if self._done or self._start_ts is None:
            return
        elapsed = self._time_now() - self._start_ts

        if elapsed >= self._next_progress:
            self._next_progress += 5.0
            self.get_logger().info(
                f'  采集中 ... {elapsed:.0f}/{self.duration:.0f}s, {len(self._gyro)} 帧')

        if elapsed >= self.duration:
            self._finish()

    # ---------- 结果 ----------

    def _finish(self):
        self._done = True
        elapsed = self._time_now() - self._start_ts

        if len(self._gyro) < self.min_samples:
            self.get_logger().warn(
                f'有效样本过少 ({len(self._gyro)} < {self.min_samples}), '
                f'请检查 IMU 话题 {self.imu_topic} 是否正常发布')
            self._shutdown_after_report()
            return

        report = evaluate_bias(
            self._gyro, self._accel, elapsed, self.gravity,
            self.assume_flat_mount, self.max_gyro_std, self.max_accel_std)

        for line in format_report(report, self.assume_flat_mount):
            self.get_logger().info(line)

        self._shutdown_after_report()

    def _shutdown_after_report(self):
        if self.auto_exit:
            self.get_logger().info('标定结束, 节点退出')
            rclpy.shutdown()

    # ---------- 辅助 ----------

    def _time_now(self) -> float:
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        return sec + nsec * 1e-9


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = ImuBiasCalibNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if not node._done and node._gyro:
            node.get_logger().info('收到退出信号, 用已采集数据输出结果')
            node._finish()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
