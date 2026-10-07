#!/usr/bin/env python3
"""MyCobot280 流式执行驱动节点 (架构修复版 v2).

在 mycobot_driver_node.py 基础上重写, 保留防抖特性,
彻底修复"只能控制一次""单指令也卡顿""关闭报错"三类问题.

============================================================
卡顿根因分析 (v1 → v2 的核心改动)
============================================================

根因1: EMA 平滑与 refresh_mode 不兼容 (卡顿主因)
  refresh_mode=1 下, 每次 send_angles 让固件立即向目标移动.
  EMA 每步只走剩余距离的 33% (a=1/smooth=1/3), 一步约 0.3°.
  固件在 speed=80 (~120°/s) 下 ~2.5ms 就到了这个中间点,
  然后停住等下一条指令 (间隔 20-25ms).
  → 走→停→走→停 = 卡顿!
  EMA 是为 fresh_mode=0 (队列模式) 设计的, 在刷新模式中
  固件自己做插值, EMA 不仅多余而且有害.
  → 修复: 移除 EMA, 直接发送目标角度.

根因2: 串口争抢 (卡顿次因)
  publish_state 20Hz 每次持锁做 2 次串口往返 (~30-40ms),
  阻塞 _stream_loop 发送, 造成流式断流.
  → 修复: 状态读取降为 10Hz + 非阻塞 trylock,
    串口繁忙时跳过状态读取, 永不阻塞执行线程.

根因3: 关闭报错 "publisher's context is invalid"
  stop() 未取消 timer、未 join 执行线程, destroy_node 后
  publish_state 仍触发, publisher context 已失效.
  → 修复: stop() 先取消 timer, 再 join 线程, 最后 mc.stop().

============================================================
保留的防抖特性
============================================================
  1. set_fresh_mode(1): 刷新模式, 最新指令覆盖固件队列,
     不再"减速→停→再加速". 固件自行平滑插值, 无需上位机 EMA.
  2. send_angles (非阻塞): 把最新目标灌给固件, 绝不等到位.
  3. 关节空间执行: ros2_control/MoveIt 已算好关节角, 直接下发.
  4. 后台执行线程: 解耦 ROS 回调与串口 I/O.
  5. 回调分组: 指令订阅与状态定时器互不阻塞.
  6. 变化检测: 目标与上次下发差值 <0.1° 时跳过, 减少空闲串口流量.

============================================================
接口
============================================================
接口保持 Float64MultiArray, 与旧 C++ bridge / ros2_control 兼容:

  ros2 run mycobot280_moveit2_control mycobot_driver_update_node \
      --ros-args -p port:=/dev/ttyACM0 -p baudrate:=115200
"""
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from packaging import version
import pymycobot

# min low version require (set_fresh_mode 是 v3.7.0+ 才有的接口)
MIN_REQUIRE_VERSION = '3.7.0'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        'The version of pymycobot library must be greater than {} or higher. '
        'The current version is {}. Please upgrade the library version.'.format(
            MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280

# URDF 中夹爪关节的弧度范围, 用于 0..100 与 radians 互转
GRIPPER_RAD_MIN = -0.7
GRIPPER_RAD_MAX = 0.15

# 变化阈值(deg): 目标与上次下发差值小于此值时跳过, 减少空闲串口流量
SEND_EPS = 0.1


class MyCobotDriverUpdate(Node):
    """订阅关节指令 + 夹爪指令, 流式下发到 MyCobot280, 发布当前状态."""

    def __init__(self):
        super().__init__('mycobot_driver_update')

        # ---- 参数 ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('speed', 80)        # 固件内部插值速度 1-100
        self.declare_parameter('stream_hz', 40.0)  # 流式下发频率 (≤50, 串口吞吐上限)
        self.declare_parameter('gripper_speed', 30)  # 夹爪开合速度 1-100

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.get_logger().info(
            f"Connecting to MyCobot at {port} with baudrate {baudrate}")

        # ---- ① 硬件: 刷新模式 (核心防抖动开关) ----
        try:
            self.mc = MyCobot280(port, baudrate)
            time.sleep(1)  # 等连接稳定

            # fresh_mode=1: 刷新模式, 最新指令覆盖固件队列, 避免分段堆叠卡顿
            if self.mc.get_fresh_mode() != 1:
                self.mc.set_fresh_mode(1)
                time.sleep(1)
            if self.mc.get_fresh_mode() != 1:
                self.get_logger().warn(
                    '刷新模式未生效: 请升级 pymycobot 与 Atom/Transponder 固件')

            self.get_logger().info("MyCobot connected successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MyCobot: {e}")
            raise

        # ---- 串口锁: 所有硬件 I/O 必须经过此锁, 杜绝并发访问 ----
        self.serial_lock = threading.Lock()

        # ---- 共享状态 (回调与执行线程之间) ----
        self.lock = threading.Lock()   # 保护 target / cur / gripper_target
        self._stop = False
        self.target = None             # 最新期望关节角度(deg), 回调写入
        self.cur = None                # 最近一次下发的角度(deg), 执行线程维护
        self.gripper_target = None     # 0..100
        self.gripper_last_sent = None  # 上次已下发的夹爪值, 避免重复 send

        # 初始化 cur 为机械臂实际角度
        try:
            with self.serial_lock:
                angles = self.mc.get_angles()
            if isinstance(angles, (list, tuple)) and len(angles) == 6:
                self.cur = [float(a) for a in angles]
                self.get_logger().info(f"Initial arm angles(deg): {self.cur}")
        except Exception as e:
            self.get_logger().warn(f"Could not read initial angles: {e}")

        # ---- 回调分组 (关键修复) ----
        # 指令订阅用 ReentrantCallbackGroup: 可随时执行, 不被状态定时器阻塞
        cmd_cb_group = ReentrantCallbackGroup()
        # 状态定时器用独立 MutuallyExclusiveCallbackGroup: 与指令回调互不干扰
        state_cb_group = MutuallyExclusiveCallbackGroup()

        # ---- 订阅 (与旧 driver 完全兼容的 Float64MultiArray 接口) ----
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/mycobot/cmd_joint_pos',
            self.cmd_callback,
            5,
            callback_group=cmd_cb_group)
        self.gripper_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/mycobot/cmd_gripper_pos',
            self.gripper_cmd_callback,
            5,
            callback_group=cmd_cb_group)

        # ---- 发布当前状态 ----
        self.state_pub = self.create_publisher(
            Float64MultiArray,
            '/mycobot/current_joint_pos',
            5)
        self.gripper_state_pub = self.create_publisher(
            Float64MultiArray,
            '/mycobot/current_gripper_pos',
            5)

        # 状态读取定时器 10Hz. 用非阻塞 trylock, 串口繁忙时跳过,
        # 永不阻塞执行线程. 20Hz 会与 40Hz 流式发送争抢串口导致卡顿.
        self.timer = self.create_timer(
            0.1, self.publish_state, callback_group=state_cb_group)

        # ---- ④ 启动流式执行线程 ----
        self._stream_thread = threading.Thread(
            target=self._stream_loop, daemon=True)
        self._stream_thread.start()

        self.get_logger().info("MyCobot Driver Update Node is ready.")

    # ============================================================
    # ROS 回调: 只更新共享目标, 不做硬件 I/O, 不阻塞
    # ============================================================
    def cmd_callback(self, msg):
        """Receive joint angle commands (radians) from ros2_control.

        弧度→角度转换, 直接写入 self.target (最新目标覆盖).
        契合 fresh_mode 语义: 固件始终执行最新指令.
        """
        if len(msg.data) != 6:
            self.get_logger().warn(
                f"Expected 6 joints, got {len(msg.data)}")
            return

        # pymycobot 用角度制(deg), ros2_control 用弧度(rad)
        angles_deg = [math.degrees(a) for a in msg.data]
        with self.lock:
            self.target = angles_deg

    def gripper_cmd_callback(self, msg):
        """Receive gripper command (radians) and convert to 0..100."""
        if not msg.data:
            self.get_logger().warn("Gripper command received empty data, skip")
            return

        radians = msg.data[0]

        if not (GRIPPER_RAD_MIN <= radians <= GRIPPER_RAD_MAX):
            self.get_logger().warn(
                f"Gripper rad {radians} out of range "
                f"[{GRIPPER_RAD_MIN}, {GRIPPER_RAD_MAX}], clamping")

        # radians=-0.7 -> 0 (closed), radians=0.15 -> 100 (fully open)
        value = int(round(
            (radians - GRIPPER_RAD_MIN) /
            (GRIPPER_RAD_MAX - GRIPPER_RAD_MIN) * 100))
        value = max(0, min(100, value))
        with self.lock:
            self.gripper_target = value

    # ============================================================
    # 辅助: 判断两组角度是否足够接近
    # ============================================================
    def _close(self, a, b, eps=SEND_EPS):
        return all(abs(x - y) < eps for x, y in zip(a, b))

    # ============================================================
    # ④ 执行线程: 目标变化时下发, refresh_mode 下固件自行插值
    #
    # refresh_mode=1 的正确用法: 直接发送最终目标, 固件从当前位置
    # 平滑插值到目标. 不要发 EMA 中间点 — 固件会到中间点后停住,
    # 等下一条指令, 造成走→停→走→停的卡顿.
    # ============================================================
    def _stream_loop(self):
        period = 1.0 / max(1.0, self.get_parameter('stream_hz').value)
        speed = self.get_parameter('speed').value
        gripper_speed = self.get_parameter('gripper_speed').value

        while not self._stop:
            # --- 读取最新目标 (不持锁做 I/O) ---
            with self.lock:
                tgt = self.target
                grip_tgt = self.gripper_target

            need_send = False
            if tgt is not None:
                # 只在目标变化时下发, 避免重复 send 浪费串口带宽
                need_send = self.cur is None or not self._close(self.cur, tgt)

            if need_send:
                try:
                    with self.serial_lock:
                        self.mc.send_angles(tgt, speed)
                    self.cur = list(tgt)
                except Exception as e:
                    self.get_logger().error(
                        f"Error sending angles(deg): {e}")

            # --- 夹爪: 只在目标变化时下发, 避免重复 send 堆积 ---
            if grip_tgt is not None and grip_tgt != self.gripper_last_sent:
                try:
                    with self.serial_lock:
                        self.mc.set_gripper_value(grip_tgt, gripper_speed)
                    self.gripper_last_sent = grip_tgt
                except Exception as e:
                    self.get_logger().error(
                        f"Error sending gripper value: {e}")

            time.sleep(period)

    # ============================================================
    # 状态发布: 非阻塞 trylock, 串口繁忙时跳过, 永不阻塞执行线程
    # ============================================================
    def publish_state(self):
        """Read current arm angles and gripper value, publish as state."""
        if self._stop:
            return

        # 非阻塞获取串口锁: 执行线程正在用串口时直接跳过本次状态读取
        if not self.serial_lock.acquire(blocking=False):
            return
        try:
            angles_deg = self.mc.get_angles()
            gripper_value = self.mc.get_gripper_value()
        except Exception as e:
            try:
                self.get_logger().warn(f"Error reading state: {e}")
            except Exception:
                pass
            return
        finally:
            self.serial_lock.release()

        # 发布关节状态
        if isinstance(angles_deg, (list, tuple)) and len(angles_deg) == 6:
            angles_rad = [math.radians(a) for a in angles_deg]
            msg = Float64MultiArray()
            msg.data = angles_rad
            try:
                self.state_pub.publish(msg)
            except Exception:
                pass  # 关闭期间 context 可能已失效

        # 发布夹爪状态
        if isinstance(gripper_value, (int, float)) and 0 <= gripper_value <= 100:
            gripper_radians = gripper_value / 100.0 * \
                (GRIPPER_RAD_MAX - GRIPPER_RAD_MIN) + GRIPPER_RAD_MIN
            msg = Float64MultiArray()
            msg.data = [gripper_radians]
            try:
                self.gripper_state_pub.publish(msg)
            except Exception:
                pass  # 关闭期间 context 可能已失效

    # ============================================================
    # 清理: 取消 timer → join 线程 → 停臂
    # ============================================================
    def stop(self):
        self._stop = True

        # 先取消定时器, 防止 publish_state 在销毁后触发
        try:
            if self.timer is not None:
                self.timer.cancel()
        except Exception:
            pass

        # 等待执行线程退出 (最多 2 秒)
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=2.0)

        # 停止机械臂运动
        try:
            with self.serial_lock:
                self.mc.stop()
        except Exception:
            pass

        try:
            self.get_logger().info("MyCobot Driver Update Node stopped.")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MyCobotDriverUpdate()

    # MultiThreadedExecutor: 配合回调分组, 让指令回调与状态定时器并行
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
