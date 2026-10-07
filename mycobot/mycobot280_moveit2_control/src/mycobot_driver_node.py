#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray, Empty
import pymycobot
import time
import math
import threading
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.7.0'

# 机械臂关节命令去重阈值 (deg)。
# C++ 硬件桥以 50Hz 持续发布命令, 臂到位后仍不断发相同最终点。
# - 阈值必须小于运动中相邻插值点的最小变化量: 最慢段 (vel_scale 0.1,
#   关节约 3°/s) 每帧约 0.06°, 故取 0.02° 不会误跳过运动指令;
# - 到位后插值器输出恒定 (逐帧差为 0), 相同命令被跳过, 不再反复
#   sync_send_angles 抢占串口, 夹爪指令才能及时下发 (否则夹爪张合
#   会被推迟到下一段臂运动结束后才物理执行)。
ARM_SEND_EPS_DEG = 0.02

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280


class MyCobotDriver(Node):
    def __init__(self):
        super().__init__('mycobot_driver')
        
        # 参数声明
        self.declare_parameter('port', '/dev/mycobot_controller')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        self.get_logger().info(f"Connecting to MyCobot at {port} with baudrate {baudrate}")
        
        try:
            # 初始化 pymycobot
            self.mc = MyCobot280(port, baudrate)
            # 稍微等待一下确保连接稳定
            time.sleep(1)

            # 所有串口访问共用一把锁, 避免指令/状态读写在同一串口上交错。
            self.serial_lock = threading.Lock()
            # 夹爪去重: C++ 桥接以 50Hz 持续发布当前夹爪指令, 只在开合度
            # 真正变化时才下发一次 set_gripper_value, 避免串口被重复指令
            # 淹没导致开合动作不可靠。
            self._last_gripper_sent = None
            # 机械臂关节命令去重: 记录上次实际下发的角度 (deg)
            self._last_arm_sent_deg = None
            # 机械臂异步下发: 与夹爪同理, 回调只记录最新目标, 由独立工作
            # 线程阻塞下发。绝不能在 executor 回调线程里调阻塞式
            # sync_send_angles(timeout=10): 臂运动时多个插值点会把
            # MultiThreadedExecutor 的全部线程堵在串口锁上, 夹爪订阅/状态
            # 定时器长期得不到线程, 夹爪张合被延迟十几秒。
            self._arm_target_deg = None
            # 最近一次从固件读到的实际关节角 (deg), 急停时用来让阻塞中的
            # sync_send_angles 立刻判"到位"退出, 释放 serial_lock。
            self._latest_angles_deg = None
            self._arm_wake = threading.Event()
            self._arm_stop = False
            self._arm_worker = threading.Thread(
                target=self._arm_stream_loop, daemon=True)
            self._arm_worker.start()
            # 夹爪异步下发: 回调线程只记录最新目标, 由独立工作线程在串口
            # 空闲时下发。C++ 以 50Hz 发夹爪指令, 直接在回调里同步下发会
            # 在串口繁忙时产生大量积压, 陈旧消息 (如旧的"张开 100") 十几
            # 秒后才物理执行, 把已经闭合的夹爪重新打开。
            self._gripper_target = None
            self._gripper_target_since = None  # 当前目标首次出现的时间
            self._gripper_wake = threading.Event()
            self._gripper_stop = False
            self._gripper_worker = threading.Thread(
                target=self._gripper_stream_loop, daemon=True)
            self._gripper_worker.start()

            # fresh_mode=0：固件完整执行每段运动，不被新命令中断
            # 配合 sync_send_angles 阻塞调用：
            # - 每段完整执行，路径不丢失，能到达目标
            # - 段间有轻微停顿（分段感），但不抖动
            with self.serial_lock:
                if self.mc.get_fresh_mode() == 1:
                    self.mc.set_fresh_mode(0)
                    time.sleep(1)

            self.get_logger().info("MyCobot connected successfully.")

        except Exception as e:
            self.get_logger().error(f"Failed to connect to MyCobot: {e}")
            raise

        # 指令订阅用可重入回调组: 臂指令/夹爪指令可并发被调度, 只在
        # 访问串口时由 serial_lock 互斥。否则默认互斥组会让夹爪指令
        # 排在臂运动指令长队 + 20Hz 状态读定时器后面, 可能延迟十几秒
        # 才物理执行 (表现为"下降后才张开、提升前未闭合")。
        self.cmd_cb_group = ReentrantCallbackGroup()

        # 订阅来自 ros2_control (C++ bridge) 的 6 个 arm 关节指令 (弧度)
        # queue_depth=1: 只保留最新一条命令，旧的直接丢弃，避免段运动队列堆积
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/mycobot/cmd_joint_pos',
            self.cmd_callback,
            1,
            callback_group=self.cmd_cb_group
        )

        # 订阅夹爪开合指令 (0~100 整数, pymycobot 原生单位)
        self.gripper_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/mycobot/cmd_gripper_pos',
            self.gripper_cmd_callback,
            1,
            callback_group=self.cmd_cb_group
        )

        # 机械臂急停: 收到任意 Empty 立即停止固件运动并丢弃待发臂目标。
        # (语音"机械臂停止"经 grasp_node 的 /pick_place/arm_stop 服务转发)
        self.stop_sub = self.create_subscription(
            Empty,
            '/mycobot/stop',
            self.stop_cmd_callback,
            1,
            callback_group=self.cmd_cb_group
        )

        # 发布当前 arm 关节状态给 ros2_control (C++ bridge)
        self.state_pub = self.create_publisher(
            Float64MultiArray,
            '/mycobot/current_joint_pos',
            5
        )

        # 发布夹爪当前状态 (0~100 整数)
        self.gripper_state_pub = self.create_publisher(
            Float64MultiArray,
            '/mycobot/current_gripper_pos',
            5
        )

        # 定时器定期读取状态并发布 (20Hz). 50Hz 会让 M5 串口来不及响应,
        # 触发 "device reports readiness to read but returned no data".
        self.timer = self.create_timer(0.05, self.publish_state)

        self.get_logger().info("MyCobot Driver Node is ready.")

    def cmd_callback(self, msg):
        """接收 6 关节指令 (弧度) — 只更新最新目标, 立即返回, 不碰串口。

        阻塞式 sync_send_angles 放到独立工作线程 _arm_stream_loop, 避免
        占用 executor 线程导致夹爪/状态回调被饿死。
        """
        if len(msg.data) != 6:
            self.get_logger().warn(f"Expected 6 joints, got {len(msg.data)}")
            return
        # pymycobot 用角度制, ros2_control 用弧度
        self._arm_target_deg = [math.degrees(a) for a in msg.data]
        self._arm_wake.set()

    def stop_cmd_callback(self, msg):
        """机械臂急停: 固件立即停止运动 + 丢弃待发/在途目标。

        - self.mc.stop() 故意不持 serial_lock: arm worker 可能正阻塞在
          sync_send_angles() 里等旧目标到位 (最多 10s), 若先抢锁则 stop
          指令要等到超时后才能发出去, 物理臂无法立即刹停;
        - 与此同时, worker 阻塞调用传入的 target 与 self._arm_target_deg
          是同一个 list, 把它原地改成当前实际角后, sync_send_angles 内部
          0.1s 一次的 is_in_position(旧target) 立刻判"到位"返回, serial_lock
          迅速释放, 夹爪/状态读写不会被堵满 10s;
        - 清空目标与去重占位后, ros2_control 桥 50Hz 持续发来的"保持当前
          位置"命令 (JTC 取消后 hold 在停止点) 只会让固件再收到一次原地
          目标, 不产生可见运动。
        """
        self.get_logger().info('Arm STOP command received -> firmware stop')
        cur = self._latest_angles_deg
        tgt = self._arm_target_deg
        if tgt is not None and isinstance(cur, (list, tuple)) and len(cur) == 6:
            try:
                tgt[:] = [float(a) for a in cur]
            except Exception:
                pass
        self._arm_target_deg = None
        self._last_arm_sent_deg = None
        self._arm_wake.set()
        try:
            self.mc.stop()
        except Exception as e:
            self.get_logger().error(f"Error stopping arm: {e}")

    def _arm_stream_loop(self):
        """工作线程: 目标变化且去重通过时, 阻塞下发最新臂关节角。

        - 只跟踪最新目标 (与 _last_arm_sent_deg 逐关节比 < EPS 则跳过);
        - 与夹爪工作线程共用 serial_lock, 臂静止时不持锁, 夹爪可即时下发;
        - sync_send_angles(fresh_mode=0) 阻塞等本段运动完成再返回。
        """
        while not self._arm_stop:
            self._arm_wake.wait(timeout=0.2)
            self._arm_wake.clear()
            target = self._arm_target_deg
            if target is None:
                continue
            if self._last_arm_sent_deg is not None and all(
                    abs(a - b) < ARM_SEND_EPS_DEG
                    for a, b in zip(target, self._last_arm_sent_deg)):
                continue
            try:
                with self.serial_lock:
                    # 等锁期间可能有更新的目标 (或已被急停清空), 重新取一次
                    target = self._arm_target_deg
                    if target is None:
                        # 急停已丢弃目标, 等 ros2_control 桥下一条保持命令
                        continue
                    if self._last_arm_sent_deg is not None and all(
                            abs(a - b) < ARM_SEND_EPS_DEG
                            for a, b in zip(target,
                                            self._last_arm_sent_deg)):
                        continue
                    self._last_arm_sent_deg = target
                    self.mc.sync_send_angles(target, 80, timeout=10.0)
            except Exception as e:
                # 失败清占位, 允许下一拍重试
                self._last_arm_sent_deg = None
                self.get_logger().error(f"Error sending angles: {e}")
                time.sleep(0.1)

    def gripper_cmd_callback(self, msg):
        """Receive gripper position command (radians) from ros2_control.

        ros2_control / MoveIt use the URDF joint limit range [-0.7, 0.15]
        radians for gripper_controller. pymycobot's set_gripper_value uses
        0..100 as the open/close range (0=closed, 100=fully open). Convert
        radians -> 0..100 before sending to the arm.
        """
        if not msg.data:
            return
        try:
            radians = msg.data[0]
            # radians=-0.7 -> 0 (closed), radians=0.15 -> 100 (fully open)
            value = int(round((radians - (-0.7)) / (0.15 - (-0.7)) * 100))
            value = max(0, min(100, value))
            # 诊断: 节流打印原始夹爪指令 (排查 C++ 桥是否下发)
            # 已按用户要求静默 (50Hz 下刷屏)。需要排查时恢复下面两行即可。
            # self.get_logger().info(
            #     f"raw gripper cmd rad={radians:.3f} -> value {value}",
            #     throttle_duration_sec=0.5)
            # 只更新最新目标, 不碰串口: 工作线程负责合并 50Hz 重复/抖动
            # 流并下发最新值, 从根本上消除陈旧夹爪命令延迟执行。
            # 目标更新绝不能放在 serial_lock 内: 臂运动刚结束时串口可能
            # 被状态读短暂占满, 而夹爪 action 停滞超时只有几秒 (stall_timeout),
            # 若回调等锁更新目标, 会错过整个指令窗口 (控制器 abort 后恢复旧值)。
            # 单变量赋值在 GIL 下原子, 无需加锁。
            now = time.monotonic()
            if value != self._gripper_target:
                self._gripper_target = value
                self._gripper_target_since = now
            self._gripper_wake.set()
        except Exception as e:
            self.get_logger().error(f"Error parsing gripper value: {e}")

    # 新夹爪目标必须连续出现这么久才物理下发。正常 50Hz 恒定指令流在
    # 0.15s 后放行; 串口繁忙期积压的陈旧消息会在几十 ms 内混杂执行
    # (旧 100 夹在新 0 中间), 闸门只放行持续稳定的最新值。
    GRIPPER_SETTLE_SEC = 0.15

    def _gripper_stream_loop(self):
        """工作线程: 目标稳定后在串口空闲处下发最新夹爪开合度。

        - 只记录/下发"最新且持续稳定"的目标, 中途的旧值直接丢弃;
        - 与机械臂指令共用 serial_lock, 臂运动时段间空隙即可完成下发;
        - 失败不更新 last_sent, 下一拍重试同一目标。
        """
        while not self._gripper_stop:
            self._gripper_wake.wait(timeout=0.2)
            self._gripper_wake.clear()
            target = self._gripper_target
            since = self._gripper_target_since
            if target is None or target == self._last_gripper_sent:
                continue
            # 稳定性闸门: 滤掉积压队列里短暂闪现的陈旧值
            if since is None or \
                    time.monotonic() - since < self.GRIPPER_SETTLE_SEC:
                continue
            try:
                with self.serial_lock:
                    # 二次确认: 等待锁期间目标可能又变了
                    target = self._gripper_target
                    if target != self._last_gripper_sent:
                        self.mc.set_gripper_value(target, 30)
                        self._last_gripper_sent = target
                self.get_logger().info(f"Gripper command -> value {target}")
            except Exception as e:
                self.get_logger().error(f"Error sending gripper value: {e}")
                time.sleep(0.2)

    def publish_state(self):
        """Read current arm angles and gripper value, publish as state."""
        try:
            # get_angles 在通信异常时会返回 int 错误码 (0/1)，正常返回 6 元素 list
            with self.serial_lock:
                angles_deg = self.mc.get_angles()
            # self.get_logger().info(f"mycobot_driver_node: get_angles() -> {angles_deg}")
            if isinstance(angles_deg, (list, tuple)) and len(angles_deg) == 6:
                # 缓存最新实际角, 供急停回调解套阻塞中的 sync_send_angles
                self._latest_angles_deg = [float(a) for a in angles_deg]
                angles_rad = [math.radians(angle) for angle in angles_deg]
                msg = Float64MultiArray()
                msg.data = angles_rad
                self.state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Error reading arm state: {e}")

        try:
            # get_gripper_value 返回 0..100 int；异常时返回 0 或 None
            with self.serial_lock:
                gripper_value = self.mc.get_gripper_value()
            if isinstance(gripper_value, (int, float)) and 0 <= gripper_value <= 100:
                # Convert 0..100 -> radians [-0.7, 0.15] to match URDF joint
                # limit. 0 (closed) -> -0.7, 100 (fully open) -> 0.15.
                gripper_radians = gripper_value / 100.0 * (0.15 - (-0.7)) + (-0.7)
                msg = Float64MultiArray()
                msg.data = [gripper_radians]
                self.gripper_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Error reading gripper state: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MyCobotDriver()

    executor = MultiThreadedExecutor()
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
