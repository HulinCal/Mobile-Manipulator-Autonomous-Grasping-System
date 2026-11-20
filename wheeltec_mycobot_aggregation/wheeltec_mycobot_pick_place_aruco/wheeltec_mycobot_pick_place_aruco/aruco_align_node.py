#!/usr/bin/env python3
"""机械臂初始化到 pick_ready 后, 用 J1~J5 让法兰 Z 轴对准 ArUco(id=1).

流程:
  1. 启动后下发绝对关节角轨迹到 pick_ready
  2. 订阅 /aruco_single/pose, 由手眼静态 TF 转到 link1 系；
  3. marker 在 link1 系的水平距离 > approach_start_distance(默认 0.20m) 时,
     发 /cmd_vel 驱动底盘接近: 前进与航向修正同时进行 (边走边调),
     运行中每 0.2s(5Hz) 用最新 marker 位姿重算方位角并修正小车方向,
     用户途中移动 marker 也能持续跟随; 距离进入
     approach_stop_distance(0.18m) 即停车, 残余航向偏差由停车后机械臂
     重新对准兜底, 清空旧对准状态后再对准一次再抓取；
     若 marker 距离 < retreat_trigger_distance(默认 0.15m) 太近,
     先不看方向直线后退(负 vx, wz=0) 到 retreat_stop_distance(0.18m),
     停稳后再重新对准抓取；
  4. 用 controller_state 的真实关节角做 URDF 正解，把 marker 位置换算到法兰系 P；
  5. 使用URDF模型的参数用牛顿迭代直接求出使 r 收敛的 J1~J5 绝对角，r就是跟踪的对象在法兰系下到法兰Z轴的距离；
  6. 对准后 marker 连续 grasp_stable_time 秒不动, 进入抓取循环:
     张开夹爪(grasp_open_hold) -> 夹持点移动到 marker (姿态允许小幅
     倾斜, 不再限定沿法兰 +Z) -> 闭合夹爪 -> 回到 pick_ready
     -> 张开夹爪 -> 重新对准
"""
import math
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.action import FollowJointTrajectory, GripperCommand
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

# 单条轨迹的插值点数: 用 smoothstep 让起止速度/加速度都为 0.
# 最后几个点位移极小, 固件流式跟随时会在目标前自然减速,
# 避免"冲到位再急停"(单点轨迹是匀速走完然后瞬间停住).
ALIGN_WAYPOINTS = 20

# marker 速度估计: 相机固定在 link1 上, P 的变化率就是 marker 的真实移动速度.
# 单帧位移超过 JUMP_CLAMP 视为检测跳变(不参与测速), 速度上限 VEL_CLAMP.
JUMP_CLAMP = 0.15   # [m]
VEL_CLAMP = 1.0     # [m/s]
VEL_EMA = 0.3       # 速度 EMA 系数


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


def _trapezoidal(t, blend=0.1):
    """梯形速度 profile: 匀加速(0→blend) + 匀速(blend→1-blend) + 匀减速(1-blend→1).

    位置 s(t) 是分段二次函数, 起停两端一阶/二阶导均为 0, 中间大部分时间匀速.
    峰值速度 = 1 / (1 - 2*blend), 默认 blend=0.1 时峰值约 1.25×均速.
    """
    # 裁剪浮点边界
    t = max(0.0, min(1.0, t))
    if t <= blend:
        # 匀加速段: s = (t/blend)^2 * (blend/2)
        return 0.5 * t * t / blend
    if t >= 1.0 - blend:
        # 匀减速段: 对称于加速段
        u = 1.0 - t
        return 1.0 - 0.5 * u * u / blend
    # 匀速段
    return 0.5 * blend + (t - blend)


def _cruise_decel(t, decel=0.2):
    """直接匀速起步 + 末段匀减速的速度 profile.

    前 (1-decel) 时间全程匀速, 最后 decel 时间从均速匀减速到 0.
    峰值速度 = 2 / (2 - decel); decel=0.2 时峰值 ≈ 1.11×均速, 几乎全程匀速.
    适合给进/插入这类"从静止起步后尽快进入匀速、临近目标再减速"的阶段.
    """
    t = max(0.0, min(1.0, t))
    v_const = 2.0 / (2.0 - decel)
    cut = 1.0 - decel
    if t <= cut:
        return v_const * t
    u = t - cut
    return v_const * (t - 0.5 * u * u / decel)


def _smooth_points(q0, q1, duration, count=ALIGN_WAYPOINTS,
                   profile='trapezoidal'):
    """生成关节插值轨迹点.

    profile:
      'trapezoidal'  —— 起停短暂匀加速/减速, 中间匀速 (blend=0.1,
                       峰值约 1.25×均速), 适合对准/回位等通用阶段.
      'cruise_decel' —— 直接匀速起步, 末段 decel=0.2 匀减速到 0
                       (峰值约 1.11×均速), 适合给进/插入阶段.

    调用方按平均速度估算 duration 即可.
    """
    if profile == 'cruise_decel':
        _s = _cruise_decel
    else:
        _s = _trapezoidal
    pts = []
    for i in range(1, count + 1):
        tau = i / count
        s = _s(tau)
        pt = JointTrajectoryPoint()
        pt.positions = [a + (b - a) * s for a, b in zip(q0, q1)]
        pt.time_from_start = rclpy.duration.Duration(
            seconds=duration * tau).to_msg()
        pts.append(pt)
    return pts


def _rotvec_matrix(r):
    """3x3 旋转矩阵 -> 旋转向量 (轴 * 角), 用于姿态误差."""
    c = 0.5 * (r[0][0] + r[1][1] + r[2][2] - 1.0)
    c = max(-1.0, min(1.0, c))
    ang = math.acos(c)
    if ang < 1e-9:
        return (0.0, 0.0, 0.0)
    s = math.sin(ang)
    if abs(s) < 1e-6:
        return (0.0, 0.0, 0.0)   # 接近 180°, 插入过程不会出现
    k = ang / (2.0 * s)
    return (k * (r[2][1] - r[1][2]),
            k * (r[0][2] - r[2][0]),
            k * (r[1][0] - r[0][1]))


def _solve_linear(a, b):
    """高斯消元解 A x = b (A 为 n x n), 奇异返回 None."""
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-12:
            return None
        m[col], m[piv] = m[piv], m[col]
        inv = 1.0 / m[col][col]
        for j in range(col, n + 1):
            m[col][j] *= inv
        for row in range(n):
            if row == col:
                continue
            f = m[row][col]
            if f:
                for j in range(col, n + 1):
                    m[row][j] -= f * m[col][j]
    return [m[i][n] for i in range(n)]


# 主状态机: 对准 -> 张开 -> 前伸 -> 闭合 -> 回 pick_ready -> 张开 -> 重新对准
ST_ALIGN = 'align'            # 视觉对准 (J1~J5)
ST_OPEN_PRE = 'open_pre'      # 前伸前张开夹爪
ST_INSERT = 'insert'          # 沿法兰 +Z 前伸到 marker
ST_CLOSE = 'close'            # 闭合夹爪
ST_RETURN = 'return'          # 回到 pick_ready
ST_OPEN_POST = 'open_post'    # 回位后张开夹爪


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
        self.declare_parameter('min_move_time', 0.15)    # [s] 轨迹最短时长
        self.declare_parameter('settle_time', 0.15)      # [s] 到位后静置时间
                                                         # (串口反馈滞后约 1s)
        self.declare_parameter('cmd_hold_time', 0.3)     # [s] 同一目标角保持期内不重复下发
        # 跟踪阶段的连续重规划周期 [s]: 单条轨迹时长与之相同,
        # 走到头正好被下一帧目标接管, 两端速度都接近 0, 拼接无突变
        self.declare_parameter('replan_interval', 0.2)
        # 相机外参系统偏差补偿: marker 在 link1 系下的位置修正量 [m],
        # 修正后位置 = 相机解算值 + 该偏移; 用于抵消手眼标定/安装的恒定偏差
        self.declare_parameter('marker_offset_x', 0.0)
        self.declare_parameter('marker_offset_y', 0.0)
        self.declare_parameter('marker_offset_z', 0.0)
        # 目标速度自适应: marker 移动越快, 轨迹速率与固件 speed 越高.
        # 静止时用慢速保证丝滑, 快速移动时自动提速减少滞后.
        self.declare_parameter('speed_adapt', True)
        self.declare_parameter('max_joint_rate_fast', 0.6)  # [rad/s] 上限
        self.declare_parameter('adapt_omega_ref', 0.4)      # [rad/s] 达到满速
        self.declare_parameter('speed_slow', 40)            # 固件 speed 下限
        self.declare_parameter('speed_fast', 80)            # 固件 speed 上限

        # ---- 对准稳定后的抓取循环 ----
        # marker 连续 grasp_stable_time 秒不动才触发抓取.
        # marker 位姿本身有约 ±1.5cm 的 PnP 抖动, 判定不宜过严:
        # 1.0s + 1.5cm 与 aruco_grasp_params.yaml 的 stable_duration /
        # stable_pos_tol 保持一致
        self.declare_parameter('grasp_stable_time', 1.0)    # [s]
        self.declare_parameter('grasp_stable_pos_tol', 0.02)  # [m] 视为不动的范围
        # 夹爪: 法兰 +Z 到实际夹持点的距离 [m].
        # 指尖全长约 0.10m (安装偏移 0.034 + 指节 0.07), 但方块是夹在
        # 两指根部之间: 实机在 0.10 时停下后距 marker 还差约 5cm, 故取 0.05.
        self.declare_parameter('tcp_flange_z', 0.05)
        # 额外前伸深度 [m]: 正数表示越过 marker 继续深入.
        # ArUco 标定板厚 0.03m, 位姿解算给出的是板中心, 实际抓取面在
        # 板前表面, 故再沿接近方向多深入半个板厚 0.015m.
        self.declare_parameter('insert_extra_depth', 0.015)
        self.declare_parameter('insert_rate', 0.3)          # [rad/s] 前伸关节速率
        self.declare_parameter('grasp_open_hold', 1.0)      # [s] 张开后保持
        self.declare_parameter('grasp_close_hold', 1.0)     # [s] 闭合后保持
        self.declare_parameter('gripper_action',
                               '/gripper_action_controller/gripper_cmd')
        self.declare_parameter('gripper_open_value', 0.15)   # [rad]
        self.declare_parameter('gripper_close_value', -0.42)  # [rad]
        self.declare_parameter('gripper_max_effort', 50.0)

        # ---- 底盘接近 ----
        # marker 在 link1 系下的水平距离 (sqrt(x^2+y^2), 不含 z) 超过
        # approach_start_distance 时, 直接发 /cmd_vel 让底盘朝 marker 前进;
        # 进入 approach_stop_distance 后停车, 机械臂再继续对准/抓取.
        # 不含 z: 竖直高度差不占机械臂水平可达范围, 否则地面上的 marker
        # 会被 3D 距离误判为一直"太远", 导致小车持续前顶.
        self.declare_parameter('chassis_approach', True)
        self.declare_parameter('chassis_cmd_topic', '/cmd_vel')
        self.declare_parameter('chassis_frame', 'base_link')
        self.declare_parameter('approach_start_distance', 0.20)  # [m]
        self.declare_parameter('approach_stop_distance', 0.18)   # [m]
        self.declare_parameter('approach_max_linear', 0.05)      # [m/s]
        self.declare_parameter('approach_max_angular', 0.20)     # [rad/s]
        self.declare_parameter('approach_turn_gain', 1.5)        # [rad/s per rad]
        self.declare_parameter('approach_max_travel', 0.5)       # [m]
        # 运行中重新检测 marker 并修正航向的周期: 0.2s = 5Hz 即可
        self.declare_parameter('approach_update_period', 0.2)    # [s]
        # 太近保护: 水平距离小于 retreat_trigger_distance 时直线后退
        # (不调方向, 只发负 vx) 到 retreat_stop_distance, 给机械臂留抓取空间
        self.declare_parameter('retreat_trigger_distance', 0.15)  # [m]
        self.declare_parameter('retreat_stop_distance', 0.18)     # [m]
        # 后退速度放慢: 后退停车延迟会带来过冲, 冲过 approach_start_distance
        # (0.20m) 会立刻又触发前进, 形成前进/后退往复.
        self.declare_parameter('retreat_max_linear', 0.01)        # [m/s]
        self.declare_parameter('retreat_max_travel', 0.5)         # [m]

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
        self.replan_interval = p('replan_interval').value
        self.marker_offset = (p('marker_offset_x').value,
                              p('marker_offset_y').value,
                              p('marker_offset_z').value)
        self.speed_adapt = p('speed_adapt').value
        self.max_joint_rate_fast = p('max_joint_rate_fast').value
        self.adapt_omega_ref = p('adapt_omega_ref').value
        self.speed_slow = p('speed_slow').value
        self.speed_fast = p('speed_fast').value
        self.grasp_stable_time = p('grasp_stable_time').value
        self.grasp_stable_pos_tol = p('grasp_stable_pos_tol').value
        self.tcp_flange_z = p('tcp_flange_z').value
        self.insert_extra_depth = p('insert_extra_depth').value
        self.insert_rate = p('insert_rate').value
        self.grasp_open_hold = p('grasp_open_hold').value
        self.grasp_close_hold = p('grasp_close_hold').value
        self.gripper_open_value = p('gripper_open_value').value
        self.gripper_close_value = p('gripper_close_value').value
        self.gripper_max_effort = p('gripper_max_effort').value
        self.chassis_approach = p('chassis_approach').value
        self.chassis_frame = p('chassis_frame').value
        self.approach_start_distance = p('approach_start_distance').value
        self.approach_stop_distance = p('approach_stop_distance').value
        self.approach_max_linear = p('approach_max_linear').value
        self.approach_max_angular = p('approach_max_angular').value
        self.approach_turn_gain = p('approach_turn_gain').value
        self.approach_max_travel = p('approach_max_travel').value
        self.approach_update_period = p('approach_update_period').value
        self.retreat_trigger_distance = p('retreat_trigger_distance').value
        self.retreat_stop_distance = p('retreat_stop_distance').value
        self.retreat_max_linear = p('retreat_max_linear').value
        self.retreat_max_travel = p('retreat_max_travel').value

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
        self._marker_vel = 0.0       # marker 移动速度 EMA [m/s]
        self._last_marker_pos = None  # link1 系上一帧位置
        self._last_marker_stamp = 0.0
        self._last_dist = 0.3        # 上次的 marker 距离 [m], 用于速度换算
        self._last_send = 0.0        # 上次下发轨迹时刻 (重规划节流)
        self._goal_handle = None     # 在途轨迹句柄, 重规划时取消
        self._goal_seq = 0           # 目标序号
        self._active_seq = 0         # 最新目标序号: 旧目标的结果一律忽略
        # 已下发的指令位置 (不是反馈): 连续重规划以它为基准累加,
        # 否则串口反馈滞后会让每步都从"落后"的位置重新起步
        self._cmd_q = None
        # ---- 抓取循环状态 ----
        self._state = ST_ALIGN          # 主状态机当前状态
        self._state_time = 0.0          # 进入当前状态的时刻
        self._stable_since = None       # 对准后 marker 开始不动的时刻
        self._stable_ref = None         # 不动判定的参考位置 (法兰系 P)
        self._arm_done = None           # 前伸/回位轨迹结果: None/True/False
        self._arm_done_time = 0.0
        self._gripper_done = None       # 夹爪 action 结果: None/True/False
        self._gripper_done_time = 0.0
        self._gripper_busy = False
        # ---- 底盘接近状态 ----
        self._marker_chassis = None       # marker 在底盘系的位姿 (选向用)
        self._approach_active = False     # 正在发速度命令接近 marker
        self._approach_travel = 0.0       # 本次接近累计指令行程 [m]
        self._approach_blocked = False    # 行程用尽, 不再自动前进
        self._approach_last_time = 0.0
        self._approach_cmd_time = 0.0     # 上次重新解算航向的时刻
        self._approach_cmd_vx = 0.0
        self._approach_cmd_wz = 0.0
        self._retreat_active = False      # marker 太近, 正在直线后退
        self._retreat_travel = 0.0        # 本次后退累计指令行程 [m]
        self._retreat_last_time = 0.0
        self._chassis_hold_until = 0.0    # 停车后继续补发零速的截止时刻
        # 定时器重入保护: ReentrantCallbackGroup + 多线程执行器下, 一次
        # IK 计算超过 50ms 周期时会有多个线程同时进入同一 tick, 造成同一帧
        # 重复解算/重复下发 (日志里同一 pos_err 连续打印多次)
        self._tick_lock = threading.Lock()

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
        # ---- 夹爪 action ----
        self.gripper_cli = ActionClient(
            self, GripperCommand,
            p('gripper_action').value,
            callback_group=self.cbg)

        # 固件速度提示: 根据 marker 移动速度实时调节驱动 speed (1-100)
        self._speed_pub = self.create_publisher(
            Int32, '/mycobot/speed_hint', 10)
        # 底盘接近: marker 太远时直接驱动底盘 (chassis_guard 未启动时用
        # /cmd_vel; 若启用了守卫, 应把该参数改为 /cmd_vel_in)
        self._chassis_cmd_topic = p('chassis_cmd_topic').value
        self._chassis_pub = self.create_publisher(
            Twist, self._chassis_cmd_topic, 10)

        # 20Hz 主循环: 初始化 -> 视觉伺服 (跟踪阶段每个 replan_interval 重规划)
        self.create_timer(0.05, self._tick, callback_group=self.cbg)
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

        # marker 在底盘系的位置: 仅用于底盘接近时的朝向选择
        # (走 link1 -> chassis_frame 静态链, 复用上面已补偿的位姿)
        if self.chassis_approach:
            try:
                tf_b = self.tf_buf.lookup_transform(
                    self.chassis_frame, self.base_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2))
                self._marker_chassis = do_transform_pose_stamped(
                    pose_l1, tf_b).pose
            except TransformException:
                self._marker_chassis = None

        # ---- marker 移动速度: 相机固定在 link1 上, 位移差即目标运动 ----
        now = time.monotonic()
        pos = pose_l1.pose.position
        if self._last_marker_pos is not None:
            dt = now - self._last_marker_stamp
            step = math.dist((pos.x, pos.y, pos.z), self._last_marker_pos)
            if 1e-3 < dt < 0.5 and step < JUMP_CLAMP:
                v = min(step / dt, VEL_CLAMP)
                self._marker_vel += VEL_EMA * (v - self._marker_vel)
        self._last_marker_pos = (pos.x, pos.y, pos.z)
        self._last_marker_stamp = now

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

    def _pose_residual(self, target_rot, target_pos, q, w_rot, tcp=0.0):
        """候选关节角下 (法兰或夹持点) 相对目标位姿的 6 维误差.

        tcp > 0 时位置误差按"夹持点 = 法兰 + R*(0,0,tcp)"计算,
        否则按法兰原点计算.
        """
        fk = self._fk(q)
        if fk is None:
            return None
        rot, pos = fk
        if tcp:
            pos = tuple(pos[i] + rot[i][2] * tcp for i in range(3))
        rv = _rotvec_matrix(_mat_mul(target_rot, _mat_t(rot)))
        return [target_pos[0] - pos[0], target_pos[1] - pos[1],
                target_pos[2] - pos[2],
                w_rot * rv[0], w_rot * rv[1], w_rot * rv[2]]

    def _solve_cartesian(self, target_rot, target_pos, w_rot=0.1, tcp=0.0):
        """模型内迭代 J1~J5, 让夹持点到目标位置, 姿态只做弱约束.

        返回 (目标角字典, 位置残差[m], 姿态残差[deg]); 失败返回 None.
        w_rot 越小姿态越自由: 0.02 时允许手腕倾斜约 9° 换取位置精度,
        不再要求沿法兰 +Z 直线前伸 (该方向已到可达边界).
        """
        if any(n not in self._joints for n in ARM_JOINTS):
            return None
        q = dict(self._joints)
        h = 1e-3
        lam2 = 1e-6
        for _ in range(80):
            err = self._pose_residual(target_rot, target_pos, q, w_rot, tcp)
            if err is None:
                return None
            if math.sqrt(sum(v * v for v in err[:3])) <= 1e-3:
                break
            # 6x5 数值雅可比
            jac = []
            for n in ALIGN_JOINTS:
                qp = dict(q)
                qp[n] += h
                ep = self._pose_residual(target_rot, target_pos, qp, w_rot, tcp)
                if ep is None:
                    return None
                jac.append([(ep[k] - err[k]) / h for k in range(6)])
            # 法方程 (J^T J + lam^2 I) dq = -J^T e (e = 目标 - 当前)
            a = [[sum(jac[i][k] * jac[j][k] for k in range(6))
                  + (lam2 if i == j else 0.0) for j in range(5)]
                 for i in range(5)]
            g = [-sum(jac[i][k] * err[k] for k in range(6))
                 for i in range(5)]
            dq = _solve_linear(a, g)
            if dq is None:
                return None
            step = max(abs(v) for v in dq)
            if step > 0.15:
                dq = [v * 0.15 / step for v in dq]
            moved = False
            for i, n in enumerate(ALIGN_JOINTS):
                nv = max(-JOINT_LIMIT, min(JOINT_LIMIT, q[n] + dq[i]))
                if abs(nv - q[n]) > 1e-6:
                    moved = True
                q[n] = nv
            if not moved:
                break
        err = self._pose_residual(target_rot, target_pos, q, w_rot, tcp)
        if err is None:
            return None
        pos_err = math.sqrt(sum(v * v for v in err[:3]))
        rot_err = math.degrees(
            math.sqrt(sum(v * v for v in err[3:])) / w_rot)
        return {n: q[n] for n in ALIGN_JOINTS}, pos_err, rot_err

    # ------------------------------------------------------------------
    # 目标速度自适应: marker 移动速度 -> 期望角速度 -> 速率/speed
    # ------------------------------------------------------------------
    def _adapt_alpha(self, dist):
        """marker 移到所需法兰角速度, 归一化为 0..1 的加速系数."""
        omega = self._marker_vel / max(dist, 0.08)
        return min(1.0, omega / self.adapt_omega_ref)

    def _adaptive_rate(self):
        """本次轨迹的关节速率上限 [rad/s]."""
        a = 0.0 if not self.speed_adapt else self._adapt_alpha(self._last_dist)
        return (self.max_joint_rate
                + (self.max_joint_rate_fast - self.max_joint_rate) * a)

    def _update_speed_hint(self):
        """持续下发固件 speed 提示; marker 丢失时速度自然衰减."""
        if time.time() - self._marker_time > self.marker_timeout:
            self._marker_vel *= 0.8
        if not self.speed_adapt:
            return
        a = self._adapt_alpha(self._last_dist)
        v = int(round(self.speed_slow
                      + (self.speed_fast - self.speed_slow) * a))
        self._speed_pub.publish(Int32(data=v))

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def _tick(self):
        """20Hz 主循环入口; 上一拍未执行完时直接跳过, 避免并发重入."""
        if not self._tick_lock.acquire(blocking=False):
            return
        try:
            self._tick_once()
        finally:
            self._tick_lock.release()

    def _tick_once(self):
        self._update_speed_hint()
        if not self._init_done:
            # 初始化阶段独占: 等 pick_ready 轨迹走完
            if not self._busy:
                self._init_pick_ready()
            return
        now = time.time()
        if self._state != ST_ALIGN:
            # 抓取循环期间绝不允许底盘继续动
            if self._approach_active or self._retreat_active:
                self._stop_chassis()
        if self._state == ST_ALIGN:
            # marker 太远时先由底盘接近接管, 本拍不做机械臂对准
            if self._chassis_approach_step(now):
                return
            if now < self._settle_until:
                return
            # 跟踪阶段不阻塞等轨迹结果: 每 replan_interval 用最新 marker 重规划,
            # 新目标会抢占在途轨迹 (旧目标结果由序号过滤)
            if now - self._last_send < self.replan_interval:
                return
            self._align_step()
        elif self._state == ST_OPEN_PRE:
            self._tick_open_pre(now)
        elif self._state == ST_INSERT:
            self._tick_insert(now)
        elif self._state == ST_CLOSE:
            self._tick_close(now)
        elif self._state == ST_RETURN:
            self._tick_return(now)
        elif self._state == ST_OPEN_POST:
            self._tick_open_post()

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
    # 底盘接近: link1 系下 marker 水平距离 > 阈值 -> 发 /cmd_vel 前进
    # ------------------------------------------------------------------
    def _publish_chassis(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._chassis_pub.publish(msg)

    def _stop_chassis(self):
        """发零速并清接近/后退状态; 停车后再补发几拍零速确保底盘停稳."""
        if self._approach_active:
            self.get_logger().info(
                f'Chassis: stop (travel={self._approach_travel:.3f}m)')
        if self._retreat_active:
            self.get_logger().info(
                f'Chassis: retreat stop (travel={self._retreat_travel:.3f}m)')
        self._approach_active = False
        self._retreat_active = False
        self._publish_chassis(0.0, 0.0)
        self._chassis_hold_until = time.time() + 0.3

    def _finish_chassis_move(self, now):
        """底盘停车, 作废移动前的对准结论, 停稳后由机械臂重新对准."""
        self._stop_chassis()
        self._reset_align_tracking()
        self._settle_until = now + self.settle_time

    def _reset_align_tracking(self):
        """底盘移动后, 让机械臂以当前实际关节重新对准并重新判定稳定.

        旧的容差状态/稳定计时/上一条目标角都必须清掉, 否则底盘移动后
        机械臂会沿用移动前的"已对准"结论直接抓取, 姿态是错的.
        """
        self._aligned = False
        self._stable_since = None
        self._stable_ref = None
        self._last_cmd = None
        self._cmd_q = None

    def _marker_face_yaw(self):
        """marker 面法线在 base_link 系下的偏航角 [rad], 仅作诊断输出."""
        if self._marker_chassis is None:
            return None
        q = self._marker_chassis.orientation
        # 四元数旋转矩阵第三列 (marker +Z 轴)
        nx = 2.0 * (q.x * q.z + q.w * q.y)
        ny = 2.0 * (q.y * q.z - q.w * q.x)
        if math.hypot(nx, ny) < 0.3:
            return None
        return math.atan2(ny, nx)

    def _marker_bearing(self):
        """marker 在底盘系下的方位角 [rad] (正 = 偏左); 位姿不可用返回 None."""
        if self._marker_chassis is None:
            return None
        cp = self._marker_chassis.position
        return math.atan2(cp.y, cp.x)

    def _approach_control(self):
        """解算一拍底盘速度, 返回 (vx, wz).

        边走边调: 前进不因航向误差中断, 只把角速度作为修正量叠加,
        这样行进中 marker 被移动也能持续跟随; 停车瞬间残余的航向偏差
        由停车后的机械臂重新对准兜底.
        """
        err = self._marker_bearing()
        if err is None:
            return 0.0, 0.0
        wz = max(-self.approach_max_angular,
                 min(self.approach_max_angular,
                     self.approach_turn_gain * err))
        # 前进速度恒定: 用 (reach - stop) 收尾会退化成亚毫米级爬行,
        # 底盘低速死区下走不动, 距离永远差一点点到不了停车判据.
        # 停车判据每拍(20Hz)检查, 不会冲过 stop_distance.
        return self.approach_max_linear, wz

    def _retreat_step(self, now, reach):
        """marker 太近时直线后退. 返回 True 表示本拍由底盘接管.

        后退不考虑方向 (wz=0, 只发负 vx), 退到 retreat_stop_distance
        (默认 0.18m) 停车, 留出机械臂抓取空间后再重新对准.
        """
        if not self._retreat_active:
            if reach >= self.retreat_trigger_distance:
                return False
            self._retreat_active = True
            self._retreat_travel = 0.0
            self._retreat_last_time = now
            self._approach_active = False
            self._reset_align_tracking()
            self.get_logger().info(
                f'Chassis: marker {reach:.3f}m < '
                f'{self.retreat_trigger_distance:.2f}m, retreat straight back')

        if reach >= self.retreat_stop_distance:
            self.get_logger().info(
                f'Chassis: retreat done at {reach:.3f}m, re-align before grasp')
            self._finish_chassis_move(now)
            return True

        dt = max(0.0, min(now - self._retreat_last_time, 0.2))
        self._retreat_last_time = now
        if (self._retreat_travel + self.retreat_max_linear * dt
                > self.retreat_max_travel):
            self.get_logger().warn(
                f'Chassis: retreat travel limit {self.retreat_max_travel:.2f}m '
                'reached, stop')
            self._finish_chassis_move(now)
            return True
        self._retreat_travel += self.retreat_max_linear * dt
        self._publish_chassis(-self.retreat_max_linear, 0.0)
        return True

    def _chassis_approach_step(self, now):
        """底盘接近一拍. 返回 True 表示本拍已由底盘接管 (跳过机械臂对准).

        距离判据: marker 在 link1 系下的水平距离 sqrt(x^2+y^2).
        竖直高度差不影响机械臂水平可达范围, 故不参与距离计算.
        运行中每 approach_update_period(默认 0.2s = 5Hz) 用最新 marker
        位姿重新解算一次航向并修正, 其余拍重复下发当前速度指令.
        """
        if not self.chassis_approach:
            return False

        # 停车后的补零速窗口
        if (now < self._chassis_hold_until
                and not self._approach_active and not self._retreat_active):
            self._publish_chassis(0.0, 0.0)
            return True

        if (self._marker_link1 is None
                or now - self._marker_time > self.marker_timeout):
            if self._approach_active or self._retreat_active:
                self.get_logger().warn(
                    'Chassis: marker lost during approach/retreat, stop')
                self._stop_chassis()
                return True
            return False

        pos = self._marker_link1.pose.position
        reach = math.hypot(pos.x, pos.y)

        # 太近: 先直线后退到抓取距离, 再由机械臂对准
        if self._retreat_step(now, reach):
            return True

        if self._approach_active:
            # 到达判定只看距离: 接近全程边走边调, 停车瞬间残余的航向
            # 偏差由停车后的机械臂重新对准兜底, 这里不再原地补转.
            if reach <= self.approach_stop_distance:
                err = self._marker_bearing()
                err_txt = ('n/a' if err is None
                           else f'{math.degrees(err):+.1f}deg')
                self.get_logger().info(
                    f'Chassis: marker within {reach:.3f}m '
                    f'(<= {self.approach_stop_distance:.2f}m), stop '
                    f'(bearing {err_txt}, arm will re-align)')
                # 底盘移动后必须重新修正一次对准角度: 清掉移动前的
                # 已对准/稳定/目标角状态, 停稳后从当前关节重新解算
                self._finish_chassis_move(now)
                self.get_logger().info(
                    'Chassis: arrived, re-align before grasp')
                return True
        else:
            if reach <= self.approach_start_distance:
                if reach <= self.approach_stop_distance:
                    self._approach_blocked = False
                return False
            if self._approach_blocked:
                self.get_logger().warn(
                    f'Chassis: approach disabled after {self.approach_max_travel:.2f}m '
                    'travel limit', throttle_duration_sec=5.0)
                return False
            if self._marker_chassis is None:
                # 底盘系位姿不可用 (TF 缺失): 无法选向, 交回机械臂对准
                return False
            self._approach_active = True
            self._approach_travel = 0.0
            self._approach_last_time = now
            self._approach_cmd_time = 0.0
            self._approach_cmd_vx = 0.0
            self._approach_cmd_wz = 0.0
            # 接近途中目标会移动, 原有的对准/稳定计时作废
            self._reset_align_tracking()
            self.get_logger().info(
                f'Chassis: marker {reach:.3f}m > '
                f'{self.approach_start_distance:.2f}m, approach forward')

        if self._marker_chassis is None:
            self._stop_chassis()
            return True

        # 5Hz: 用最新 marker 位置重新解算航向, 运行中持续修正小车方向
        if now - self._approach_cmd_time >= self.approach_update_period:
            self._approach_cmd_time = now
            vx, wz = self._approach_control()
            self._approach_cmd_vx = vx
            self._approach_cmd_wz = wz
            err = self._marker_bearing()
            face_yaw = self._marker_face_yaw()
            face_txt = ('n/a' if face_yaw is None
                        else f'{math.degrees(face_yaw):+.1f}deg')
            self.get_logger().info(
                f'Chassis track 5Hz: marker {reach:.3f}m '
                f'bearing {math.degrees(err):+.1f}deg face_yaw {face_txt} '
                f'-> vx={vx:.3f}m/s wz={wz:+.3f}rad/s',
                throttle_duration_sec=self.print_throttle)

        # 每拍重复下发当前指令 (底盘驱动只保持最后一条命令)
        vx = self._approach_cmd_vx
        wz = self._approach_cmd_wz

        # 累计指令行程, 超过上限停车 (防止误检/漂移导致一直前顶)
        dt = max(0.0, min(now - self._approach_last_time, 0.2))
        self._approach_last_time = now
        if self._approach_travel + vx * dt > self.approach_max_travel:
            self._approach_blocked = True
            self.get_logger().warn(
                f'Chassis: travel limit {self.approach_max_travel:.2f}m '
                'reached, stop approach')
            self._stop_chassis()
            return True
        self._approach_travel += vx * dt

        self._publish_chassis(vx, wz)
        return True

    # ------------------------------------------------------------------
    # 视觉对准: J1~J5 (marker 位置 P 在法兰系)
    # ------------------------------------------------------------------
    def _align_step(self):
        now = time.time()
        if self._marker_link1 is None:
            return
        if now - self._marker_time > self.marker_timeout:
            # marker 丢失/过期: 不动的计时必须重置, 否则重新出现会立刻误触发抓取
            self._stable_since = None
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
        self._last_dist = dist

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
            self._stable_since = None
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
                self._stable_since = now
                self._stable_ref = P
            self.get_logger().info(
                f'aligned: r={r:.4f}m <= {self.tolerance:.3f}m',
                throttle_duration_sec=2.0)
            if self._grasp_stability_ok(P, now):
                self.get_logger().info(
                    f'marker still for {self.grasp_stable_time:.1f}s, '
                    'start grasp cycle')
                self._enter_open_pre()
            return
        # 滞回: 已对准后, 误差回到 release_tolerance 以内不再动作,
        # 避免检测噪声让伺服来回小幅摆动
        if self._aligned and r <= self.release_tolerance:
            self._stable_since = None
            return
        self._aligned = False
        self._stable_since = None

        # 一次性解算: 在模型里迭代出 J1~J5 目标角, 再单条轨迹下发
        target = self._solve_aim(m)
        if target is None:
            self.get_logger().warn('aim solve failed',
                                   throttle_duration_sec=2.0)
            return
        base_q = {n: (self._cmd_q[n] if self._cmd_q else self._joints[n])
                  for n in ALIGN_JOINTS}
        dq = {n: target[n] - base_q[n] for n in ALIGN_JOINTS}
        rate = self._adaptive_rate()
        # 单步幅度 = 速率 x 重规划周期: 轨迹时长正好等于一个控制周期,
        # 走到头被下一帧接管时两端速度都接近 0, 拼接处不产生速度突变
        step_cap = min(self.max_total_step, rate * self.replan_interval)
        m_step = max(abs(v) for v in dq.values())
        if m_step > step_cap:
            s = step_cap / m_step
            dq = {n: v * s for n, v in dq.items()}
            target = {n: base_q[n] + dq[n] for n in ALIGN_JOINTS}
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
                       max(abs(v) for v in dq.values()) / rate)
        detail = ' '.join(
            f'J{i + 1} {math.degrees(self._joints[n]):+.2f}->'
            f'{math.degrees(target[n]):+.2f}deg'
            for i, n in enumerate(ALIGN_JOINTS))
        self.get_logger().info(
            f'aim: P=({P[0]:.4f},{P[1]:.4f},{P[2]:.4f}) r={r:.4f}m '
            f'{detail} in {duration:.2f}s')
        self._goal_kind = 'align'
        if not self._send_positions(positions, duration, preempt=True):
            self._settle_until = time.time() + RETRY_INTERVAL
        else:
            self._cmd_q = dict(zip(ARM_JOINTS, positions))

    # ------------------------------------------------------------------
    # 对准后的抓取循环 (状态机, 由 _tick 驱动)
    # ------------------------------------------------------------------
    def _goto_state(self, state):
        self._state = state
        self._state_time = time.time()

    def _enter_align(self):
        """一个抓取循环结束, 清空对准/稳定计时后重新开始视觉对准."""
        self._aligned = False
        self._stable_since = None
        self._stable_ref = None
        self._last_cmd = None
        self._arm_done = None
        self._gripper_done = None
        self._settle_until = time.time() + self.settle_time
        self._goto_state(ST_ALIGN)

    def _grasp_stability_ok(self, P, now):
        """marker 已对准且连续 grasp_stable_time 秒几乎不动 -> True."""
        if self._stable_since is None or self._stable_ref is None:
            self._stable_since = now
            self._stable_ref = P
            return False
        if math.dist(P, self._stable_ref) > self.grasp_stable_pos_tol:
            self._stable_since = now
            self._stable_ref = P
            return False
        return now - self._stable_since >= self.grasp_stable_time

    # ---- 1. 张开夹爪 ----
    def _enter_open_pre(self):
        self.get_logger().info('Grasp: opening gripper before insert')
        if not self._send_gripper(self.gripper_open_value):
            self._enter_align()
            return
        self._goto_state(ST_OPEN_PRE)

    def _tick_open_pre(self, now):
        if self._gripper_done is None:
            return
        if not self._gripper_done:
            self.get_logger().warn('Grasp: open gripper failed, back to align')
            self._enter_align()
            return
        if now - self._gripper_done_time >= self.grasp_open_hold:
            self._enter_insert()

    # ---- 2. 沿法兰 +Z 前伸到 marker ----
    def _enter_insert(self):
        mission = self._insert_target()
        if mission is None:
            self._enter_align()
            return
        positions, duration, gap = mission
        if positions is None:
            # 夹持点已经贴到 marker, 不需要再前伸, 直接闭合
            self._enter_close()
            return
        self._arm_done = None
        self._goal_kind = 'insert'
        if not self._send_positions(positions, duration, preempt=True,
                                    profile='cruise_decel'):
            self._enter_align()
            return
        self._cmd_q = dict(zip(ARM_JOINTS, positions))
        self.get_logger().info(
            f'Grasp: move grasp point to marker {gap:.3f}m '
            f'in {duration:.2f}s')
        self._goto_state(ST_INSERT)

    def _insert_target(self):
        """夹持点 (法兰 +Z 方向 tcp_flange_z 处) 到达 marker 的目标关节角.

        不再强制沿法兰 +Z 直线前伸: 该方向已到可达边界, 留给姿态
        约 9° 的自由度, 由求解器自行选择路径, 保证夹持点真正到点.
        """
        now = time.time()
        if (self._marker_link1 is None
                or now - self._marker_time > self.marker_timeout):
            self.get_logger().warn('Grasp: marker lost before insert')
            return None
        fk = self._fk()
        if fk is None:
            return None
        rot_fe, t_fe = fk
        m = self._marker_link1.pose.position
        # 夹持点当前所在位置 -> marker 还需走的距离
        tcp_now = tuple(t_fe[i] + rot_fe[i][2] * self.tcp_flange_z
                        for i in range(3))
        d = (m.x - tcp_now[0], m.y - tcp_now[1], m.z - tcp_now[2])
        dist = math.sqrt(sum(v * v for v in d))
        gap = dist + self.insert_extra_depth
        if gap < 0.003:
            self.get_logger().info(
                f'Grasp: grasp point already at marker (gap={gap:.4f}m), '
                'skip insert')
            return None, 0.0, gap
        # insert_extra_depth > 0 时沿当前夹持点->marker 方向再深入
        u = tuple(v / dist for v in d) if dist > 1e-6 else (0.0, 0.0, 1.0)
        extra = self.insert_extra_depth
        target_pos = (m.x + u[0] * extra,
                      m.y + u[1] * extra,
                      m.z + u[2] * extra)
        # 位置约束在夹持点上, 姿态权重取 0.02 (约 9° 倾斜换位置精度)
        solved = self._solve_cartesian(
            rot_fe, target_pos, w_rot=0.02, tcp=self.tcp_flange_z)
        if solved is None:
            self.get_logger().warn(
                'Grasp: insert IK failed, back to align',
                throttle_duration_sec=2.0)
            return None
        target, pos_err, rot_err = solved
        if pos_err > 0.005:
            self.get_logger().warn(
                f'Grasp: insert unreachable '
                f'(tcp_err={pos_err * 1000:.1f}mm, rot_err={rot_err:.1f}deg), '
                'back to align')
            return None
        positions = [target.get(n, self._joints[n]) for n in ARM_JOINTS]
        base_q = {n: (self._cmd_q[n] if self._cmd_q else self._joints[n])
                  for n in ALIGN_JOINTS}
        dq = max(abs(target[n] - base_q[n]) for n in ALIGN_JOINTS)
        duration = max(self.min_move_time, dq / self.insert_rate)
        self.get_logger().info(
            f'Grasp: gap={gap:.3f}m, tcp_err={pos_err * 1000:.1f}mm '
            f'wrist tilt={rot_err:.1f}deg')
        return positions, duration, gap

    def _tick_insert(self, now):
        if self._arm_done is None:
            return
        if not self._arm_done:
            self.get_logger().warn('Grasp: insert trajectory failed, back to align')
            self._enter_align()
            return
        if now - self._arm_done_time >= self.settle_time:
            self._enter_close()

    # ---- 3. 闭合夹爪 ----
    def _enter_close(self):
        self.get_logger().info('Grasp: closing gripper')
        if not self._send_gripper(self.gripper_close_value):
            self._enter_align()
            return
        self._goto_state(ST_CLOSE)

    def _tick_close(self, now):
        if self._gripper_done is None:
            return
        if not self._gripper_done:
            self.get_logger().warn(
                'Grasp: close gripper not confirmed, return anyway')
        elif now - self._gripper_done_time < self.grasp_close_hold:
            return
        self._enter_return()

    # ---- 4. 回到 pick_ready ----
    def _enter_return(self):
        base_q = {n: (self._cmd_q[n] if self._cmd_q else self._joints[n])
                  for n in ARM_JOINTS}
        dq = max(abs(PICK_READY_JOINT_VALUES[i] - base_q[n])
                 for i, n in enumerate(ARM_JOINTS))
        duration = max(1.0, dq / self.max_joint_rate)
        self._arm_done = None
        self._goal_kind = 'return'
        if not self._send_positions(list(PICK_READY_JOINT_VALUES), duration,
                                    preempt=True):
            self._enter_align()
            return
        self._cmd_q = dict(zip(ARM_JOINTS, PICK_READY_JOINT_VALUES))
        self.get_logger().info(f'Grasp: return to pick_ready in {duration:.2f}s')
        self._goto_state(ST_RETURN)

    def _tick_return(self, now):
        if self._arm_done is None:
            return
        if not self._arm_done:
            self.get_logger().warn('Grasp: return trajectory failed')
        elif now - self._arm_done_time < self.settle_time:
            return
        self._enter_open_post()

    # ---- 5. 回位后张开夹爪, 进入下一循环 ----
    def _enter_open_post(self):
        self.get_logger().info('Grasp: opening gripper at pick_ready')
        if not self._send_gripper(self.gripper_open_value):
            self._enter_align()
            return
        self._goto_state(ST_OPEN_POST)

    def _tick_open_post(self):
        if self._gripper_done is None:
            return
        if not self._gripper_done:
            self.get_logger().warn('Grasp: open gripper after return failed')
        self.get_logger().info('Grasp cycle complete, resume ArUco align')
        self._enter_align()

    # ------------------------------------------------------------------
    # 夹爪 action
    # ------------------------------------------------------------------
    def _send_gripper(self, value: float) -> bool:
        if self._gripper_busy:
            return False
        if not self.gripper_cli.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn(
                'gripper action server not ready', throttle_duration_sec=2.0)
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(value)
        goal.command.max_effort = float(self.gripper_max_effort)
        self._gripper_busy = True
        self._gripper_done = None
        future = self.gripper_cli.send_goal_async(goal)
        future.add_done_callback(self._on_gripper_response)
        return True

    def _on_gripper_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('gripper goal rejected')
            self._gripper_done = False
            self._gripper_done_time = time.time()
            self._gripper_busy = False
            return
        handle.get_result_async().add_done_callback(self._on_gripper_result)

    def _on_gripper_result(self, future):
        wrapped = future.result()
        result = wrapped.result if wrapped is not None else None
        ok = bool(result is not None
                  and (result.reached_goal or result.stalled))
        self._gripper_done = ok
        self._gripper_done_time = time.time()
        self._gripper_busy = False

    # ------------------------------------------------------------------
    # 轨迹下发 (无规划器)
    # ------------------------------------------------------------------
    def _send_positions(self, positions, duration: float,
                        preempt: bool = False,
                        profile: str = 'trapezoidal') -> bool:
        if preempt:
            # 跟踪重规划: 取消在途轨迹, 新目标从当前实际位置接管
            self._cancel_active()
        if not self.traj_cli.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('trajectory action server not ready',
                                   throttle_duration_sec=2.0)
            return False

        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        # 起点优先用已下发的指令位置, 保证连续重规划时轨迹首尾相接
        q0 = [self._cmd_q[n] if self._cmd_q and n in self._cmd_q
              else self._joints.get(n) for n in ARM_JOINTS]
        if any(v is None for v in q0):
            # 关节反馈未就绪, 无法生成平滑插值: 退回单点
            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            pt.time_from_start = \
                rclpy.duration.Duration(seconds=duration).to_msg()
            traj.points.append(pt)
        else:
            traj.points = _smooth_points(q0, positions, duration,
                                          profile=profile)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self._goal_seq += 1
        seq = self._goal_seq
        self._active_seq = seq
        self._busy = True
        self._last_send = time.time()
        future = self.traj_cli.send_goal_async(goal)
        future.add_done_callback(
            lambda f, s=seq: self._on_goal_response(f, s))
        return True

    def _cancel_active(self):
        """取消在途轨迹 (旧目标的结果回调由序号过滤, 不影响状态)."""
        handle, self._goal_handle = self._goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()

    def _on_goal_response(self, future, seq):
        if seq != self._active_seq:
            return
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('trajectory goal rejected, will retry')
            self._busy = False
            self._cmd_q = None      # 指令位置已不可信, 下一步回到反馈基准
            self._settle_until = time.time() + RETRY_INTERVAL
            self._arm_done = False
            self._arm_done_time = time.time()
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f, s=seq: self._on_goal_result(f, s))

    def _on_goal_result(self, future, seq):
        if seq != self._active_seq:
            return      # 已被更新目标抢占, 旧结果不参与状态机
        self._busy = False
        self._goal_handle = None
        result = future.result()
        code = result.result.error_code if result is not None else -1
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(
                f'trajectory error_code={code}, will retry')
            self._cmd_q = None      # 指令位置已不可信, 下一步回到反馈基准
            self._settle_until = time.time() + RETRY_INTERVAL
            self._arm_done = False
            self._arm_done_time = time.time()
            return
        if self._goal_kind == 'init':
            self._init_done = True
            self.get_logger().info(
                'pick_ready reached, start J1~J5 align')
            self._settle_until = time.time() + self.settle_time
        elif self._goal_kind in ('insert', 'return'):
            self._arm_done = True
            self._arm_done_time = time.time()


def main():
    rclpy.init()
    node = ArucoAlignNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _sigint_handler(_signum, _frame):
        # Ctrl+C: 底盘驱动会保持最后一条速度指令, 必须先补发零速再退出
        try:
            for _ in range(5):
                node._publish_chassis(0.0, 0.0)
                time.sleep(0.05)
        finally:
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前确保底盘收到零速, 避免残留速度命令
        try:
            node._stop_chassis()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
