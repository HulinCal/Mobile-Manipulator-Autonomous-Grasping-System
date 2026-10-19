#!/usr/bin/env python3
"""Chassis obstacle-avoidance guard —— 组合入口(ROS 接线 + 状态调度).

语音/键盘手动驾驶时的反应式动态避障网关, 通过**话题分离**接入, 不修改
任何现有包:

  * 外部速度源(语音/键盘)发布到 ``/cmd_vel_in`` 作为意图(intent);
  * 本节点 20Hz 把"避障修正后的意图"发到 ``/cmd_vel`` 驱动底盘,
    输入/输出分离, 自己的输出不会回灌;
  * pick_place 任务期间自动旁路(/pick_place/state), 让位 navigation_node;
  * TF 不可用时降级为只用原始雷达; 轮廓外扩急停(锁存+滞回解除)。

模块组成(本文件只负责 ROS IO、共享状态与调度, 功能均在子模块)
----------------------------------------------------------------
============== ======================================================
模块            功能
============== ======================================================
config         全部参数 + 速度自适应公式(s/lookahead/停距)
geometry_utils 角度归一化/四元数偏航
footprint      环境感知①: 整车矩形碰撞原语(走廊净空/旋转扫掠/
               直圆弧路径预估/击杀矩形急停)
local_map      环境感知②: odom 系滚动占据栅格(侧后盲区记忆)、
               LaserScan/PointCloud2 解析
safety         每帧运动安全评估(纯旋转圆盘 / 前进路径预估急停)
planner        选择绕行方向: 最小偏转走廊(best_option)、前向半球
               搜索(search_all)、障碍质心方位、平行判定
rescue         脱困策略: 原地逆时针(CCW)转圈找第一条可通行路径(转满
               一圈无路则停转等指令)
motion         ARC 弧线蠕行 / TURN 原地转向限速 / 转向看门狗
============== ======================================================

各模块的详细功能与使用方法见各自文件开头的模块 docstring。
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .config import GuardConfig
from .geometry_utils import (EPS_VEL, ZERO_VEL, wrap_angle, yaw_from_quat)
from .footprint import RobotFootprint
from .local_map import LocalMap, parse_xyz, scan_to_xy
from .safety import evaluate_motion
from .planner import BypassPlanner
from .rescue import RescueController
from .motion import (arc_command, turn_rate, InPlaceTurn,
                     TICK_SETTLE, TICK_WATCHDOG, TICK_TIMEOUT)


class ChassisGuardNode(Node):

    def __init__(self):
        super().__init__('chassis_guard')

        # ---- parameters (declare: 默认值即文档, yaml 可覆盖) -----------
        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel_in')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('vision_obstacle_topic', '/guard/vision_obstacles')
        self.declare_parameter('map_topic', '/guard/local_map')
        self.declare_parameter('status_topic', '/guard/status')
        self.declare_parameter('pick_state_topic', '/pick_place/state')
        self.declare_parameter('odom_frame', 'odom_combined')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('map_hz', 2.0)
        # 局部动态地图
        self.declare_parameter('map_size', 6.0)          # m, 正方形边长
        self.declare_parameter('map_resolution', 0.05)   # m/格
        self.declare_parameter('map_shift_threshold', 1.0)  # m, 滚动重 centered
        self.declare_parameter('lidar_obstacle_timeout', 5.0)   # s, 障碍记忆衰减
        self.declare_parameter('vision_obstacle_timeout', 2.0)  # s
        self.declare_parameter('scan_max_range', 1.5)    # m, 只关心近处
        self.declare_parameter('scan_beams', 120)        # 每帧最多使用的光束数
        # 避障行为
        # 整车矩形轮廓(URDF 实测: 轮子外缘 y=±0.208, x=±0.233; 车体 y=±0.142)
        self.declare_parameter('robot_half_length', 0.24)  # m, 前/后缘半长
        self.declare_parameter('robot_half_width', 0.21)   # m, 半宽(轮子外缘)
        self.declare_parameter('safety_margin', 0.02)      # m, 附加安全裕量
        self.declare_parameter('brake_time', 1.0)        # s, 刹车距离=vx*brake_time
        self.declare_parameter('min_clearance', 0.3)     # m, 通道最小净空(提前触发)
        self.declare_parameter('emergency_dist', 0.10)   # m, 轮廓外扩急停距离
        self.declare_parameter('emergency_clear', 0.20) # m, 急停锁存解除距离(滞回)
        # m/s, 反应距离/转向增益的标定速度(详见 config.GuardConfig)
        self.declare_parameter('tuned_vx', 0.05)
        self.declare_parameter('align_tolerance', 0.26)  # rad, ~15deg
        self.declare_parameter('forward_align_angle', 0.9)  # rad, 内弧线绕行
        self.declare_parameter('max_veer_angle', 1.4)    # rad ~80deg
        self.declare_parameter('side_switch_margin', 0.25)  # m
        self.declare_parameter('return_confirm_cycles', 6)  # 锚点连续 N 周期空才回正
        self.declare_parameter('veer_step_deg', 5.0)     # 选向步进角
        self.declare_parameter('parallel_x', 0.10)       # m, 平行判定纵向阈值
        self.declare_parameter('steer_kp', 1.2)          # 航向 P 增益
        self.declare_parameter('max_steer_w', 0.2)       # rad/s
        self.declare_parameter('hist_sectors', 72)       # 极向直方图扇区数(5deg)
        self.declare_parameter('max_rotate_time', 20.0)  # s, 转向总时长上限
        self.declare_parameter('rescue_turn_tol', 0.05)  # rad, 脱困到位容差
        self.declare_parameter('scan_timeout', 0.5)      # s, 雷达超时失效
        # 任务旁路
        self.declare_parameter('bypass_states',
                               ['DETECTING', 'NAV_TO_APPLE', 'GRASPING',
                                'NAV_TO_DELIVERY'])

        # ---- 配置 + 功能模块组合 ---------------------------------------
        self.cfg = GuardConfig.from_params(
            lambda name: self.get_parameter(name).value)
        c = self.cfg
        self.fp = RobotFootprint(c, self.get_logger())          # 碰撞原语
        self.lmap = LocalMap(c, self.get_logger())              # 滚动栅格
        self.planner = BypassPlanner(c, self.fp, self.get_logger())
        self.rescue = RescueController(c, self.fp, c.publish_hz,
                                       self.get_logger())
        self.turn = InPlaceTurn(c)                              # TURN 看门狗

        # ---- runtime state ----------------------------------------------
        self.intent_vx = 0.0          # 意图机体线速度
        self.intent_wz = 0.0          # 意图机体角速度
        self.anchor_yaw = None        # 命令时刻世界航向(前向意图方向锚点)
        self.anchor_x = None          # 前向命令起点世界坐标(路径恢复基准)
        self.anchor_y = None
        # 侧向承诺(hysteresis): 选定左/右绕后, 锚点方向未连续清空
        # return_cycles 周期前不换侧, 防障碍前左右摇摆.
        self.commit_side = 0          # +1 左绕, -1 右绕, 0 无承诺
        self.clear_count = 0          # 锚点方向连续清空计数
        self.recover_block_cnt = 0    # 已平行但原航线持续堵塞计数
        self.locked_heading = None    # 锁定绕行偏移(相对锚点, 世界系恒定)
        # 脱困转圈锁存: 满足三条件(非绕行 + 视野无路)才置位
        self.emergency_active = False
        # 绕过障碍的进度判据(用户: 车心与障碍平行即视为已越过)
        self.commit_xy = None    # 承诺绕行时的车体起点(世界系)
        self.commit_obs_x = None  # 承诺时障碍近端在锚点系的纵向坐标
        self._last_commit_side = 0  # 上一次记账时的承诺侧(换侧/解约即重记)
        # 连续碰撞停车计数(发布非零速度时清零): 绕行承诺下反复卡住时
        # 据此放弃死守绕行、转判为"无路可走", 允许脱困
        self.stuck_cnt = 0
        self.front_gap = float('inf')   # 最近一帧车前方轮廓净空(m)
        # 原始雷达点(激光/车体系, 2D); 不经过地图/TF
        self.raw_pts = np.zeros((0, 2))
        self.scan_stamp = -1.0
        self.bypass = False
        self.last_status = ''
        self._last_sent = ZERO_VEL    # 守卫实际发布速度(急停评估用)

        # ---- TF ----------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- IO ----------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(Twist, c.cmd_topic, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, c.map_topic, 2)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        status_qos.durability = 2  # TRANSIENT_LOCAL: 后连调试者收到最近状态
        self.status_pub = self.create_publisher(String, c.status_topic,
                                                status_qos)
        self.create_subscription(String, c.pick_state_topic,
                                 self._on_pick_state, 10)
        # 只订 /cmd_vel_in, 自己发 /cmd_vel 不回灌
        self.create_subscription(Twist, c.cmd_in_topic,
                                 self._on_cmd_vel, 10)
        self.create_subscription(LaserScan, c.scan_topic,
                                 self._on_scan, sensor_qos)
        self.create_subscription(PointCloud2, c.vision_topic,
                                 self._on_vision_points, 10)

        self.ctrl_timer = self.create_timer(1.0 / c.publish_hz,
                                            self._control_loop)
        self.map_timer = self.create_timer(1.0 / c.map_hz,
                                           self._publish_map)
        self.pub.publish(Twist())  # 启动安全零速
        self._set_status('ACTIVE', 'guard started (overlay on '
                                   + c.cmd_topic + ')')
        self.get_logger().info(
            f'chassis_guard started: cmd={c.cmd_topic}, '
            f'map={self.lmap.n_grid}x{self.lmap.n_grid}@{c.map_resolution}m, '
            f'lidar_timeout={c.lidar_timeout}s, vision_topic={c.vision_topic}')

    # ------------------------------------------------------------------
    # 状态发布
    # ------------------------------------------------------------------
    def _set_status(self, state: str, info: str = ''):
        text = f'{state}: {info}' if info else state
        if text != self.last_status:
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)
            self.last_status = text

    # ------------------------------------------------------------------
    # 任务旁路: pick_place 期间让位 navigation_node
    # ------------------------------------------------------------------
    def _on_pick_state(self, msg: String):
        state = msg.data.strip()
        task_active = any(s in state for s in self.cfg.bypass_states)
        if task_active and not self.bypass:
            self.bypass = True
            self.intent_vx = self.intent_wz = 0.0
            self.anchor_yaw = None
            self.commit_side = 0
            self.clear_count = 0
            self.locked_heading = None
            self._send_once(ZERO_VEL)
            self._set_status('BYPASS', f'pick_place state={state}')
            self.get_logger().info(
                f'Bypass enabled (pick_place state: {state}); '
                'manual cmd_vel passes straight through to consumers.')
        elif not task_active and self.bypass:
            self.bypass = False
            self._set_status('ACTIVE', 'pick_place task finished')
            self.get_logger().info('Bypass disabled: guard re-activated '
                                   '(fresh voice command required).')

    # ------------------------------------------------------------------
    # 命令捕获: 外部命令 -> 意图更新
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist):
        if self.bypass:
            return
        vx, wz = float(msg.linear.x), float(msg.angular.z)
        if vx == self.intent_vx and wz == self.intent_wz:
            # 同一命令的重复发布(键盘/语音心跳)一律忽略 —— 包括脱困转圈中:
            # 重复的同一条语音不得打断转圈, 否则每秒被取消一次、spun 归零,
            # 小车转两步就停(实测"脱困时不动"的主因之一). 只有**命令真的
            # 变化**才打断脱困/急停锁存.
            return
        self.get_logger().info(
            f'External command: vx={vx:+.3f} wz={wz:+.3f}')
        self.intent_vx, self.intent_wz = vx, wz
        self.turn.reset()
        # 新命令优先级最高(用户要求): 立即打断脱困/急停锁存; 但碰撞保护
        # 不取消 —— 新命令真会撞时, _on_scan/rollout 下一帧会重新锁停.
        if self.emergency_active:
            self.get_logger().info(
                'New command received: override rescue lock, '
                'resume normal following')
            self.emergency_active = False
            self.rescue.reset()
        # 新(变化的)命令: 清绕行承诺, 重新锚定原方向
        self.commit_side = 0
        self.clear_count = 0
        self.recover_block_cnt = 0
        self.locked_heading = None
        self.stuck_cnt = 0
        if abs(vx) > EPS_VEL:
            pose = self._robot_pose()
            if pose is not None:
                self.anchor_x, self.anchor_y, self.anchor_yaw = pose
            else:
                self.anchor_yaw = self._robot_yaw()
        else:
            self.anchor_yaw = self._robot_yaw()

    # ------------------------------------------------------------------
    # 脱困入口: 三条件门控(状态机本体在 rescue.RescueController)
    # ------------------------------------------------------------------
    def _nearest_lane_obs_x(self, pts):
        """障碍物近端在**锚点系**(原航线方向)的纵向坐标 x(相对当前车心).

        只统计落在原航线车道内(|横坐标| < half_w+0.06)且在车心前方的点,
        取最小纵向坐标 = 障碍离车心最近的那个面。
        """
        pose = self._robot_pose()
        if pose is None or self.anchor_yaw is None or pts is None \
                or len(pts) == 0:
            return None
        d = wrap_angle(pose[2] - self.anchor_yaw)
        xa = math.cos(d) * pts[:, 0] - math.sin(d) * pts[:, 1]
        ya = math.sin(d) * pts[:, 0] + math.cos(d) * pts[:, 1]
        m = (np.abs(ya) < self.cfg.half_w + 0.06) & (xa > 0.0)
        if not np.any(m):
            return None
        return float(np.min(xa[m]))

    def _obstacle_passed(self, pts) -> bool:
        """车体中心是否已与所绕障碍物平行(用户判据).

        用户原话: "只要小车的中心点与障碍物平行就应该满足脱困条件(也就是
        小车转向原来航线时)". 即中心沿**原航线方向**的进度到达障碍物近端
        纵向坐标处, 就算已越过。

        **不能**用"锚点系车道内前方还有没有点"的逐帧点集判据: 车体大角度
        绕行时, 车侧/车后的点会在锚点系里落到"前方车道"内, 造成明明早已
        越过障碍却判 passed=False(实测: 车已离开障碍 1m 多仍判未通过,
        脱困被门控挡住)。故改为**按自身进度判**: 承诺绕行那一刻记下起点
        与障碍近端纵向坐标 x_obs, 之后只看车体沿原航线走了多远:
        progress >= x_obs - parallel_x 即判平行/已越过。
        """
        # 承诺侧变化(含 0->+1/-1、换侧 +1->-1)即重新记账: 旧进度作废
        if self.commit_side != self._last_commit_side:
            self.commit_xy = None
            self.commit_obs_x = None
            self._last_commit_side = self.commit_side
        if self.commit_side == 0 or self.locked_heading is None \
                or self.anchor_yaw is None:
            return True
        pose = self._robot_pose()
        if pose is None:
            return False
        if self.commit_xy is None:
            # 刚承诺绕行: 记录障碍近端纵向坐标(没有则本帧不计账)
            obs = self._nearest_lane_obs_x(pts)
            if obs is None:
                # 锚点车道前方本无障碍 -> 立即视为已越过(门控可放行)
                return True
            self.commit_xy = (float(pose[0]), float(pose[1]))
            self.commit_obs_x = obs
            return False            # 刚起步, 不可能已平行
        dx = pose[0] - self.commit_xy[0]
        dy = pose[1] - self.commit_xy[1]
        progress = (dx * math.cos(self.anchor_yaw)
                    + dy * math.sin(self.anchor_yaw))
        return progress >= self.commit_obs_x - self.cfg.parallel_x

    STUCK_RELEASE = 10       # 连续受阻帧数: 超过则判定"锁定方向真的走不通"

    def _try_rescue(self, pts, reason):
        """门控通过才进入脱困(原地逆时针转圈找路), 否则返回 False.

        门控条件:
        1. **不在绕行阶段** —— 未承诺绕行, 或已越过障碍物(平行通过);
        2. **视野内没有"可达"的可通行路线** —— 前向 ±90° 内 search_all
           找到的走廊必须能**原地转到**(rotation_blocked=False); 若走廊
           存在但转不过去, 视为不可达, 不否决脱困(否则每帧选中同一条
           走廊、每帧被扫掠否决, 永久静止)。

        **卡住强制脱困**(兜底): 连续受阻 STUCK_RELEASE 帧后直接进脱困,
        绕过门控 —— 那条"路"可能不可达或陷入振荡, 原地转圈是唯一能改变
        位姿的手段。

        :returns: True = 已在脱困(含本次启动); False = 门控未过。
        """
        if self.rescue.state != '':
            return True                       # 已在脱困中
        look = self.cfg.lookahead(self.intent_vx)

        # ---- 卡住强制脱困(最后兜底) ----
        # 连续 STUCK_RELEASE 帧被拦: 即使门控说"视野有路", 那条路也可能
        # 不可达(转不过去)或陷入振荡; 原地转圈是唯一能改变位姿的手段,
        # 故直接进脱困, 不再受 search_all 门控约束.
        # 旧实现要求 commit_side != 0: 但 abort 路径已把 commit_side 清 0,
        # 条件恒 False, stuck 永远释放不了(实测死锁根因之一).
        if self.stuck_cnt >= self.STUCK_RELEASE:
            if self.commit_side != 0:
                self.commit_side = 0
                self.locked_heading = None
                self.clear_count = 0
                self.recover_block_cnt = 0
                self.turn.reset()
            self.stuck_cnt = 0
            self.rescue.enter()
            self.emergency_active = True
            self._publish(0.0, 0.0)
            self._set_status(
                'EMERGENCY',
                f'stuck {self.STUCK_RELEASE} frames: force CCW rescue')
            self.get_logger().error(
                f'RESCUE FORCED: stuck {self.STUCK_RELEASE} frames, '
                'override gate and rotate CCW to search a path')
            return True

        # ---- 门控: 前向半球是否存在"可达"的可通行走廊 ----
        # 关键修复: search_all 只判"直线走廊净空 > lookahead", 但车要先
        # 原地转到该航向才能走; 若 rotation_blocked 判定转不过去, 这条
        # "路"实际不可达, 不能用来否决脱困(否则每帧选中同一条走廊、每帧
        # 被扫掠否决, 物理上车需要"先挪一下"才能转过去, 而唯一能改变
        # 位姿的脱困转圈恰恰被门控挡死 -> 永久静止).
        found = self.planner.search_all(pts, look)
        if found is not None and self.fp.rotation_blocked(
                pts, found[1],
                half_l=self.cfg.robot_half_length,
                half_w=self.cfg.robot_half_width):
            self.get_logger().warn(
                f'[dbg] corridor at {np.degrees(found[1]):+.0f}deg/'
                f'{found[0]:.2f}m unreachable (rotation blocked), '
                'treat as no path',
                throttle_duration_sec=1.0)
            found = None
        in_bypass = self.commit_side != 0 and not self._obstacle_passed(pts)
        if found is not None or in_bypass:
            self.get_logger().warn(
                f'[dbg] rescue gated: in_bypass={in_bypass} '
                f'commit={self.commit_side} stuck={self.stuck_cnt} '
                f'passed={self._obstacle_passed(pts)} route='
                + ('none' if found is None else
                   f'{np.degrees(found[1]):+.0f}deg/{found[0]:.2f}m'),
                throttle_duration_sec=1.0)
            return False
        self.rescue.enter()
        self.emergency_active = True
        self._publish(0.0, 0.0)               # 启动即零速
        self._set_status('EMERGENCY',
                         f'{reason}; rescue: rotate CCW to find path')
        self.get_logger().error(
            f'RESCUE START: {reason}; rotating CCW to search a path')
        return True

    def _abort_bypass_and_rescue(self, pts):
        """确认当前方向走不通: 放弃绕行承诺, 零速后启动脱困(若门控通过)."""
        # 连续 abort(rotation_blocked 扫掠否决)也算卡住: 每帧 +1, 达
        # STUCK_RELEASE 后由 _try_rescue 强制脱困. 旧实现此处不涨计数,
        # 导致 stuck 永远到不了阈值(死锁根因之二).
        self.stuck_cnt += 1
        self.commit_side = 0
        self.locked_heading = None
        self._publish(0.0, 0.0)               # 绝不把旧速度留给底盘
        return self._try_rescue(pts, 'no passable corridor in view (360 deg)')

    # ------------------------------------------------------------------
    # 雷达回调: (1) 原始点云 + 每帧硬急停 (2) 写局部动态地图
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds * 1e-9
        n = len(msg.ranges)
        if n:
            self.raw_pts, n = scan_to_xy(msg)
            self.scan_stamp = now

            # 用守卫**实际发布**速度评估(_last_sent), 不用用户意图:
            # 原地 TURN 时意图仍是前进, 按前进预估会误急停(实测).
            # 绕行中(已承诺方向)收紧阈值: 只在即将真正接触才触发脱困,
            # 避免弧线让开障碍前的贴边过渡帧被误判(用户要求: 路径无碰撞
            # 就继续前进, 不随便脱困).
            act_vx, act_wz = self._last_sent
            bypassing = self.commit_side != 0 and \
                self.locked_heading is not None
            limit = self.cfg.emergency_stop_distance(act_vx) \
                if bypassing else None
            m = evaluate_motion(self.fp, self.raw_pts, act_vx, act_wz,
                                self.cfg, limit)
            self.front_gap = m.front_gap
            moving = abs(act_vx) > EPS_VEL or abs(act_wz) > EPS_VEL
            if not self.emergency_active:
                if moving and m.blocked:
                    # 安全底线: 立即零速(不等控制周期)
                    self._publish(0.0, 0.0)
                    self.stuck_cnt += 1
                    self.get_logger().warn(
                        f'[dbg] collision stop: gap={self.front_gap:+.2f} m '
                        f'actual vx={act_vx:+.2f} wz={act_wz:+.2f} '
                        f'(intent {self.intent_vx:+.2f}/{self.intent_wz:+.2f})',
                        throttle_duration_sec=1.0)
                    # 门控通过(非绕行 + 视野无路)才进脱困; 否则交控制周期
                    # 重新规划(有路就正常绕行, 不轻易脱困)
                    self._try_rescue(
                        self.raw_pts,
                        f'obstacle within footprint '
                        f'(gap {self.front_gap:+.2f} m)')

        # ---- (2) 写地图(急停不依赖此步, TF 失败也不影响急停) ----------
        tf = self._tf_odom_scan(msg.header.frame_id)
        if tf is None:
            return
        ox = tf.transform.translation.x
        oy = tf.transform.translation.y
        oyaw = yaw_from_quat(tf.transform.rotation)

        if not self.lmap.initialized:
            self.lmap.initialize(ox, oy)
            self.get_logger().info(
                f'Local map centered at odom pose ({ox:.2f}, {oy:.2f})')

        if n == 0:
            return
        step = max(1, n // self.cfg.scan_beams)
        max_r2 = min(self.cfg.scan_max_range, float(msg.range_max or 1e3))
        min_r2 = float(msg.range_min or 0.0)

        for i in range(0, n, step):
            r = msg.ranges[i]
            if not np.isfinite(r) or r <= min_r2:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            wx = ox + r * np.cos(oyaw + ang)
            wy = oy + r * np.sin(oyaw + ang)
            hit = r < max_r2
            self.lmap.ray_update(ox, oy, wx, wy, hit, now,
                                 self.cfg.lidar_timeout)

    # ------------------------------------------------------------------
    # 视觉融合预留: PointCloud2 点写入地图
    # ------------------------------------------------------------------
    def _on_vision_points(self, msg: PointCloud2):
        """未来视觉节点发布 PointCloud2 即零改动融合(字段 xyz/float32)."""
        pts = parse_xyz(msg)
        if not pts:
            return
        tf = self._tf_odom_scan(msg.header.frame_id)
        if tf is None:
            self.get_logger().warn('vision points: no TF '
                                   f'{self.cfg.odom_frame} <- {msg.header.frame_id}',
                                   throttle_duration_sec=5.0)
            return
        ox = tf.transform.translation.x
        oy = tf.transform.translation.y
        oyaw = yaw_from_quat(tf.transform.rotation)
        now = self.get_clock().now().nanoseconds * 1e-9
        for (x, y, _z) in pts:
            wx = ox + x * np.cos(oyaw) - y * np.sin(oyaw)
            wy = oy + x * np.sin(oyaw) + y * np.cos(oyaw)
            i, j = self.lmap.world_to_cell(wx, wy)
            if i is not None:
                self.lmap.occ_t[i, j] = now
                self.lmap.occ_timeout[i, j] = self.cfg.vision_timeout

    # ------------------------------------------------------------------
    # TF 辅助
    # ------------------------------------------------------------------
    def _tf_odom_scan(self, frame_id: str):
        try:
            return self.tf_buffer.lookup_transform(
                self.cfg.odom_frame, frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            return None

    def _robot_pose(self):
        tf = self._tf_odom_scan(self.cfg.robot_frame)
        if tf is None:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y,
                yaw_from_quat(tf.transform.rotation))

    def _robot_yaw(self):
        pose = self._robot_pose()
        return pose[2] if pose else None

    # ------------------------------------------------------------------
    # 主控制循环
    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.bypass:
            return
        c = self.cfg
        now = self.get_clock().now().nanoseconds * 1e-9

        # ============ 最高优先级: 脱困状态机(原地逆时针转圈找路) =========
        if self.emergency_active:
            pts_r = self.raw_pts
            look_dbg = c.lookahead(self.intent_vx)
            near_dbg = (float(np.min(np.hypot(pts_r[:, 0], pts_r[:, 1])))
                        if len(pts_r) else float('inf'))
            rot_dbg = self.fp.rotation_blocked(
                pts_r, 0.01, step=0.01,
                half_l=c.robot_half_length, half_w=c.robot_half_width)
            self.get_logger().warn(
                f'[dbg] rescue_state={self.rescue.state or "?"} '
                f'spun={np.degrees(self.rescue.spun):.0f}deg '
                f'corridor(0)={self.fp.corridor_clearance(pts_r, 0.0):.2f} '
                f'look={look_dbg:.2f} near={near_dbg:.2f} '
                f'rotblk={rot_dbg} pts={len(pts_r)} '
                f'vx={self.intent_vx:+.2f}',
                throttle_duration_sec=1.0)

            out = self.rescue.step(pts_r, look_dbg)
            if out.kind == 'spin':
                # 原地逆时针转动(z 正方向 = 车头向左), 边转边找路
                self._publish(0.0, out.wz)
                self._set_status('EMERGENCY', out.status)
                return
            if out.kind == 'hold':
                # 只可能是 state 非 'turn'(正常不会走到): 兜底零速, 不闭锁
                self._publish(0.0, 0.0)
                self._set_status('EMERGENCY', out.status)
                return
            # done: 找到第一条可通行路径 -> 以当前车头为新锚点直接前进
            pose = self._robot_pose()
            if pose is not None:
                self.anchor_yaw, self.anchor_x, self.anchor_y = pose
            self.commit_side = 0
            self.locked_heading = None
            self.clear_count = 0
            self.recover_block_cnt = 0
            self.turn.reset()
            self.stuck_cnt = 0          # 脱困成功 -> 不再卡住
            self.emergency_active = False
            self.rescue.complete()
            self._set_status('ACTIVE', 'rescue: path found, go forward')
            # 不 return: 落到下方正常流程, 本帧即以新航向前进

        # 零速度意图: 补发一次零速后静默
        if abs(self.intent_vx) < EPS_VEL and abs(self.intent_wz) < EPS_VEL:
            self.turn.reset()
            self._send_once(ZERO_VEL)
            self._set_status('ACTIVE', 'idle (waiting command)')
            return

        # 雷达数据失效: 停车不盲走
        if self.scan_stamp < 0 or now - self.scan_stamp > c.scan_timeout:
            self._send_once(ZERO_VEL)
            self._set_status('DEGRADED', 'no fresh scan, holding still')
            return

        # ---- 融合障碍点: 原始雷达(真值) + 记忆地图(侧后盲区) ----------
        pts = self.raw_pts
        pose = self._robot_pose()
        if pose is not None:
            rx, ry, yaw = pose
            self.lmap.recenter(rx, ry)
            mem = self.lmap.memory_points(rx, ry, yaw, now)
            if len(mem):
                pts = np.vstack((pts, mem))
            anchor_rel = wrap_angle(
                (self.anchor_yaw if self.anchor_yaw is not None else yaw)
                - yaw)
        else:
            anchor_rel = 0.0
            self._set_status('DEGRADED',
                             f'no TF {c.odom_frame}<->{c.robot_frame}; '
                             'raw scan only')

        # ---- 纯旋转意图: 外接圆盘碰撞检查 -----------------------------
        if abs(self.intent_wz) > EPS_VEL and abs(self.intent_vx) < EPS_VEL:
            if self.fp.disk_blocked(pts):
                self._send_once(ZERO_VEL)
                gap = float(np.hypot(pts[:, 0], pts[:, 1]).min())
                self._set_status('BLOCKED',
                                 f'rotation obstacle {gap:.2f} m')
            else:
                self.turn.reset()
                self._publish(0.0, self.intent_wz)
                self._set_status('ACTIVE', 'rotating (disk clear)')
            return

        # ---- 前向意图: 走廊判定 + 选向状态机 ---------------------------
        vx_cmd = abs(self.intent_vx)
        s = c.speed_scale(vx_cmd)
        lookahead = c.lookahead(vx_cmd)

        def clear(heading):
            return self.planner.is_clear(pts, heading, lookahead)

        def depth(heading):
            return self.planner.depth(pts, heading)

        # TURN 锁存: 正在原地转向时固定 commit_side 不换侧, 防振荡
        turning_now = self.turn.rotate_since is not None \
            and abs(self.intent_vx) < EPS_VEL

        # 转向超时闭锁机制已删除: 它会让小车无声永久零速(实测静止根因),
        # 现改为"分段续转"——超时只重置本段计时并继续转向。

        # 平行判定(用户逻辑): "小车中心点与障碍物在同一个 x 值的位置",
        # 用 _obstacle_passed 的**自身进度**判据(稳健, 不受绕行大角度时
        # 车侧/车后点落进锚点系车道内造成的误判影响).
        # 平行前 clear_count 锁死为 0 —— 锚点方向暂时畅通也不回正.
        parallel_now = True
        if self.commit_side != 0 and self.locked_heading is not None \
                and pose is not None:
            parallel_now = self._obstacle_passed(pts)
        # TURN 期间不更新 clear_count(anchor_rel 随 yaw 变化, 检测无意义)
        if not turning_now:
            if parallel_now and clear(anchor_rel):
                self.clear_count += 1
            else:
                self.clear_count = 0

        target = None
        side_tag = 0
        chosen_depth = float('inf')

        if self.commit_side == 0:
            # 未承诺: 锚点畅通就直走(避免刚启动前几帧误转向)
            if clear(anchor_rel):
                target = anchor_rel
                chosen_depth = depth(anchor_rel)
            else:
                # 延迟转向: 障碍还远(>0.4m*s)继续直行, 等接近再转
                front_d = depth(anchor_rel)
                if front_d > 0.4 * s:
                    target = anchor_rel
                    chosen_depth = front_d
                else:
                    left = self.planner.best_option(pts, anchor_rel, +1,
                                                    lookahead)
                    right = self.planner.best_option(pts, anchor_rel, -1,
                                                     lookahead)
                    if left is None and right is None:
                        # ±80° 无走廊: 前向半球 ±90° 兜底(不后退)
                        found = self.planner.search_all(pts, lookahead)
                        if found is not None:
                            target = found[1]
                            chosen_depth = found[0]
                            self.commit_side = 0
                            self.locked_heading = None
                            self.get_logger().info(
                                f'Front blocked, search found corridor at '
                                f'{np.degrees(found[1]):+.0f} deg, '
                                f'depth {found[0]:.2f} m')
                        else:
                            # 前向半球彻底无路: 启动脱困(逆时针转圈找路)
                            self._abort_bypass_and_rescue(pts)
                            return
                    # 方向稳定性: 优先绕障碍所在侧的对侧; 居中取更深者
                    obs_side = self.planner.obstacle_side(pts)
                    if obs_side != 0:
                        self.commit_side = obs_side
                    elif left is not None and right is not None:
                        self.commit_side = +1 if left[0] >= right[0] else -1
                    elif left is not None:
                        self.commit_side = +1
                    else:
                        self.commit_side = -1
                    self.get_logger().info(
                        f'Obstacle ahead, commit '
                        f'{"LEFT" if self.commit_side > 0 else "RIGHT"} '
                        f'L={None if left is None else round(left[0], 2)} '
                        f'R={None if right is None else round(right[0], 2)}')
                    self.clear_count = 0
                    first_opt = self.planner.best_option(
                        pts, anchor_rel, self.commit_side, lookahead)
                    # 锁定相对锚点的世界系偏移: 转动时目标世界方向不变
                    self.locked_heading = wrap_angle(first_opt[1] - anchor_rel) \
                        if first_opt else None
                    # anchor_yaw 保持不变, 越障后自然回到原始意图方向
        else:
            # 已承诺: 锚点连续畅通 return_cycles 才回正, 否则评估换侧;
            # TURN 期间禁止换侧(防原地转圈振荡).
            if self.clear_count >= c.return_cycles:
                # 障碍已通过: 回正航向 + 横向偏移修正拉回原路径.
                # anchor_x/y/yaw 全程保持首次命令值, 不重置(防锚点漂移).
                self.locked_heading = None
                lat = 0.0
                if pose is not None and self.anchor_x is not None:
                    rx, ry, _ = pose
                    dx, dy = rx - self.anchor_x, ry - self.anchor_y
                    lat = (-dx * np.sin(self.anchor_yaw)
                           + dy * np.cos(self.anchor_yaw))
                lat_kp = 0.6  # rad/m, 偏 0.5m 给约 17° 修正
                target = anchor_rel - lat_kp * lat
                # 恢复方向必须真畅通; 只有带横向修正的航向被挡时退化为
                # 直接回锚点方向(本分支已确认锚点方向畅通), 绝不换锚.
                if not clear(target):
                    target = anchor_rel
                chosen_depth = depth(target)
                if abs(lat) < 0.1 and abs(anchor_rel) < c.align_tol:
                    self.commit_side = 0
                    target = anchor_rel
                    chosen_depth = depth(anchor_rel)
                side_tag = 0
            elif turning_now:
                # TURN 锁存: 不重选 commit_side, 继续用已承诺方向
                pass
            elif parallel_now and not clear(anchor_rel):
                # 已平行但原航线仍堵: 连续 return_cycles*3 周期后换锚,
                # 以当前前行方向为新原方向; 期间锁定航向不左右找缝隙.
                locked_now = wrap_angle(anchor_rel + self.locked_heading) \
                    if self.locked_heading is not None else anchor_rel
                if clear(locked_now):
                    self.recover_block_cnt += 1
                else:
                    self.recover_block_cnt = 0   # 连前行方向都堵: 交下方换侧
                if self.recover_block_cnt >= c.return_cycles * 3:
                    self.get_logger().info(
                        'Original line persistently blocked after passing '
                        'obstacle, re-anchor to current heading')
                    if pose is not None:
                        self.anchor_yaw = float(wrap_angle(yaw + locked_now))
                        self.anchor_x = pose[0]
                        self.anchor_y = pose[1]
                    self.commit_side = 0
                    self.locked_heading = None
                    self.clear_count = 0
                    self.recover_block_cnt = 0
                    anchor_rel = 0.0   # 局部同步, 防旧锚点覆盖目标
                    target = 0.0
                    chosen_depth = depth(0.0)
                    side_tag = 0
            elif not parallel_now:
                # 绕行锁定期(平行前): 方向完全冻结, 不重算/不换侧/不重选,
                # 碰撞安全由 rollout 急停兜底(用户要求: 平行前禁止改向).
                self.recover_block_cnt = 0
                pass
            else:
                left = self.planner.best_option(pts, anchor_rel, +1,
                                                lookahead)
                right = self.planner.best_option(pts, anchor_rel, -1,
                                                 lookahead)
                if left is None and right is None:
                    found = self.planner.search_all(pts, lookahead)
                    if found is not None:
                        target = found[1]
                        chosen_depth = found[0]
                        # 锁定该走廊为承诺航向(不清零! 清零会逐帧重选抖动)
                        self.commit_side = 1 if found[1] >= 0 else -1
                        self.locked_heading = wrap_angle(
                            found[1] - anchor_rel)
                        self.get_logger().info(
                            f'Front blocked, search found corridor at '
                            f'{np.degrees(found[1]):+.0f} deg, '
                            f'depth {found[0]:.2f} m')
                    else:
                        # 前向半球彻底无路: 启动脱困(逆时针转圈找路)
                        self._abort_bypass_and_rescue(pts)
                        return
                cur = left if self.commit_side > 0 else right
                opp = right if self.commit_side > 0 else left
                # 仅当前侧完全堵死且对侧可用才换侧; 深度波动不换侧(防摇摆)
                if cur is None and opp is not None and not turning_now:
                    self.commit_side = -self.commit_side
                    self.locked_heading = wrap_angle(opp[1] - anchor_rel)
                    self.clear_count = 0
                    self.get_logger().info(
                        f'Veer side blocked, switch to '
                        f'{"LEFT" if self.commit_side > 0 else "RIGHT"}')

        # 锁定 heading(世界系): 目标 = 锚点 + 锁定偏移. 平行前方向冻结:
        # 即使锁定航向走廊暂不畅通也保持不变, 由 rollout 急停兜底.
        locked_world = None
        if self.commit_side != 0 and self.locked_heading is not None:
            locked_world = wrap_angle(anchor_rel + self.locked_heading)
        if locked_world is not None:
            target = locked_world
            chosen_depth = depth(locked_world)
        elif target is None:
            opt = self.planner.best_option(pts, anchor_rel, self.commit_side,
                                           lookahead)
            if opt is None:
                found = self.planner.search_all(pts, lookahead)
                if found is not None:
                    target = found[1]
                    chosen_depth = found[0]
                    # 同样锁定不清零(防逐帧重选抖动)
                    self.commit_side = 1 if found[1] >= 0 else -1
                    self.locked_heading = wrap_angle(found[1] - anchor_rel)
                    self.get_logger().info(
                        f'Committed side closed, search found '
                        f'corridor at {np.degrees(found[1]):+.0f} deg, '
                        f'depth {found[0]:.2f} m')
                else:
                    # 承诺侧与兜底搜索都无路: 放弃绕行后启动脱困(逆时针转圈)
                    self._abort_bypass_and_rescue(pts)
                    return
            else:
                chosen_depth, target = opt[0], opt[1]
            side_tag = self.commit_side
            # 注: 旧版 tight-pass(深度 ~0.33m 的贴身缝隙)已删除 —— 该深度
            # < lookahead/实际停距, 车驶入即被 rollout 急停(选定却不走).

        # ---- 控制律: 小偏差 ARC 弧线蠕行, 大偏差 TURN 原地对准 ---------
        err = wrap_angle(target)
        if abs(err) <= c.fwd_align:
            vx, wz, err = arc_command(c, self.intent_vx, err)
            self.turn.reset()
            mode = 'ARC'
        else:
            # 矩形逐位姿扫掠判定能否转过去(禁止用外接圆盘: 车头前 0.33m
            # 的障碍会被判成任何角度都不能转, 小车抱死).
            # 用**物理车体矩形**: 车已停在离障碍很近处, 带 0.02 裕量的规划
            # 矩形会让任何角度都判 obstructed -> 无声零速永久静止(实测).
            if self.fp.rotation_blocked(
                    pts, err,
                    half_l=c.robot_half_length, half_w=c.robot_half_width):
                self.turn.reset()
                self.get_logger().warn(
                    f'bypass turn obstructed (target '
                    f'{np.degrees(err):+.0f} deg): abort bypass, re-plan',
                    throttle_duration_sec=1.0)
                # 不能无声死等: 放弃本次绕行承诺, 重新选向(或转脱困)
                self._abort_bypass_and_rescue(pts)
                return
            self.turn.begin_if_needed(now, err)
            flag = self.turn.tick(now, err)
            if flag == TICK_SETTLE:
                self._publish(0.0, 0.0)
                self._set_status('AVOIDING',
                                 f'turn settle, err {np.degrees(err):+.0f} deg')
                return
            if flag == TICK_WATCHDOG:
                self._publish(0.0, 0.0)
                self._set_status('AVOIDING', 'turn watchdog pause')
                return
            if flag == TICK_TIMEOUT:
                # 分段续转, 绝不无声死锁: 旧实现置 turn_blocked 闭锁零速直到
                # 锚点畅通 -> 实测永久静止; 且 max_rotate_time=4s @0.15rad/s
                # 只能转 34°, 而 TURN 恰在 |err|>34° 才启用, 必然超时.
                self.turn.reset()
                self.turn.begin_if_needed(now, err)
                self.get_logger().warn(
                    f'turn segment budget hit, continuing turn toward '
                    f'{np.degrees(err):+.0f} deg',
                    throttle_duration_sec=2.0)
            vx = 0.0
            wz = turn_rate(c, err)
            self.turn.accumulate(wz, 1.0 / c.publish_hz)
            mode = 'TURN'

        # ---- 路径预估闭环: 最终 (vx,wz) 扫掠轮廓验证后才下发 -----------
        # 只用实时雷达点 raw_pts: 记忆点经 TF 受 EKF 漂移影响可能假阳性.
        if vx > EPS_VEL:
            stop_dist = c.stop_distance(vx)
            # 绕行中(已承诺方向)收紧到紧急停距离: 弧线让开障碍前的贴边
            # 过渡帧(碰撞 0.2~0.3m)不算急停条件; 只有几乎真正接触才脱困.
            # 直行接近障碍仍用完整刹车距离(保留原有预判刹车行为).
            trig_dist = c.emergency_stop_distance(vx) \
                if (self.commit_side != 0 and self.locked_heading is not None) \
                else stop_dist
            clear_d, hit = self.fp.rollout_clearance(
                self.raw_pts, vx, wz,
                max(stop_dist + 0.6 * s, 1.2 * s),
                half_l=c.kill_half_l, half_w=c.kill_half_w)
            if hit and clear_d < trig_dist:
                self.front_gap = clear_d
                # 安全底线: 立即零速; 只有"非绕行 + 视野无路"才进脱困
                self._publish(0.0, 0.0)
                self.stuck_cnt += 1
                self._try_rescue(
                    self.raw_pts,
                    f'collision in {clear_d:.2f} m along path')
                return

        self._publish(vx, wz)
        if side_tag == 0 and abs(err) < c.align_tol:
            self._set_status('ACTIVE', 'cruise (footprint clear)')
        elif side_tag == 0:
            self._set_status('RETURNING',
                             f'back to anchor {np.degrees(err):+.0f} deg')
        else:
            side_s = 'L' if side_tag > 0 else 'R'
            self._set_status('AVOIDING',
                             f'{mode} {side_s} {np.degrees(err):+.0f} deg, '
                             f'depth {chosen_depth:.2f} m')

    # ------------------------------------------------------------------
    # 输出
    # ------------------------------------------------------------------
    def _publish(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)
        self._last_sent = (vx, wz)
        # 只有**前进**才算真正脱离卡住; 纯原地旋转(TURN)不清零 —— err 在
        # fwd_align 边界抖动时 TURN/ARC 交替, TURN 帧 publish(0, wz) 一发
        # 就把 stuck_cnt 归零, 导致永远积累不到 STUCK_RELEASE(死锁根因之三).
        if abs(vx) > EPS_VEL:
            self.stuck_cnt = 0

    def _send_once(self, vel):
        """同一速度只发一次(零意图静默, 不 20Hz 刷屏零速)."""
        if self._last_sent == vel:
            return
        self._publish(*vel)

    # ------------------------------------------------------------------
    # 地图可视化
    # ------------------------------------------------------------------
    def _publish_map(self):
        c = self.cfg
        now = self.get_clock().now().nanoseconds * 1e-9
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = c.odom_frame
        grid.info.resolution = c.map_resolution
        grid.info.width = self.lmap.n_grid
        grid.info.height = self.lmap.n_grid
        grid.info.origin.position.x = self.lmap.grid_cx - c.map_size / 2
        grid.info.origin.position.y = self.lmap.grid_cy - c.map_size / 2
        grid.info.origin.orientation.w = 1.0
        grid.data = self.lmap.grid_data(now)
        self.map_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisGuardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Ctrl+C 后先零速 1s @20Hz, 确保底盘收到停车后再退出(防滑行).
        try:
            zero = Twist()
            for _ in range(20):
                node.pub.publish(zero)
                time.sleep(0.05)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
