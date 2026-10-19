#!/usr/bin/env python3
"""集中式参数容器与速度自适应公式.

功能
----
- :class:`GuardConfig` 持有 chassis_guard 的**全部可调参数**以及由它们
  派生的几何量(整车矩形半长/半宽、急停击杀矩形、旋转外接圆半径、
  直方图扇区角分辨率). 各功能模块(感知/选向/脱困/控制)只依赖本对象,
  不直接接触 ROS 参数服务器, 便于脱离节点做单元测试;
- 速度自适应三件套(用户标定逻辑, 全节点统一入口, 禁止各模块各算一遍):
  * :meth:`GuardConfig.speed_scale`  —— s = vx/tuned_vx, 下限 1;
  * :meth:`GuardConfig.lookahead`    —— 走廊选向/脱困搜向的净空前瞻;
  * :meth:`GuardConfig.stop_distance`—— 路径预估急停的实际停距.

使用方法
--------
节点在 ``declare_parameter`` 之后用参数读取函数一次性构造::

    cfg = GuardConfig.from_params(lambda n: self.get_parameter(n).value)
    cfg.half_l          # 派生量可直接用
    cfg.lookahead(0.10) # vx=0.10(2x) -> 0.60 m

纯函数测试::

    GuardConfig(robot_half_length=0.24, ...).speed_scale(0.10)  # 2.0
"""

import math
from dataclasses import dataclass, field


@dataclass
class GuardConfig:
    # ---- 话题/坐标系 ---------------------------------------------------
    cmd_in_topic: str = '/cmd_vel_in'
    cmd_topic: str = '/cmd_vel'
    scan_topic: str = '/scan'
    vision_topic: str = '/guard/vision_obstacles'
    map_topic: str = '/guard/local_map'
    status_topic: str = '/guard/status'
    pick_state_topic: str = '/pick_place/state'
    odom_frame: str = 'odom_combined'
    robot_frame: str = 'base_footprint'

    # ---- 周期 ----------------------------------------------------------
    publish_hz: float = 20.0
    map_hz: float = 2.0

    # ---- 局部动态地图 --------------------------------------------------
    map_size: float = 6.0            # m, 正方形滚动栅格边长
    map_resolution: float = 0.05     # m/格
    shift_thresh: float = 1.0        # m, 位移超过该值栅格窗口才滚动
    lidar_timeout: float = 5.0       # s, 激光障碍记忆时长
    vision_timeout: float = 2.0      # s, 视觉障碍记忆时长

    # ---- 雷达 ----------------------------------------------------------
    scan_max_range: float = 1.5      # m, 只关心近处, 深度也截断到该值
    scan_beams: int = 120            # 每帧写地图最多使用的光束数

    # ---- 整车矩形轮廓(URDF 实测 + 安全裕量) ----------------------------
    robot_half_length: float = 0.24
    robot_half_width: float = 0.21
    safety_margin: float = 0.02

    # ---- 避障行为 ------------------------------------------------------
    brake_time: float = 2.0          # s, 制动距离 = vx*brake_time
    min_clearance: float = 0.3       # m, 标定速度下的最小通道净空
    emergency_dist: float = 0.10     # m, 轮廓外扩急停距离
    emergency_clear: float = 0.20    # m, 急停锁存滞回解除距离
    tuned_vx: float = 0.05           # m/s, 标定速度(速度自适应分母)
    align_tol: float = 0.26          # rad, ~15°, 回正完成容差
    fwd_align: float = 0.9           # rad, 该偏航内走弧线 ARC, 否则原地 TURN
    max_veer: float = 1.4            # rad ~80°, 绕行最大偏航
    side_switch_margin: float = 0.25
    return_cycles: int = 6           # 锚点方向连续 N 周期空才回正
    veer_step: float = 0.08726646    # rad (5°), 选向/限幅步进
    parallel_x: float = 0.10         # m, 平行判定的纵向阈值
    steer_kp: float = 1.2            # 航向 P 增益
    max_steer_w: float = 0.2         # rad/s, 最大转向角速度
    n_sectors: int = 72              # 极向直方图扇区数
    max_rotate_time: float = 20.0    # s, 原地转向总时长上限
    rescue_turn_tol: float = 0.05    # rad, 脱困转向到位容差(< veer_step)
    scan_timeout: float = 0.5        # s, 雷达超时视为失效

    # ---- 任务旁路(pick_place 进行中的状态集合) -------------------------
    bypass_states: set = field(default_factory=set)

    # ---- 派生量(__post_init__ 计算, 不来自参数服务器) ------------------
    half_l: float = 0.0              # 规划矩形半长 = physical + margin
    half_w: float = 0.0
    kill_half_l: float = 0.0         # 急停击杀矩形半长 = physical + 0.02
    kill_half_w: float = 0.0
    disk_r: float = 0.0              # 规划矩形外接圆半径
    sector_res: float = 0.0          # 直方图扇区角分辨率 = 2pi/n_sectors

    def __post_init__(self):
        self.tuned_vx = max(float(self.tuned_vx), 0.01)
        # 规划矩形(含安全裕量): 选向/走廊判定, 保持保守
        self.half_l = self.robot_half_length + self.safety_margin
        self.half_w = self.robot_half_width + self.safety_margin
        # 击杀矩形(物理车体 + 0.02 微缓冲): 只拦即将真实接触的障碍,
        # 侧面 3-5cm 缝隙不误判
        self.kill_half_l = self.robot_half_length + 0.02
        self.kill_half_w = self.robot_half_width + 0.02
        self.disk_r = math.hypot(self.half_l, self.half_w)
        self.sector_res = 2.0 * math.pi / self.n_sectors

    # ------------------------------------------------------------------
    # 速度自适应(唯一实现处): s=vx/tuned_vx, 下限 1
    # ------------------------------------------------------------------
    def speed_scale(self, vx: float) -> float:
        """速度比例 s: 前瞻/急停缓冲等固定距离按它随速度放大."""
        return max(1.0, abs(vx) / self.tuned_vx)

    def lookahead(self, vx: float) -> float:
        """走廊净空前瞻(m): max(最小净空*s, 制动距离)."""
        s = self.speed_scale(vx)
        return max(self.min_clearance * s, abs(vx) * self.brake_time)

    def stop_distance(self, vx: float) -> float:
        """实际停距(m) = 制动距离 + 急停缓冲*s; rollout 碰撞距离小于它才急停."""
        s = self.speed_scale(vx)
        return abs(vx) * self.brake_time + self.emergency_dist * s

    def emergency_stop_distance(self, vx: float) -> float:
        """**紧急停距离**(m) = 急停缓冲*s, 不含制动距离.

        绕行(commit_side != 0)时用它作触发阈值: 用户要求"只要行进路径上
        没有障碍物就可以前进, 不能随便触发脱困" —— 弧线绕行的过渡帧里
        路径常贴着障碍(碰撞距离 0.2~0.3m), 用完整停距会在让开障碍前就
        误判急停. 本阈值只在几乎真正接触时才触发脱困, 满足"路径无碰撞
        就继续走".
        """
        return self.emergency_dist * self.speed_scale(vx)

    # ------------------------------------------------------------------
    # 构造
    # ------------------------------------------------------------------
    @classmethod
    def from_params(cls, getter):
        """从 ROS 参数读取函数构造(节点已 declare 全部参数后调用).

        :param getter: ``getter(name) -> 参数值``, 节点传
            ``lambda n: self.get_parameter(n).value``.
        """
        def f(name, cast=str):
            return cast(getter(name))

        return cls(
            cmd_in_topic=f('cmd_vel_in_topic'),
            cmd_topic=f('cmd_vel_topic'),
            scan_topic=f('scan_topic'),
            vision_topic=f('vision_obstacle_topic'),
            map_topic=f('map_topic'),
            status_topic=f('status_topic'),
            pick_state_topic=f('pick_state_topic'),
            odom_frame=f('odom_frame'),
            robot_frame=f('robot_frame'),
            publish_hz=f('publish_hz', float),
            map_hz=f('map_hz', float),
            map_size=f('map_size', float),
            map_resolution=f('map_resolution', float),
            shift_thresh=f('map_shift_threshold', float),
            lidar_timeout=f('lidar_obstacle_timeout', float),
            vision_timeout=f('vision_obstacle_timeout', float),
            scan_max_range=f('scan_max_range', float),
            scan_beams=f('scan_beams', int),
            robot_half_length=f('robot_half_length', float),
            robot_half_width=f('robot_half_width', float),
            safety_margin=f('safety_margin', float),
            brake_time=f('brake_time', float),
            min_clearance=f('min_clearance', float),
            emergency_dist=f('emergency_dist', float),
            emergency_clear=f('emergency_clear', float),
            tuned_vx=f('tuned_vx', float),
            align_tol=f('align_tolerance', float),
            fwd_align=f('forward_align_angle', float),
            max_veer=f('max_veer_angle', float),
            side_switch_margin=f('side_switch_margin', float),
            return_cycles=f('return_confirm_cycles', int),
            veer_step=math.radians(f('veer_step_deg', float)),
            parallel_x=f('parallel_x', float),
            steer_kp=f('steer_kp', float),
            max_steer_w=f('max_steer_w', float),
            n_sectors=f('hist_sectors', int),
            max_rotate_time=f('max_rotate_time', float),
            rescue_turn_tol=f('rescue_turn_tol', float),
            scan_timeout=f('scan_timeout', float),
            bypass_states=set(str(s) for s in getter('bypass_states')),
        )
