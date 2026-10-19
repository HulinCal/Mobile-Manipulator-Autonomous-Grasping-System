#!/usr/bin/env python3
"""环境感知(1): 整车矩形轮廓碰撞原语.

设计要点(真机实测结论, 不可退回外接圆近似)
------------------------------------------
URDF 实测车体为 ``0.48 x 0.42 m`` 矩形。所有点均为**车体系**坐标
(x 向前、y 向左)。碰撞/净空判定一律沿"预计行进路线"逐位姿采样整车
矩形, 而不是用 R≈0.32m 的外接圆 —— 外接圆会把车体侧面(不在行进
路线上)的障碍也算成碰撞, 车在可通行的窄缝前抱死。

提供的原语
----------
- :meth:`RobotFootprint.forward_gap` 前方矩形带宽内的前缘净空;
- :meth:`RobotFootprint.disk_blocked` 外接圆阻塞(仅纯旋转急停用);
- :meth:`RobotFootprint.corridor_clearance` 直线走廊净空深度(选向初选);
- :meth:`RobotFootprint.rotation_blocked` 原地旋转矩形逐位姿扫掠;
- :meth:`RobotFootprint.rollout_clearance` 直/圆弧路径预估(候选指令验证);
- :meth:`RobotFootprint.kill_rollout` 紧击杀矩形路径预估(每帧硬急停).

使用方法
--------
>>> fp = RobotFootprint(cfg)
>>> fp.corridor_clearance(pts, heading=0.1)   # 沿 0.1rad 航向的净空(m)
>>> fp.rotation_blocked(pts, dtheta=0.5)     # 原地转 0.5rad 会不会碰
>>> fp.kill_rollout(pts, vx=0.1, wz=0.2)     # (无碰撞距离, 是否碰撞)

``pts`` 为 ``(N,2)`` numpy 数组或空数组; ``None``/空一律视为无障碍。
"""

import math

import numpy as np

from .geometry_utils import EPS_VEL


class RobotFootprint:
    """整车矩形轮廓 + 沿路线扫掠的碰撞原语集合(无状态、可全局共享)."""

    def __init__(self, cfg, logger=None):
        """:param cfg: :class:`~chassis_guard.config.GuardConfig`
        :param logger: 可选 rclpy logger(本类目前不打日志, 预留).
        """
        self.cfg = cfg
        self.logger = logger
        # 规划矩形(含裕量)
        self.half_l = cfg.half_l
        self.half_w = cfg.half_w
        # 击杀矩形(贴物理车体)
        self.kill_half_l = cfg.kill_half_l
        self.kill_half_w = cfg.kill_half_w
        # 纯旋转急停用的外接圆半径
        self.disk_r = cfg.disk_r

    # ------------------------------------------------------------------
    # 直线/静止几何
    # ------------------------------------------------------------------
    def forward_gap(self, pts, ahead: float) -> float:
        """前方轮廓净空: 车体矩形宽度内最近障碍点到**前缘**的距离(m).

        :param ahead: 相对车体中心的检查距离; 最近点 x >= ahead 视为
            量程内无障碍(太远不关心). 调用方一般传
            ``half_l + emergency_dist``, 表示只查前缘外 emergency_dist 内。
        :returns: 净空米数; 无点/带宽内无障碍返回 ``inf``。
        """
        if pts is None or len(pts) == 0:
            return float('inf')
        band = pts[(np.abs(pts[:, 1]) <= self.half_w)
                   & (pts[:, 0] > -self.half_l)]
        if len(band) == 0:
            return float('inf')
        xm = float(band[:, 0].min())
        if xm < ahead:
            return xm - self.half_l
        return float('inf')

    def disk_blocked(self, pts) -> bool:
        """纯旋转阻塞判据: 车体外接圆(含裕量)内出现任意点即 True.

        仅用于**纯原地旋转**指令(前进/弧线指令禁止用它, 否则误杀).
        """
        if pts is None or len(pts) == 0:
            return False
        return bool(np.hypot(pts[:, 0], pts[:, 1]).min() < self.disk_r)

    def corridor_clearance(self, pts, heading: float) -> float:
        """沿 heading 直线行驶的走廊净空深度(相对车体前缘, m).

        把点云旋转到候选航向下: ``|y'| < half_w 且 x' > -half_l`` 的
        带宽内最近点到前缘的距离即净空。落在车体侧面(不在走廊内)的
        障碍被正确排除。返回 ``inf`` 表示走廊内无障碍。
        """
        if pts is None or len(pts) == 0:
            return float('inf')
        c, s = math.cos(heading), math.sin(heading)
        xr = c * pts[:, 0] + s * pts[:, 1]
        yr = -s * pts[:, 0] + c * pts[:, 1]
        band = xr[(np.abs(yr) < self.half_w) & (xr > -self.half_l)]
        if len(band) == 0:
            return float('inf')
        return max(0.0, float(band.min()) - self.half_l)

    def rotation_blocked(self, pts, dtheta: float, step: float = None,
                         half_l: float = None, half_w: float = None) -> bool:
        """原地旋转 dtheta 的扫掠路径上是否碰到点(不后退脱困的安全检查).

        沿旋转路径按 ``step``(默认 veer_step=5°)逐位姿扫掠矩形, 任一位姿
        有点落入轮廓即 True。比外接圆精确: 车头正前方 0.25m 的点在车头
        转到约 ±16° 后才进入轮廓, 本函数能正确放行小角度重选方向。

        :param half_l/half_w: 覆盖默认规划矩形半长/半宽。**原地脱困转圈
            时必须传物理车体尺寸**(robot_half_length/width): 默认矩形含
            0.02 安全裕量, 车停在离障碍 0.02m 处时车身"已经"贴在矩形边
            界上, 任何微小转动都被判定 blocked -> 脱困状态机永久 hold 转
            不动(实测现象)。物理矩形才是真实碰撞边界。
        """
        hl = self.half_l if half_l is None else half_l
        hw = self.half_w if half_w is None else half_w
        if pts is None or len(pts) == 0 or abs(dtheta) < 1e-3:
            return False
        st = self.cfg.veer_step if step is None else step
        n = int(abs(dtheta) / st) + 1
        for i in range(1, n + 1):
            th = math.copysign(i * st, dtheta)
            c, s = math.cos(th), math.sin(th)
            xr = c * pts[:, 0] + s * pts[:, 1]
            yr = -s * pts[:, 0] + c * pts[:, 1]
            if np.any((np.abs(xr) < hl) & (np.abs(yr) < hw)):
                return True
        return False

    # ------------------------------------------------------------------
    # 路径预估(直线/圆弧逐位姿积分 + 矩形扫掠)
    # ------------------------------------------------------------------
    def rollout_clearance(self, pts, vx: float, wz: float, horizon: float,
                          half_l: float = None, half_w: float = None):
        """规划矩形的圆弧路径预估: (无碰撞行驶距离 m, 是否碰撞).

        沿 (vx, wz) 预计轨迹按 0.05m 空间步长积分, 每个位姿把点云变换
        到车体系检查矩形相交。弧线行驶时车尾内切/车头外摆这类横向碰撞
        直线走廊(:meth:`corridor_clearance`)看不出, 由本函数兜底。
        可传 ``half_l/half_w`` 改用更紧的击杀矩形(控制循环最终验证时
        就是这样用的)。
        """
        if pts is None or len(pts) == 0 or abs(vx) < EPS_VEL:
            return float('inf'), False
        hl = self.half_l if half_l is None else half_l
        hw = self.half_w if half_w is None else half_w
        ds = 0.05  # m, 空间步长远小于车长, 相邻矩形必然重叠无缝隙
        x = y = th = 0.0
        dist = 0.0
        while dist < horizon:
            c, s = math.cos(th), math.sin(th)
            dx, dy = pts[:, 0] - x, pts[:, 1] - y
            xr = c * dx + s * dy
            yr = -s * dx + c * dy
            if np.any((np.abs(xr) < hl) & (np.abs(yr) < hw)):
                return dist, True
            th += wz / vx * ds
            x += ds * math.cos(th)
            y += ds * math.sin(th)
            dist += ds
        return float('inf'), False

    def kill_rollout(self, pts, vx: float, wz: float,
                     horizon: float = None):
        """击杀矩形路径预估(每帧硬急停用): (无碰撞距离 m, 是否碰撞).

        与 :meth:`rollout_clearance` 的区别:
        - 矩形更紧(physical+0.02), 侧面留 3-5cm 余量时不误判;
        - 必须显式传入**守卫实际发布**的速度 (vx, wz)(绕行原地 TURN 时
          实际 vx=0, 不能拿用户意图 vx 预估前进, 否则原地转向被误急停);
        - ``horizon`` 默认 = 制动距离 + 急停缓冲(且不小于 0.5*s)。
        """
        if pts is None or len(pts) == 0 or abs(vx) < EPS_VEL:
            return float('inf'), False
        ds = 0.05
        x = y = th = 0.0
        dist = 0.0
        if horizon is None:
            k = self.cfg.speed_scale(vx)
            horizon = max(vx * self.cfg.brake_time
                          + self.cfg.emergency_dist * k,
                          0.5 * k)
        while dist < horizon:
            c, s = math.cos(th), math.sin(th)
            dx, dy = pts[:, 0] - x, pts[:, 1] - y
            xr = c * dx + s * dy
            yr = -s * dx + c * dy
            if np.any((np.abs(xr) < self.kill_half_l)
                      & (np.abs(yr) < self.kill_half_w)):
                return dist, True
            if abs(vx) > 1e-6:
                th += wz / vx * ds
            x += ds * math.cos(th)
            y += ds * math.sin(th)
            dist += ds
        return float('inf'), False
