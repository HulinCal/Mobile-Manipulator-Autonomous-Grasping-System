#!/usr/bin/env python3
"""选择绕行方向: VFH-lite 极向走廊选向(纯函数集合, 无内部状态).

节点的前向避障状态机(承诺/锁定/回正/换侧仍在节点里)每帧用本类
回答三类问题:

1. :meth:`BypassPlanner.best_option` —— 从锚点(原始世界航向)起, 向
   指定侧按 veer_step(5°)**逐步递增**探测, 第一个整条走廊净空 >
   lookahead 的方向即选定(偏转最小可行方向);
2. :meth:`BypassPlanner.search_all` —— ±max_veer(80°)内无走廊时,
   在前向半球 ±90° 内从 0° 向两侧取偏转最小的可通行方向;
   **绝不选 |h|>90°**(那等于掉头沿来路后退, 用户禁止);
3. :meth:`BypassPlanner.obstacle_side` / :meth:`parallel_passed` ——
   障碍质心偏左/偏右(决定绕哪侧)与"车体中心是否已与障碍平行"
   (平行前锁定航向, 平行后才允许回正)。

净空/深度语义
-------------
- ``clear``: 该航向整车矩形走廊净空 > lookahead(速度自适应前瞻);
- ``depth``: 净空截断到 scan_max_range(1.5m)——开阔方向(inf)与量程
  尽头公平比较, 否则 inf 恒赢;
- 走廊净空本身已用车体矩形保证该航向上 lookahead 内无碰撞, 弧线
  过渡的安全由路径预估急停(:mod:`chassis_guard.safety`/footprint)兜底。

使用方法
--------
>>> planner = BypassPlanner(cfg, fp)
>>> look = cfg.lookahead(vx_cmd)
>>> planner.best_option(pts, anchor_rel, side=+1, lookahead=look)
>>> planner.search_all(pts, look)
"""

import math

import numpy as np


class BypassPlanner:
    """走廊选向原语(无状态; 点云均为车体系 (N,2))."""

    def __init__(self, cfg, footprint, logger=None):
        """:param cfg: GuardConfig
        :param footprint: RobotFootprint(走廊净空计算).
        """
        self.cfg = cfg
        self.fp = footprint
        self.logger = logger
        self.max_steps = int(round(cfg.max_veer / cfg.veer_step))

    # ------------------------------------------------------------------
    # 基础判据
    # ------------------------------------------------------------------
    def is_clear(self, pts, heading: float, lookahead: float) -> bool:
        """沿 heading 的走廊净空 > lookahead."""
        return self.fp.corridor_clearance(pts, heading) > lookahead

    def depth(self, pts, heading: float) -> float:
        """沿 heading 的走廊深度, 截断到 scan_max_range(m)."""
        return min(self.fp.corridor_clearance(pts, heading),
                   self.cfg.scan_max_range)

    # ------------------------------------------------------------------
    # 选向
    # ------------------------------------------------------------------
    def best_option(self, pts, anchor_rel: float, side: int,
                    lookahead: float):
        """承诺侧的可行航向(比"最小可行"再多让一个 veer_step).

        从锚点方向起向 ``side``(+1 左 / -1 右)按 veer_step 递增, 第一个
        净空 > lookahead 的方向为**候选**; 再向该侧多走一个 veer_step 作为
        **返回值** —— ARC 弧线绕行的实际轨迹比直线走廊更宽(车头外摆/车尾
        内切), 用户实测"转弯太小, 贴着障碍边缘走会触发脱困", 多让一步即
        留出额外净空. 多让那一步仍限制在 max_veer 内, 且**该方向确实更
        畅通**(净空 > lookahead)时才采用, 否则退回候选项(不朝窄墙多转).

        :returns: ``(深度 d, 车体系航向 h, 偏转扇区数 k)``; 该侧
            ±max_veer 内无可行方向返回 ``None``。
        """
        for k in range(1, self.max_steps + 1):
            h = anchor_rel + side * k * self.cfg.veer_step
            if abs(h) > self.cfg.max_veer + 1e-6:
                break
            d = self.depth(pts, h)
            if d <= lookahead:
                continue
            # 多加一个 veer_step: 留足弧线绕行净空(不越 max_veer)
            if k < self.max_steps:
                h2 = anchor_rel + side * (k + 1) * self.cfg.veer_step
                if abs(h2) <= self.cfg.max_veer + 1e-6:
                    d2 = self.depth(pts, h2)
                    if d2 > lookahead:
                        return (d2, h2, k + 1)
            return (d, h, k)
        return None

    def search_all(self, pts, lookahead: float):
        """前向半球 ±90° 搜索可通行走廊(承诺侧封闭时的兜底).

        从 0° 起按 sector_res(5°)向两侧递增, 取偏转最小的可通行方向。
        :returns: ``(深度, 车体系航向)`` 或 ``None``(前向半球无路,
            节点应零速抱住等新命令, 绝不后退/掉头)。
        """
        n_quarter = int(round((math.pi / 2) / self.cfg.sector_res))
        for k in range(1, n_quarter + 1):
            h = k * self.cfg.sector_res
            if self.is_clear(pts, h, lookahead):
                return (self.depth(pts, h), h)
            if self.is_clear(pts, -h, lookahead):
                return (self.depth(pts, -h), -h)
        return None

    # ------------------------------------------------------------------
    # 障碍方位 / 平行判定
    # ------------------------------------------------------------------
    def obstacle_side(self, pts) -> int:
        """正前方近障质心偏哪侧 -> 应绕的对侧.

        只看 x∈(0.2, 1.5)m、|y|<0.6m 的障碍点 y 中位数:
        障碍偏左(median>0.05)返回 -1(向右绕); 偏右返回 +1(向左绕);
        点数不足或居中返回 0(节点改用两侧深度更深者)。
        """
        front = pts[(pts[:, 0] > 0.2) & (pts[:, 0] < 1.5)
                    & (np.abs(pts[:, 1]) < 0.6)]
        if len(front) < 2:
            return 0
        obs_y = float(np.median(front[:, 1]))
        if obs_y > 0.05:
            return -1
        if obs_y < -0.05:
            return +1
        return 0

    def parallel_passed(self, pts, d_par: float) -> bool:
        """车体中心是否已与原航线走廊内的障碍平行/越过.

        用户原话判据: "小车中心点与障碍物在同一个 x 值的位置"。把障碍
        点换算到锚点系(原航线方向, d_par = yaw - anchor_yaw): 原航线
        走廊内(|y_a|<half_w+0.06)、车中心之前(x_a>parallel_x)已无点即
        视为平行。不能用车体系方位角中位数(侧旁 90° 点与远处 5° 点
        混在一起会永远判不出平行)。

        :returns: True=已平行(可开始计回正); False=障碍还在前方,
            航向必须继续锁定。点不足(<2)按已平行处理(沿用原行为)。
        """
        front = pts[(pts[:, 0] > -0.35) & (pts[:, 0] < 1.5)
                    & (np.abs(pts[:, 1]) < 0.6)]
        if len(front) < 2:
            return True
        xa = (math.cos(d_par) * front[:, 0]
              - math.sin(d_par) * front[:, 1])   # 锚点系纵向(前+)
        ya = (math.sin(d_par) * front[:, 0]
              + math.cos(d_par) * front[:, 1])   # 锚点系横向
        in_lane = np.abs(ya) < self.cfg.half_w + 0.06
        ahead = xa[in_lane & (xa > self.cfg.parallel_x)]
        return len(ahead) == 0
