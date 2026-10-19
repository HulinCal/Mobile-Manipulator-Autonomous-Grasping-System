#!/usr/bin/env python3
"""安全层: 每帧运动安全评估(是否触发硬急停 / 急停是否可解除).

雷达回调每帧用**守卫实际发布**的速度(而非用户意图)调用
:func:`evaluate_motion`, 得到 :class:`MotionSafety`:
- ``blocked``   —— 当前实际运动轨迹即将碰撞(距离小于实际停距),
  节点据此进入急停脱困(绝不后退, 只停车重选方向);
- ``clear_now`` —— 击杀矩形外 emergency_clear 内已无碰撞, 供急停
  锁存滞回解除计数(连续 3 帧才解除);
- ``front_gap`` —— 最近碰撞距离(m), 仅用于日志诊断。

分支语义(与真机调试后的行为严格一致)
------------------------------------
- 纯旋转(vx≈0, wz!=0): 外接圆内出现任意点即阻塞;
- 前进/弧行(vx!=0): 沿实际 (vx,wz) 做击杀矩形路径预估, 只有碰撞距离
  小于 ``制动距离 + 急停缓冲`` 才阻塞; horizon 边缘(如 0.5m)的碰撞
  不算紧急; 解除判据用固定短视野 emergency_clear;
- 完全静止: 永不触发。

使用方法
--------
>>> m = evaluate_motion(fp, pts, act_vx, act_wz, cfg)
>>> if moving and m.blocked: node.enter_rescue(...)
>>> elif m.clear_now: node.emergency_clear_cnt += 1
"""

from dataclasses import dataclass

import numpy as np

from .geometry_utils import EPS_VEL


@dataclass
class MotionSafety:
    """单帧运动安全评估结果."""
    blocked: bool          # 沿实际轨迹即将碰撞 -> 应进入急停
    clear_now: bool        # 短视野内已无碰撞 -> 急停可计一帧"已净空"
    front_gap: float       # 最近碰撞距离(m), 无障碍为 inf


def evaluate_motion(footprint, pts, act_vx: float, act_wz: float, cfg,
                    block_dist: float = None):
    """按实际发布速度评估本帧运动安全性.

    :param footprint: :class:`~chassis_guard.footprint.RobotFootprint`
    :param pts: 车体系 (N,2) 雷达点
    :param act_vx/act_wz: 守卫**实际发布**的速度(关键: 不是用户意图)
    :param cfg: GuardConfig
    :param block_dist: 触发阻塞的碰撞距离阈值(m); ``None`` 用完整刹车距离
        ``cfg.stop_distance(act_vx)``。绕行中(commit_side != 0)传
        ``cfg.emergency_stop_distance(act_vx)`` 收紧触发 —— 用户要求
        "行进路径上没有障碍物就可以前进", 弧线绕行的过渡帧路径贴边
        (0.2~0.3m)不应被当成脱困条件。
    """
    rotating = abs(act_wz) > EPS_VEL and abs(act_vx) < EPS_VEL
    if rotating:
        blocked = footprint.disk_blocked(pts)
        if pts is not None and len(pts):
            front_gap = float(np.hypot(pts[:, 0], pts[:, 1]).min())
        else:
            front_gap = float('inf')
        return MotionSafety(blocked, not blocked, front_gap)

    if abs(act_vx) > EPS_VEL:
        # 碰撞距离 < 触发阈值才触发; horizon 边缘的碰撞不是紧急情况
        limit = cfg.stop_distance(act_vx) if block_dist is None else block_dist
        gap, hit = footprint.kill_rollout(pts, vx=act_vx, wz=act_wz)
        blocked = hit and gap < limit
        front_gap = gap if hit else float('inf')
        # 滞回解除: 短视野 emergency_clear 内无碰撞才算一帧净空
        _, clear_blocked = footprint.kill_rollout(
            pts, vx=act_vx, wz=act_wz, horizon=cfg.emergency_clear)
        return MotionSafety(blocked, not clear_blocked, front_gap)

    # 完全静止: 不触发
    return MotionSafety(False, True, float('inf'))
