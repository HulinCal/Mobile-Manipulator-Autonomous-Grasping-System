#!/usr/bin/env python3
"""脱困策略: 原地**逆时针(CCW)**转动搜向, 找到第一条可通行路径即前进.

启动门控(由节点判定, 三个条件全满足才调用 :meth:`RescueController.enter`)
--------------------------------------------------------------------------------
1. **不在绕行阶段**(未承诺绕行方向 / 已越过障碍物) —— 绕行本身就能绕开
   障碍时不需要脱困;
2. **视野内没有可通行的路线** —— 前向 ±90° 内不存在整条净空 > lookahead
   的走廊(节点用 :meth:`BypassPlanner.search_all` 判定);
3. 满足 1、2 才启动本状态机原地转圈找路。
条件不满足时节点按普通避障处理(选向绕行 / 直行), 绝不轻易脱困。

启动后的行为(用户规格)
------------------------
- 原地**匀速逆时针**转动(z ``wz > 0``, ROS 右手系: 车头向左转), 每帧
  检查**当前车头朝向**的走廊净空; 一旦净空 > lookahead 即判定"找到第一
  条可通行路径", 立即停止转动并返回 ``done``, 节点以当前车头方向前进;
- **中途绝不停车**: 转满一圈、旋转扫掠受阻、雷达点很近……都不许中断
  转圈, 一直转到找到可通行方向为止(真机实测: 任何一次中途停车都会让小
  车原地不动)。整车异常时只打印提示日志, 不改状态。
- 绝不后退/掉头: 只用原地旋转 + 前进两种运动。

本状态机**不发布速度、不改节点锚点**, 只返回 :class:`RescueOutcome`,
由节点执行副作用, 便于脱离 ROS 单测。

使用方法
--------
>>> rescue = RescueController(cfg, fp, publish_hz, logger)
>>> rescue.enter()                       # 门控通过后(节点已先零速)
>>> look = cfg.lookahead(intent_vx)      # 每帧前瞻
>>> out = rescue.step(pts, look)
>>> if out.kind == 'spin':   node._publish(0.0, out.wz)   # 逆时针转
>>> elif out.kind == 'done': node.resume_from_rescue()    # 以当前车头前进
"""

import math
from dataclasses import dataclass

# 逆时针搜向角速度(rad/s): 与绕行转向限速一致, 太快会因 EKF 滞后转过头
CCW_RATE = 0.15
# 整圈角度: 用于"已转满 N 圈仍无路"的提示(不用于停车, 脱困只由找到路结束)
FULL_TURN = 2.0 * math.pi


@dataclass
class RescueOutcome:
    """脱困状态机单帧决策."""
    kind: str                # 'spin' | 'hold' | 'done'
    wz: float = 0.0          # spin: 逆时针角速度(>0)
    status: str = None       # 要显示的状态文本; None = 不改变状态灯
    heading: float = 0.0     # done: 找到的车体系航向(0 = 当前车头方向)


class RescueController:
    """脱困转圈状态机(持有转动进度)."""

    def __init__(self, cfg, footprint, publish_hz: float, logger=None):
        self.cfg = cfg
        self.fp = footprint
        self.publish_hz = publish_hz
        self.logger = logger
        self.state = ''          # '' / 'turn' / 'hold'
        self.spun = 0.0          # 本次脱困已累计转角(rad)

    # ------------------------------------------------------------------
    # 外部事件
    # ------------------------------------------------------------------
    def reset(self):
        """新外部命令到来: 清空脱困进度(节点命令回调中调用)."""
        self.state = ''
        self.spun = 0.0

    def enter(self):
        """启动脱困(三条件门控已在节点侧判定通过, 节点已先发零速)."""
        self.state = 'turn'
        self.spun = 0.0

    def complete(self):
        """转到位、节点已恢复行驶后调用(收尾)."""
        self.reset()

    # ------------------------------------------------------------------
    # 每帧状态机
    # ------------------------------------------------------------------
    def step(self, pts, lookahead: float) -> RescueOutcome:
        """推进一帧: 原地逆时针转圈, 直到当前车头方向可通行才停转.

        :param pts: 车体系雷达点 (N,2)
        :param lookahead: 走廊净空前瞻(m), 由节点按意图速度算好传入

        用户真机要求: **脱困转圈中途绝不停车** —— 单帧"旋转扫掠受阻"或
        "已转满一圈"都不许中断, 一旦中断小车就停在原地不动(实测多次).
        故本状态机只有两个出口:
          * 当前车头方向净空 > lookahead -> 'done' 停转并前进;
          * 否则 -> 'spin' 继续逆时针转。
        转动过程中不做碰撞否决(原地旋转是唯一脱困手段, 且绝不后退)。
        """
        if self.state != 'turn':
            return RescueOutcome('hold', status='rescue: idle')

        # 当前车头朝向已可通行: 找到第一条可通行路径, 停转前进
        d = self.fp.corridor_clearance(pts, 0.0)
        if d > lookahead:
            if self.logger is not None:
                self.logger.info(
                    f'rescue: passable heading found after '
                    f'{math.degrees(self.spun):.0f} deg CCW '
                    f'(depth {min(d, self.cfg.scan_max_range):.2f} m); '
                    'stop rotating, go forward')
            return RescueOutcome('done', heading=0.0)

        inc = CCW_RATE / self.publish_hz
        prev = self.spun
        self.spun += inc
        if self.logger is not None and self.spun >= FULL_TURN * (
                int(prev / FULL_TURN) + 1):
            # 已转满整圈仍无路: 继续转(不停止), 只提示
            self.logger.warn(
                f'rescue: {int(self.spun / FULL_TURN)} full CCW turn(s) '
                'without a passable heading; keep rotating')
        return RescueOutcome(
            'spin', wz=CCW_RATE,
            status=f'rescue: rotate CCW {math.degrees(self.spun):.0f} deg '
                   'searching path')
