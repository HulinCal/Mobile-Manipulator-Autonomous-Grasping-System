#!/usr/bin/env python3
"""运动控制律: 速度自适应前向蠕行(ARC)/原地转向(TURN)/转向看门狗.

绕行选向得到目标航向偏差 ``err``(车体系, rad)后, 节点按
``|err| <= fwd_align`` 分两种执行方式:

- **ARC 弧线蠕行** (:func:`arc_command`): 每帧目标偏差最多递增一个
  veer_step(5°, 禁止一次大弧线甩头); 线速度按偏差大小衰减(保底 0.3
  倍意图速度), 角速度用 steer_kp 比例并限幅 max_steer_w;
- **TURN 原地转向** (:func:`turn_rate` + :class:`InPlaceTurn`): vx=0,
  角速度限速 0.15rad/s(EKF 航向滞后, 快转会转过头再回摆); 带看门狗:
  指令转角明显超过初始偏差时停 1s 等 TF 收敛; 总时长超 max_rotate_time
  判 BLOCKED, 由节点闭锁零速直到锚点方向畅通/新命令到来。

使用方法
--------
>>> if abs(err) <= cfg.fwd_align:
...     vx, wz, err = arc_command(cfg, intent_vx, err)
... else:
...     turn.begin_if_needed(now, err)
...     flag = turn.tick(now, err)     # 'ok'/'settle'/'watchdog'/'timeout'
...     if flag == 'ok':
...         wz = turn_rate(cfg, err); turn.accumulate(wz, dt)
"""

# tick() 返回: 可以正常发转向 / 等 TF 收敛(零速) / 看门狗暂停(零速) / 超时
TICK_OK = 'ok'
TICK_SETTLE = 'settle'
TICK_WATCHDOG = 'watchdog'
TICK_TIMEOUT = 'timeout'

# 绕行原地转向限速(rad/s): 实测 max_steer_w 偏快 + TF 滞后容易转过头
BYPASS_TURN_W = 0.15


def arc_command(cfg, intent_vx: float, err: float):
    """ARC 弧线蠕行控制律.

    :returns: ``(vx, wz, err_clamped)``。err 先限幅到 ±veer_step(分步
        递增逼近锁定航向); vx 随偏差衰减(>=0.3*意图); wz = P 控制并
        限幅 max_steer_w(增益不随速度放大)。
    """
    err = max(-cfg.veer_step, min(cfg.veer_step, err))
    vx = intent_vx * max(0.3, 1.0 - abs(err) / cfg.fwd_align)
    wz = max(-cfg.max_steer_w,
             min(cfg.max_steer_w, cfg.steer_kp * err))
    return vx, wz, err


def turn_rate(cfg, err: float) -> float:
    """TURN 原地转向角速度: steer_kp*err, 限速 min(max_steer_w, 0.15)."""
    w = min(cfg.max_steer_w, BYPASS_TURN_W)
    return max(-w, min(w, cfg.steer_kp * err))


class InPlaceTurn:
    """原地转向看门狗状态(每段 TURN 一组计时/积分)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.rotate_since = None        # 本段转向起始时刻(s)
        self.rotate_cmd = 0.0           # 本段指令转角积分(rad)
        self.rotate_err0 = 0.15         # 本段起始 |err|
        self.rotate_wait_until = 0.0    # TF 收敛等待截止时刻(s)

    def reset(self):
        """新命令/转到位/超时时清零."""
        self.rotate_since = None

    def begin_if_needed(self, now: float, err: float):
        """本段 TURN 首帧初始化计时(后续帧保持)."""
        if self.rotate_since is None:
            self.rotate_since = now
            self.rotate_cmd = 0.0
            self.rotate_err0 = max(abs(err), 0.15)
            self.rotate_wait_until = 0.0

    def tick(self, now: float, err: float) -> str:
        """每帧检查, 返回 TICK_* 状态(节点据此决定零速等待还是继续转).

        - ``settle``: 处于看门狗触发后的 1s TF 等待窗内;
        - ``watchdog``: 指令累计转角已超初始偏差*1.3+0.15, 本帧起停转
          1s 等 TF 收敛(内部已重置积分/等待窗);
        - ``timeout``: 本段转向超过 max_rotate_time(节点应闭锁并清承诺);
        - ``ok``: 可按 :func:`turn_rate` 发转向, 发完调 :meth:`accumulate`。
        """
        if now < self.rotate_wait_until:
            return TICK_SETTLE
        if self.rotate_cmd > self.rotate_err0 * 1.3 + 0.15:
            self.rotate_cmd = 0.0
            self.rotate_err0 = max(abs(err), 0.15)
            self.rotate_wait_until = now + 1.0
            return TICK_WATCHDOG
        if now - self.rotate_since > self.cfg.max_rotate_time:
            return TICK_TIMEOUT
        return TICK_OK

    def accumulate(self, wz: float, dt: float):
        """记录本帧实际下发的转角(dt=1/publish_hz)."""
        self.rotate_cmd += abs(wz) * dt
