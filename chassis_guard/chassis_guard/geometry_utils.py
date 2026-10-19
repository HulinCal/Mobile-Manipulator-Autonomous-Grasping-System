#!/usr/bin/env python3
"""几何基础工具(无 ROS 依赖, 可被任意模块/单测直接 import).

功能
----
- :data:`EPS_VEL`: 速度判零阈值(rad/s 或 m/s, 数值上通用);
- :data:`ZERO_VEL`: 零速指令 ``(vx, wz) = (0, 0)``;
- :func:`wrap_angle`: 角度归一化到 ``[-pi, pi]``(标量或 numpy 数组);
- :func:`yaw_from_quat`: 四元数 -> 2D 偏航角(rad).

使用方法
--------
>>> from chassis_guard.geometry_utils import wrap_angle, yaw_from_quat
>>> wrap_angle(3.5)        # -> -2.783...
>>> yaw_from_quat(q)       # q: geometry_msgs/Quaternion 形状(w,x,y,z)
"""

import math

import numpy as np

# 速度小于该值视为零(线速度 m/s / 角速度 rad/s 共用同一数值阈值)
EPS_VEL = 1e-3

# 零速 (vx, wz)
ZERO_VEL = (0.0, 0.0)


def wrap_angle(a):
    """归一化角度到 [-pi, pi]; 接受标量或 numpy 数组."""
    return np.arctan2(np.sin(a), np.cos(a))


def yaw_from_quat(q) -> float:
    """四元数 -> 偏航角(rad, [-pi, pi]).

    q 需要具备 w/x/y/z 属性(geometry_msgs.msg.Quaternion 或同构对象).
    """
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
