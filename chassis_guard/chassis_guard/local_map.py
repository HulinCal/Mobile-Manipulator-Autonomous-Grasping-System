#!/usr/bin/env python3
"""环境感知(2): 局部动态地图与传感器点云解析.

- :class:`LocalMap` —— odom_combined 系下的滚动占据栅格。雷达只覆盖
  车头前方, 侧后方障碍靠本地图**记忆**: 扫描线穿过的栅格清空、命中
  栅格标记占据并按超时衰减(动态障碍移走后自动失效); 栅格窗口随车
  滚动, 重叠区域保留时间戳记忆。
- :func:`parse_xyz` —— 最小化 PointCloud2 解析(不依赖 sensor_msgs_py),
  供视觉融合预留话题 ``/guard/vision_obstacles`` 使用。
- :func:`scan_to_xy` —— LaserScan -> 车体系 (N,2) 点云。

使用方法
--------
>>> m = LocalMap(cfg)
>>> m.initialize(ox, oy)                 # 首帧对齐地图中心
>>> m.ray_update(x0, y0, wx, wy, hit, now, cfg.lidar_timeout)
>>> m.recenter(rx, ry)                   # 控制周期里随车滚动
>>> pts = m.memory_points(rx, ry, yaw, now)   # 侧后盲区记忆障碍(车体系)
>>> data = m.grid_data(now)              # OccupancyGrid.data 取值
"""

import math

import numpy as np
from sensor_msgs.msg import PointField

_EPOCH = -1e9


class LocalMap:
    """滚动栅格地图的全部状态与读写原语(不接触 ROS, 可独立单测)."""

    def __init__(self, cfg, logger=None):
        """:param cfg: GuardConfig(地图尺寸/分辨率/超时/雷达量程等)."""
        self.cfg = cfg
        self.logger = logger
        self.map_size = cfg.map_size
        self.res = cfg.map_resolution
        self.n_grid = int(round(cfg.map_size / cfg.map_resolution))
        # 栅格中心(odom 系)与占据/清空时间戳
        self.grid_cx = 0.0
        self.grid_cy = 0.0
        self.occ_t = np.full((self.n_grid, self.n_grid), _EPOCH)
        self.clr_t = np.full((self.n_grid, self.n_grid), _EPOCH)
        self.occ_timeout = np.full((self.n_grid, self.n_grid),
                                   cfg.lidar_timeout)
        self.initialized = False

    # ------------------------------------------------------------------
    def initialize(self, ox: float, oy: float):
        """首帧把地图中心对齐到当前车位(防 odom 非原点导致点云落窗 外)."""
        if self.initialized:
            return
        self.grid_cx, self.grid_cy = ox, oy
        self.initialized = True

    # ------------------------------------------------------------------
    # 坐标变换 / 光线更新
    # ------------------------------------------------------------------
    def world_to_cell(self, wx: float, wy: float):
        """世界(odom)坐标 -> 栅格 (i 行=y, j 列=x); 越界返回 (None, None)."""
        j = int((wx - (self.grid_cx - self.map_size / 2)) / self.res)
        i = int((wy - (self.grid_cy - self.map_size / 2)) / self.res)
        if 0 <= i < self.n_grid and 0 <= j < self.n_grid:
            return i, j
        return None, None

    def ray_update(self, x0, y0, x1, y1, hit, now, timeout):
        """Bresenham 式光线更新: 穿过的栅格清空, 命中终点标记占据."""
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(dist / (self.res * 0.5)))
        for k in range(steps):
            t = k / steps
            i, j = self.world_to_cell(x0 + (x1 - x0) * t,
                                      y0 + (y1 - y0) * t)
            if i is not None:
                self.clr_t[i, j] = now
        if hit:
            i, j = self.world_to_cell(x1, y1)
            if i is not None:
                self.occ_t[i, j] = now
                self.occ_timeout[i, j] = timeout

    def recenter(self, rx: float, ry: float):
        """栅格窗口跟随机器人滚动, 重叠区域保留时间戳记忆.

        位移超过整张地图(手动推车导致 odom 跳变)时无重叠区域, 直接清空
        记忆 —— 不能切片拷贝, 否则维度不匹配崩溃。
        """
        if (abs(rx - self.grid_cx) < self.cfg.shift_thresh
                and abs(ry - self.grid_cy) < self.cfg.shift_thresh):
            return
        di = int(round((ry - self.grid_cy) / self.res))
        dj = int(round((rx - self.grid_cx) / self.res))
        if abs(di) >= self.n_grid or abs(dj) >= self.n_grid:
            self.grid_cx, self.grid_cy = rx, ry
            self.occ_t.fill(_EPOCH)
            self.clr_t.fill(_EPOCH)
            self.occ_timeout.fill(self.cfg.lidar_timeout)
            if self.logger is not None:
                self.logger.warn(
                    'Pose jump larger than local map; memory reset')
            return
        self.grid_cx += dj * self.res
        self.grid_cy += di * self.res
        for arr, fill in ((self.occ_t, _EPOCH), (self.clr_t, _EPOCH),
                          (self.occ_timeout, self.cfg.lidar_timeout)):
            shifted = np.full_like(arr, fill)
            src_i = slice(max(0, -di), min(self.n_grid, self.n_grid - di))
            dst_i = slice(max(0, di), min(self.n_grid, self.n_grid + di))
            src_j = slice(max(0, -dj), min(self.n_grid, self.n_grid - dj))
            dst_j = slice(max(0, dj), min(self.n_grid, self.n_grid + dj))
            shifted[dst_i, dst_j] = arr[src_i, src_j]
            arr[:] = shifted

    # ------------------------------------------------------------------
    # 记忆 -> 车体系障碍点
    # ------------------------------------------------------------------
    def memory_points(self, rx: float, ry: float, yaw: float, now: float):
        """当前未超时占据栅格中心 -> 车体系 (N,2) 点(补雷达侧后盲区).

        占据条件: 占据时间戳新于清空时间戳且未超过各自超时; 距离裁剪
        到 scan_max_range 内; 剔除原点附近数值噪声。
        """
        occ = (self.occ_t > self.clr_t) & \
              (now - self.occ_t < self.occ_timeout)
        if not occ.any():
            return np.zeros((0, 2))
        ii, jj = np.nonzero(occ)
        wx = (self.grid_cx - self.map_size / 2) + (jj + 0.5) * self.res
        wy = (self.grid_cy - self.map_size / 2) + (ii + 0.5) * self.res
        dx, dy = wx - rx, wy - ry
        c, s = math.cos(-yaw), math.sin(-yaw)
        bx = c * dx - s * dy
        by = s * dx + c * dy
        d2 = bx * bx + by * by
        keep = (d2 > 1e-6) & (d2 < self.cfg.scan_max_range ** 2)
        return np.column_stack((bx[keep], by[keep]))

    def grid_data(self, now: float):
        """当前占据栅格 -> OccupancyGrid.data(行优先 int8 list, 100/0).

        OccupancyGrid 按行优先且 x 为列, 故对 (i,j) 矩阵转置后展平。
        """
        occ = (self.occ_t > self.clr_t) & \
              (now - self.occ_t < self.occ_timeout)
        data = np.where(occ, 100, 0).astype(np.int8)
        return data.T.flatten().tolist()


# ----------------------------------------------------------------------
# 传感器消息解析
# ----------------------------------------------------------------------
def parse_xyz(msg) -> list:
    """最小化 PointCloud2 xyz 解析(避免依赖 sensor_msgs_py).

    :returns: ``[(x, y, z), ...]``; 字段缺失/非 float32/空消息返回 ``[]``。
    期望常规排布的 x/y/z(float32)字段, frame_id 任意(需 TF 可达 odom)。
    """
    fields = {f.name: f for f in msg.fields}
    if not {'x', 'y', 'z'} <= set(fields):
        return []
    offsets = [fields[c].offset for c in ('x', 'y', 'z')]
    dtypes = [fields[c].datatype for c in ('x', 'y', 'z')]
    if any(d != PointField.FLOAT32 for d in dtypes):
        return []
    step = msg.point_step
    buf = msg.data
    n = len(buf) // step
    if n == 0:
        return []
    arr = np.frombuffer(bytes(buf), dtype=np.uint8, count=n * step)
    arr = arr.reshape(n, step)
    out = []
    for off in offsets:
        out.append(np.frombuffer(arr[:, off:off + 4].tobytes(),
                                 dtype='<f4'))
    x, y, z = out
    keep = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    return list(zip(x[keep].tolist(), y[keep].tolist(), z[keep].tolist()))


def scan_to_xy(msg):
    """LaserScan -> 车体系点云.

    :returns: ``(pts, n)``: pts 为 (N,2) numpy 数组(x 前 y 左), n 为原始
        光束数(可能为 0)。剔除 inf/超量程/过近光束。
    """
    n = len(msg.ranges)
    if n == 0:
        return np.zeros((0, 2)), 0
    min_r = float(msg.range_min)
    max_r = float(msg.range_max or 1e3)
    idx = np.arange(n)
    ang = msg.angle_min + idx * msg.angle_increment
    rng = np.asarray(msg.ranges, dtype=np.float64)
    valid = np.isfinite(rng) & (rng > max(min_r, 0.03)) & (rng < max_r)
    xs = rng[valid] * np.cos(ang[valid])
    ys = rng[valid] * np.sin(ang[valid])
    return np.column_stack((xs, ys)), n
