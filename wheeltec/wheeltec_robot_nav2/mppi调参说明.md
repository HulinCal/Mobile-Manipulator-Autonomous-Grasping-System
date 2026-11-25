# MPPI 控制器调试总结

## 1. 涉及文件

| 文件 | 作用 |
|---|---|
| `launch/wheeltec_nav2_mppi.launch.py` | 独立启动文件，结构与 `wheeltec_nav2.launch.py` 一致，参数文件指向 `param_mppi.yaml` |
| `param/wheeltec_param/param_mppi.yaml` | MPPI 专用参数，基础参数沿用 `param_four_wheel_diff_bs.yaml`，仅 `FollowPath` 换成 MPPI |

启动：

```bash
ros2 launch wheeltec_nav2 wheeltec_nav2_mppi.launch.py                # 带 RViz
ros2 launch wheeltec_nav2 wheeltec_nav2_mppi.launch.py open_rviz:=false  # 不带 RViz
```

> 注意：launch 参数名是 `open_rviz`，不是 `rviz`。
> 参数文件读取的是 install 副本，改完 src 必须 `colcon build --packages-select wheeltec_nav2`。

---

## 2. 三条硬约束（违反会直接异常或卡死）

### 2.1 控制周期 ≤ model_dt

```text
controller_frequency: 5.0  →  周期 0.2 s
model_dt: 0.2 s            →  满足 0.2 ≤ 0.2
```

违反时报错并直接退出：

```text
Controller period more then model dt, set it equal to model dt
FATAL: Lifecycle node controller_server does not have error state implemented
```

改 `controller_frequency` 必须同步改 `model_dt`（如 10 Hz 对应 `model_dt: 0.1`）。

### 2.2 inflation_radius ≥ 轮廓外接圆半径

当前轮廓 `0.52 × 0.44`，外接圆 `sqrt(0.26² + 0.22²) = 0.3406 m`，局部 `inflation_radius: 0.45` 满足要求。

低于该值时日志约 20 Hz 刷屏：

```text
The inflation radius (0.320000) is smaller than the circumscribed radius (0.367696)
```

### 2.3 转向能力必须大于进度检查窗口

`progress_checker`：`required_movement_radius: 0.2`、`movement_time_allowance: 10.0`。
`wz_max: 0.1 rad/s` 时 10 s 只能转 57°，转 90° 要 15.7 s，必然触发：

```text
Failed to make progress
```

---

## 3. 参数演进过程（问题 → 改动 → 原因）

| # | 现象 | 改动 | 原因 |
|---|---|---|---|
| 1 | 初版 | `time_steps 40` / `model_dt 0.05` / `batch_size 1000` / `vx 0.20` / `wz 0.50` | 以原 DWB `sim_time 1.7 s` 为参照，2.0 s 时域 |
| 2 | 不在路中间、目标点附近来回走 | 全局 `inflation_radius 0.25→0.40`、`cost_scaling 2.0→3.0`；`PathAlign 14→20`、`offset_from_furthest 20→10`；`xy_goal_tolerance 0.10→0.12`、`yaw_goal_tolerance 0.10→0.15`；`PreferForward` 触发距离 `0.5→0.2`；`GoalAngle 阈值 0.5→0.8` | 全局膨胀过小导致路径贴墙；目标附近多个 critic 同时生效造成往复修正 |
| 3 | 撞墙、不沿全局路径 | `CostCritic 3.81→10`、`near_goal_distance 1.0→0.2`；局部 `inflation 0.25→0.32`；`PathAlign 20→14`、`max_path_occupancy_ratio 0.05→0.20` | 避障权重过低；接近目标 1 m 就弱化避障；`max_path_occupancy_ratio` 过小会让路径对齐 critic 在窄道被整段停用 |
| 4 | 日志刷屏 `Failed to make progress` | 局部 `inflation 0.32→0.45`；`PathFollow 50→5`、`PathAngle 20→2` | 0.32 < 外接圆 0.3677；路径类 critic 权重几十倍于避障，必然撞墙 |
| 5 | 偏离全局路径方向 | `GoalCritic 阈值 1.4→0.5`、`GoalAngle 0.8→0.5`、`PathFollow 阈值 1.4→0.5`、`PathAlign 阈值 0.5→0.2`；`PathAlign 14→20`、`max_path_occupancy_ratio 0.20→1.0`、`PathFollow 5→8`；`vx_std 0.15→0.08`、`wz_std 0.20→0.12`；`CostCritic 10→20`；`VelocityDeadband 35→10`、`deadband 0.05→0.02` | 距目标 1.4 m 就切入“直奔目标”，等于允许抄近路；采样噪声过大导致左右摆动 |
| 6 | 要求“必须严格走全局路径” | `PathAlign 20→40`、`use_path_orientations false→true`、`PathAngle 2→8`、`max_angle_to_furthest 1.0→1.2`、`PathFollow 8→10`；`CostCritic 20→10`；`GoalCritic/GoalAngle 3.0/0.3m`；`PreferForward` 关闭；`vx_min -0.10→0`；`vx_std/wz_std → 0.05/0.08`；`temperature 0.3→0.2`；`batch_size 1000→700`；`visualize true→false` | 路径贴合设为最高优先级；禁止倒车避免“后退再前进”式绕路；降低 CPU 占用 |
| 7 | configure 阶段 FATAL | `model_dt 0.1→0.2`、`time_steps 60→20` | 见 2.1 |
| 8 | 撞墙风险审查 | `wz_max 0.1→0.50`、`vx_max 0.40→0.25`、`vx_min -0.40→-0.10`、`vy_std 0.05→0`；`prune_distance 1.0→1.2`；`CostCritic 10→15`；`PreferForward` 开启；`footprint 0.50×0.50→0.52×0.44 + padding 0.01`；局部 `cost_scaling 2.0→3.0` | 见 2.3；0.4 m/s 在 5 Hz 下每周期位移 8 cm，避障反应距离不足；倒车速度是前进的 2 倍最危险；`prune_distance` 需大于 `vx_max × 时域` |
| 9 | 中途停车等待 | `vx_std 0.05→0.10`、`wz_std 0.08→0.10`、`temperature 0.2→0.3`、`PathFollow 10→20`、`CostCritic 15→10`、`VelocityDeadband 10→3`、`deadband 0.02→0.01` | 见第 5 节“MPPI 粘滞” |

---

## 4. 当前参数与设计依据

### 4.1 运动学与采样

| 参数 | 值 | 依据 |
|---|---:|---|
| `vx_max` | 0.25 m/s | 5 Hz 下每周期 5 cm，留足避障反应距离 |
| `vx_min` | -0.10 m/s | 允许小幅倒车脱困，但车尾是雷达遮挡区，必须慢 |
| `wz_max` | 0.50 rad/s | 10 s 可转 286°，不会触发进度超时 |
| `vy_max` | 0.0 | 差速车不能横移 |
| `vx_std` / `wz_std` | 0.10 / 0.10 | 过小会粘滞在零速，过大会左右摆动 |
| `time_steps × model_dt` | 20 × 0.2 = 4.0 s | 0.25 m/s 下前瞻 1.0 m |
| `batch_size` | 500 | 1000 时实测控制环掉到 5~7 Hz |
| `prune_distance` | 1.2 m | 必须 > `vx_max × 时域`= 1.0 m |
| `temperature` | 0.3 | 0.2 过于尖锐，容易锁定零速 |
| `motion_model` | DiffDrive | 差速底盘 |

### 4.2 Critic 权重（当前）

| Critic | 权重 | 作用 |
|---|---:|---|
| `PathAlignCritic` | 40 | 最高：轨迹必须贴合全局路径，`use_path_orientations: true` 同时约束方向 |
| `PathFollowCritic` | 20 | 抗粘滞主力量：奖励沿路径推进 |
| `CostCritic` | 10 | 避障；碰撞由 `collision_cost: 1e6` 绝对否决，不依赖权重 |
| `PathAngleCritic` | 8 | 车头与路径切向对齐，防斜插 |
| `PreferForwardCritic` | 5 | 抑制无谓倒车 |
| `ConstraintCritic` | 4 | 速度越限惩罚 |
| `GoalCritic` / `GoalAngleCritic` | 3 / 3 | 压低，防止抄近路；阈值 0.3 m |
| `VelocityDeadbandCritic` | 3 | 过高会抹平低速修正，造成“走一下停一下” |

### 4.3 碰撞相关

| 参数 | 值 | 说明 |
|---|---:|---|
| `consider_footprint` | true | 前进、后退都按整车矩形轮廓检查 |
| `critical_cost` | 300 | 只有真 lethal（254）判碰撞；若设为 253 会要求离墙 0.44 m，窄道无解 |
| `collision_cost` | 1e6 | 碰撞轨迹一律淘汰 |
| `near_goal_distance` | 0.2 m | 只在最后 0.2 m 才弱化避障 |
| 局部 `inflation_radius` | 0.45 | ≥ 外接圆 0.3406 |
| `footprint` | `[[-0.26,-0.22],[-0.26,0.22],[0.26,0.22],[0.26,-0.22]]` | 实测车体 0.50×0.42 + 1 cm padding |

### 4.4 配套修复

| 项目 | 问题 | 修复 |
|---|---|---|
| `behavior_server.local_frame` | 缺省查找不存在的 `odom`，spin/backup 全部失败 | 显式设 `odom_combined` |
| 全局 `inflation_radius` | 0.25 只比内切半径多 1 格，路径贴墙 | 0.40 |
| `visualize` | `/trajectories` 发布占 CPU | 关闭；调试时再开 |

---

## 5. 典型问题速查

### 5.1 `Failed to make progress`

进度检查：10 s 内位移 < 0.2 m 即失败 → 中止 → 恢复行为 → 重新下发 FollowPath，表现为“停一会再走”。

按顺序排查：

1. 停车期间 `/cmd_vel` 是否为零？
   - 为零 → MPPI 粘滞（见 5.2）
   - 非零但车不动 → 底盘/使能/odom 停更，与 MPPI 无关
2. `wz_max` 是否过小，导致转不过弯；
3. `planner_server` 是否 `no valid path found`（全局成本地图被挡或目标不可达）。

### 5.2 MPPI 粘滞（停止不动）

机理：代价沿轨迹累加，“原地不动”只付起点一个点的代价，“往前走”要付整条轨迹的膨胀代价。当推进奖励不足时，零速成为数值最优解，且 `iteration_count: 1` + 小 `vx_std` + 尖锐 `temperature` 会让它多周期锁定。

对策：加大探索（`vx_std`、`temperature`）、加强推进奖励（`PathFollowCritic`）、削弱静止优势（`CostCritic`、`VelocityDeadbandCritic`）。

### 5.3 偏离全局路径

优先查全局路径本身是否居中；再确认各 critic 的 `threshold_to_consider` 不要过大（过大会在距目标很远时就开始“直奔目标”）。

### 5.4 撞墙

保证 `consider_footprint: true`、`critical_cost: 300`、`collision_cost` 足够大、局部 `inflation_radius ≥ 外接圆`、速度与频率匹配（每周期位移 5 cm 量级）。

### 5.5 看不到 `/amcl_pose`、`/particle_cloud`

1. AMCL 只在滤波器更新时发布，门限 `update_min_d: 0.025`、`update_min_a: 0.06`——**静止时静默是正常的**；
2. QoS 不匹配：`/amcl_pose` 是 `TRANSIENT_LOCAL`，`/particle_cloud` 是 `BEST_EFFORT`，CLI 默认 RELIABLE 收不到，还会触发 AMCL 的 incompatible QoS 警告；
3. 导航不依赖这两个话题，定位链路是 TF `map → odom_combined → base_footprint`。

---

## 6. 调参顺序建议

1. 先确认硬约束：频率/`model_dt`、膨胀/外接圆、`wz_max`/进度检查；
2. 再确定速度量级：`vx_max` 使每周期位移约 5 cm；
3. 再定 critics 的相对关系：`PathAlign ≥ PathFollow > Cost > PathAngle`；
4. 最后用 `vx_std` / `temperature` / `VelocityDeadband` 消除粘滞与抖动；
5. 每轮只改一组，配合 `visualize: true` 观察 `/trajectories`。
