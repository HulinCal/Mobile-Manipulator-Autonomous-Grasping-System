# chassis_guard — 底盘动态避障守护节点

语音/键盘手动驾驶时的**反应式动态避障**。通过**话题分离**接入，

节点主文件(chassis_guard_node.py)只负责 ROS IO、共享状态与调度，
全部功能下沉到 8 个子模块。本文档按"模块依赖 → 闭环路径追踪 →
数据流 → 核心概念 → 逐函数索引 → 参数 → 状态机 → 测试与踩坑"组织。

---

## 1. 包结构与模块依赖关系

### 1.1 文件清单

```
src/chassis_guard/
├── chassis_guard/
│   ├── __init__.py
│   ├── chassis_guard_node.py   # 组合入口: ROS IO + 共享状态 + 调度
│   ├── config.py               # GuardConfig: 全部参数 + 速度自适应公式
│   ├── geometry_utils.py       # wrap_angle / yaw_from_quat / EPS_VEL
│   ├── footprint.py            # 环境感知①: 整车矩形碰撞 
│   ├── local_map.py            # 环境感知②: 滚动栅格地图 + 点云解析
│   ├── safety.py               # 每帧运动安全评估(硬急停判定)
│   ├── planner.py              # 选择绕行方向: VFH-lite 走廊选向
│   ├── rescue.py               # 脱困策略: 原地逆时针转圈找路
│   └── motion.py               # ARC 弧线蠕行 / TURN 原地转向 + 看门狗
├── config/chassis_guard.yaml   # 全部参数
├── launch/chassis_guard.launch.py
├── package.xml / setup.py / setup.cfg
└── README.md
```

### 1.2 模块依赖图（箭头 = "依赖/调用"）

```
                    chassis_guard_node.py  (唯一接触 ROS 的模块)
                     │   │   │   │   │   │
        ┌────────────┘   │   │   │   │   └────────────┐
        ▼                ▼   ▼   ▼   ▼                ▼
     config.py      planner rescue safety          motion
     (参数容器)        │      │      │                │
        ▲             ▼      ▼      ▼                ▼
        └───      ───── footprint.py              geometry_utils.py
        │                 ▲                            ▲
        └───────  ───── local_map.py ──────────────────┘
```

依赖规则：

| 模块 | 依赖 | 被谁用 | 职责一句话 |
|---|---|---|---|
| `geometry_utils` | 仅 numpy/math | 所有模块 | 角度归一化、四元数→偏航、零速常量 |
| `config` | 仅 dataclasses | 所有模块 | 参数容器 + 速度自适应三公式（唯一实现处） |
| `footprint` | config, geometry_utils | planner / safety / rescue / 节点 | 走廊净空、旋转扫掠、路径预估、击杀矩形急停 |
| `local_map` | config | 节点 | odom 系滚动占据栅格、LaserScan/PointCloud2 解析 |
| `planner` | config, footprint | 节点 | 最小偏转走廊 best_option、半球兜底 search_all、障碍方位、平行判定 |
| `safety` | geometry_utils (+footprint 入参) | 节点 | 按**实际发布速度**评估本帧是否该硬急停 |
| `rescue` | config, footprint | 节点 | 原地 CCW 转圈状态机，找到第一条可通行路径即 done |
| `motion` | config | 节点 | ARC/TURN 控制律 + 原地转向看门狗 |
| `chassis_guard_node` | 以上全部 | — | ROS 接线、绕行状态机（承诺/锁定/回正/换侧）、脱困门控 |
---

## 2. 完整闭环代码路径追踪

### 2.1 总览图

```
语音/键盘 ─/cmd_vel_in─▶ 守卫(20Hz) ─/cmd_vel─▶ 底盘 MCU
                           ▲
                    /scan 雷达回调(10Hz) ← 检测障碍 + 每帧硬急停
                           │
                    /tf (odom_combined↔base_footprint)
```

### 2.2 闭环追踪表

| # | 阶段 | 文件 | 函数 | 行号 | 说明 |
|---|---|---|---|---|---|
| **0** | 命令到达 | `chassis_guard_node.py` | `_on_cmd_vel` | **229-264** | 记录 `intent_vx/wz`；**命令真变化才生效**（重复心跳忽略，不打断脱困）；新命令打断急停/脱困锁存、清 `commit_side/stuck_cnt`，前进意图锚定 `anchor_yaw/anchor_x/anchor_y` |
| **1** | 每帧硬急停 | `chassis_guard_node.py` | `_on_scan` | **424-460** | 雷达回调用**实际发布速度** `_last_sent` 调 `safety.evaluate_motion`；绕行中阈值收紧为 `emergency_stop_distance`；blocked → 零速 + `stuck_cnt+=1` → `_try_rescue`(**329**) 门控 |
| **2** | 运动安全评估 | `safety.py` | `evaluate_motion` | **42-77** | 纯旋转→`disk_blocked`；前进→`kill_rollout` 碰撞距离 < 停距才阻塞；另算 `clear_now`（emergency_clear 短视野，滞回解除用） |
| **3** | 写局部地图 | `chassis_guard_node.py` + `local_map.py` | `_on_scan`(2) / `ray_update` | **462-490** / **68-82** | 扫描线 Bresenham 清空 + 命中占据（雷达 5s / 视觉 2s 衰减）。**急停不依赖此步**，TF 失败不影响急停 |
| **4** | 控制循环入口 | `chassis_guard_node.py` | `_control_loop` | **543-635** | 20Hz。优先级：脱困状态机(549-591) > 零意图静默(593) > 雷达超时停车(600) > 融合点集 raw+memory(606-622) > 纯旋转意图圆盘检查(624-635) |
| **5** | 脱困状态机 | `rescue.py` | `RescueController.step` | **85-124** | `emergency_active` 时最高优先级：原地 CCW(0.15rad/s) 转圈，当前车头走廊净空 > lookahead 即 `done` → 节点以当前车头为新锚点前进(578-591)。**中途绝不停车** |
| **6** | 脱困门控 | `chassis_guard_node.py` | `_try_rescue` | **329-408** | 三条件：不在绕行阶段 + 视野无**可达**走廊 + （或）stuck≥10 强制。可达性 = `search_all` 找到走廊 **且** `rotation_blocked`=False（转不过去的纸面走廊不否决脱困，死锁修复） |
| **7** | 平行判定 | `chassis_guard_node.py` | `_obstacle_passed` | **287-325** | 按**自身进度**判：承诺绕行时记起点与障碍近端纵向坐标 `commit_obs_x`，沿原航线走过 `obs_x - parallel_x(0.10m)` 即平行。平行前 `clear_count` 锁 0，方向完全冻结 |
| **8** | 首次选向 | `chassis_guard_node.py` | `_control_loop` 未承诺分支 | **674-727** | 锚点堵且障碍 ≤0.4m·s → `planner.best_option` 左右各搜，障碍质心对侧优先(`obstacle_side`)，锁定 `locked_heading`（相对锚点的世界系偏移） |
| **9** | 走廊选向 | `planner.py` | `best_option` / `search_all` | **65-94** / **96-110** | 从锚点起按 veer_step(5°) 递增，首个净空 > lookahead 的方向**再多让一步**（弧线余量）；±80° 全堵时 `search_all` 在前向半球 ±90° 取偏转最小走廊（绝不掉头） |
| **10** | 已承诺约束 | `chassis_guard_node.py` | `_control_loop` 承诺分支 | **728-817** | 回正(731-752)：平行 + 锚点连续畅通 `return_cycles` 帧 → 锚点方向 + lat_kp·横向偏移修正；已平行但原航线持续堵 3×N 帧才换锚(756-780)；**平行前完全冻结**(781-785)；仅承诺侧全堵且对侧可用才换侧(808-817) |
| **11** | 控制律 | `motion.py` | `arc_command` / `turn_rate` | **36-47** / **50-53** | err ≤ fwd_align(0.6rad≈34°) → ARC：err 限幅 ±veer_step，vx 按偏差衰减(≥0.3×)，wz=steer_kp·err；否则 TURN：vx=0，wz 限速 0.15rad/s |
| **12** | TURN 前检查 | `chassis_guard_node.py` | `_control_loop` TURN 分支 | **858-898** | 用**物理车体矩形** `rotation_blocked` 判定能否转过去；转不过 → `_abort_bypass_and_rescue`(410-419)：`stuck_cnt+=1` + 清承诺 + 零速 + `_try_rescue`。看门狗 `InPlaceTurn.tick`：指令转角超初始偏差 1.3 倍停 1s 等 TF；超时**分段续转**不闭锁(885-894) |
| **13** | 路径预估闭环 | `footprint.py` | `rollout_clearance`(kill 矩形) | **135-163** | 控制循环 **900-922**：对最终 (vx,wz) 沿弧线每 5cm 逐位姿扫掠击杀矩形；碰撞距离 < 触发阈值（直行=stop_distance，绕行=emergency_stop_distance）→ 零速 + `stuck_cnt+=1` + `_try_rescue`。**只用 raw_pts**（记忆点经 TF 可能假阳性） |
| **14** | 发布出口 | `chassis_guard_node.py` | `_publish` / `_send_once` | **939-955** | 唯一出口；记录 `_last_sent`（急停评估用）；**仅前进(vx≠0)清零 stuck_cnt**（纯旋转不清零，否则 TURN/ARC 抖动时 stuck 永远到不了阈值） |
| **15** | 退出停车 | `chassis_guard_node.py` | `main` | **976-994** | Ctrl+C 后先 20Hz 发零速 1s 确保底盘停车，再销毁节点 |

### 2.3 关键变量速查（均在节点 `__init__` **125-151**）

| 变量 | 含义 |
|---|---|
| `intent_vx/wz` | 用户意图速度（`_on_cmd_vel` 设） |
| `anchor_yaw/x/y` | 锚点：命令时刻的世界航向与坐标，绕行全程不换，越障后回原航线 |
| `commit_side` | 0=未承诺 / +1=左绕 / -1=右绕 |
| `locked_heading` | 锁定绕行偏移（相对锚点，世界系恒定），平行前完全冻结 |
| `clear_count` | 锚点方向连续畅通计数，≥ return_cycles 才回正 |
| `recover_block_cnt` | 已平行但原航线持续堵塞计数，≥ 3×return_cycles 才换锚 |
| `emergency_active` | 急停锁存（进入脱困即置位） |
| `commit_xy / commit_obs_x` | 承诺绕行时起点与障碍近端纵向坐标（平行进度判据） |
| `stuck_cnt` | 连续受阻帧数（碰撞零速/abort 各 +1，前进清零），≥ STUCK_RELEASE(10) 强制脱困 |
| `raw_pts` | 最新原始雷达点（车体系），急停与 rollout 唯一数据源 |
| `_last_sent` | 守卫实际发布速度（急停评估用它，不用意图） |

### 2.4 两种碰撞矩形（`config.py` `__post_init__` **96-106**）

| 矩形 | 尺寸 | 用途 |
|---|---|---|
| **规划矩形** `half_l/w` | physical + safety_margin(0.02) | 选向走廊净空、rollout 规划验证（保守） |
| **击杀矩形** `kill_half_l/w` | physical + 0.02 | 每帧硬急停 `kill_rollout`（贴物理车体，侧面 3-5cm 缝隙不误判） |
| **物理矩形** `robot_half_l/w` | 0.24 / 0.21 实测 | 原地旋转扫掠 `rotation_blocked`（带裕量的矩形会把"贴障碍停车"判成永远转不动） |

---

## 3. 原理与数据流

> 先根据车的尺寸预计行进路线，看路线上有没有障碍，
> 有就提前绕行/停车，没有才动车；每 50ms 重新评估一次（闭环）**。

**话题分离是根基**：守卫订 `/cmd_vel_in`（意图），发到 `/cmd_vel`。
输出永不回灌成输入，对 ROS 共享内存乱序/延迟完全免疫。

```
LaserScan ─scan_to_xy(local_map.py:183)→ raw_pts(车体系) ──┐
                                                            ├─ pts ─→ 走廊选向/rollout/急停
局部动态地图 ─memory_points(local_map.py:119)→ mem_pts ────┘         ↓
(odom 系滚动栅格, 补侧后盲区)                              (vx,wz) → /cmd_vel
```

关键点：**决策点集 = 原始雷达点 + 地图记忆点**；**硬急停与最终 rollout
只用原始点**（不经过地图/TF，任何一路失效都能停）。

---

## 4. 核心概念精读

### 4.1 速度自适应（唯一实现处：`config.py`）

```python
s = max(1.0, |vx| / tuned_vx)                       # tuned_vx=0.05
lookahead(vx)   = max(min_clearance*s, vx*brake_time)   # 走廊选向前瞻
stop_distance(vx) = vx*brake_time + emergency_dist*s    # 直行急停停距
emergency_stop_distance(vx) = emergency_dist*s          # 绕行中收紧阈值
```

提速后所有反应距离自动放大，无需手调。

### 4.2 车体碰撞模型（`footprint.py`）

真实矩形 0.48×0.42m（URDF 实测）

| 函数 | 行号 | 用途 |
|---|---|---|
| `forward_gap` | 58-75 | 前方矩形带内前缘净空 |
| `disk_blocked` | 77-84 | 外接圆阻塞（**仅纯旋转意图**用） |
| `corridor_clearance` | 86-101 | 沿航向直行走廊净空（选向/脱困找路） |
| `rotation_blocked` | 103-130 | 原地旋转逐位姿矩形扫掠（TURN 前检查/门控可达性） |
| `rollout_clearance` | 135-163 | 直/圆弧路径预估，空间步长 0.05m |
| `kill_rollout` | 165-198 | 击杀矩形路径预估（每帧硬急停），horizon=停距且 ≥0.5·s |

### 4.3 局部动态地图（`local_map.py` `LocalMap`）

- odom 系 6m×6m、0.05m/格滚动栅格；`occ_t/clr_t` 双时间戳 +
  每格 `occ_timeout`（雷达 5s / 视觉 2s）。
- 占据判定：`occ_t > clr_t` 且未超时——动态障碍离开后不残留。
- `recenter`(**84-114**)：车走出 1m 按整格平移保留重叠记忆；位移超过整张
  地图（手动推车 odom 跳变）直接清空，不能切片拷贝（维度不匹配会崩）。
- `memory_points`(**119-138**)：未超时占据格 → 车体系点，补雷达侧后盲区。

### 4.4 VFH-lite 选向（`planner.py`）

- `best_option`(**65-94**)：首个净空 > lookahead 的方向**再多让一步
  veer_step**——ARC 弧线实际轨迹比直线走廊宽（车头外摆/车尾内切），实测
  "转弯太小贴边会触发脱困"，多让一步留余量；多让方向仍畅通才采用。
- `search_all`(**96-110**)：前向半球 ±90° 从 0° 向两侧取偏转最小走廊；
  **绝不选 |h|>90°**（等于掉头沿来路后退，用户禁止）。
- `obstacle_side`(**115-131**)：正前方 x∈(0.2,1.5)、|y|<0.6 障碍点 y 中位
  数定绕对侧。
- `parallel_passed`(**133-155**)：锚点系车道内前方无点即平行（节点实际
  用 `_obstacle_passed` 的进度判据，见 2.2 #7）。

### 4.5 方向承诺与回正（节点 `_control_loop` **728-817**）

- **平行前完全冻结**：锁定航向不重选/不换侧，碰撞安全由 rollout 急停兜底
  （实测绕行中因走廊误判反复改向最终撞障）。
- **回正**：平行 + 锚点连续畅通 `return_cycles`(3) 帧 → 目标 = 锚点方向
  + lat_kp(0.6)·横向偏移修正；修正方向被挡退化为锚点方向，**不换锚**。
- **换锚**：已平行但原航线持续堵 3×N 帧，才以当前前行方向为新锚点。
- **换侧**：仅承诺侧走廊完全消失且对侧可用（TURN 期间禁止）。

### 4.6 控制律与转向看门狗（`motion.py`）

- ARC(**36-47**)：err 限幅 ±veer_step（每帧最多逼近 5°，禁止大弧甩头）；
  vx 衰减保底 0.3×；wz=steer_kp·err 限幅 max_steer_w（增益不随速度放大，
  实测 2x 速度下 wz=0.21 太快）。
- TURN：限速 `BYPASS_TURN_W=0.15`rad/s（EKF 滞后快转会过冲回摆）。
- `InPlaceTurn.tick`(**78-96**)：指令转角 > 初始偏差×1.3+0.15 → 停 1s 等
  TF 收敛（EKF 原地旋转滞后：指令 50° TF 只记 16°）；超 max_rotate_time
  由节点**分段续转**（重置计时继续转，绝不无声闭锁零速）。

### 4.7 双层急停

| 层 | 位置 | 数据 | 判定 |
|---|---|---|---|
| **每帧层** | `_on_scan` (424-460) → `safety.evaluate_motion` | 仅 raw_pts | 实际速度 (vx,wz) 的 kill_rollout 碰撞距离 < 阈值（直行 stop_distance / 绕行 emergency_stop_distance） |
| **周期层** | `_control_loop` (900-922) → `rollout_clearance`(kill 矩形) | 仅 raw_pts | 最终指令扫掠碰撞 < 同上阈值 |

触发即零速 + `stuck_cnt+=1` + `_try_rescue` 门控。**绝不自动后退**（用户
要求，代码内无负 vx）。

### 4.8 急停脱困（`rescue.py` + 节点门控）★ 本次死锁修复核心

**门控（`_try_rescue` 329-408）** 三条件全过才进脱困：

1. 不在绕行阶段（未承诺 / 已越过障碍）；
2. 视野无**可达**走廊：`search_all` 找到的走廊必须 `rotation_blocked`=False
   ——纸面走廊转不过去等于没有路（旧实现用不可达走廊否决脱困：每帧选中
   同一条 +75° 走廊、每帧被扫掠否决，唯一能改变位姿的脱困转圈被门控挡死，
   stuck=1096 永久静止）；
3. **stuck≥10 强制脱困**：连续受阻 10 帧直接 `rescue.enter()` 绕过门控。

**stuck_cnt 计数规则**（三处失效修复）：

| 场景 | 行为 | 位置 |
|---|---|---|
| 碰撞零速（scan/rollout 拦截） | +1 | 449, 918 |
| abort（rotation_blocked 否决 TURN） | +1 | 415 |
| 发布前进速度 | 清零 | 948（纯旋转**不清零**——否则 TURN/ARC 抖动永远攒不到 10） |
| 新命令 / 脱困成功 / 强制脱困触发 | 清零 | 256, 587, 362 |

**脱困行为（`rescue.py` `RescueController.step` 85-124）**：

- 原地 CCW 0.15rad/s 匀速转圈，每帧查**当前车头**走廊净空 > lookahead
  → `done`，节点以当前车头为新锚点直接前进；
- **中途绝不停车**：转满整圈、扫掠受阻都不中断（实测任何一次中途停车
  都会让小车原地不动）；转动中不做碰撞否决（原地旋转是唯一脱困手段）；
- 语音/键盘**新命令**（真变化的）立即打断脱困（`_on_cmd_vel` 245-250），
  碰撞保护不取消——真会撞时下一帧重新锁停。

---

## 5. 逐函数索引（按模块）

### 5.1 `chassis_guard_node.py` — ROS 接线与调度

| 函数 | 行号 | 职责 |
|---|---|---|
| `__init__` | 61-189 | 声明参数 → 构造 cfg/模块 → 状态变量 → TF/IO/定时器 |
| `_set_status` | 194-200 | /guard/status（TRANSIENT_LOCAL，后连可见） |
| `_on_pick_state` | 205-224 | pick_place 任务旁路开关 |
| `_on_cmd_vel` | 229-264 | 意图登记（去重）；新命令打断脱困、清承诺/stuck、锚定 |
| `_nearest_lane_obs_x` | 269-285 | 障碍近端在锚点系车道的纵向坐标 |
| `_obstacle_passed` | 287-325 | 平行判定（自身进度 ≥ obs_x - parallel_x） |
| `_try_rescue` | 329-408 | 脱困门控：可达性检查 + stuck≥10 强制 |
| `_abort_bypass_and_rescue` | 410-419 | 走不通：stuck+1、清承诺、零速、试脱困 |
| `_on_scan` | 424-490 | 每帧硬急停 + 写局部地图 |
| `_on_vision_points` | 495-516 | 视觉 PointCloud2 → 地图（预留融合接口） |
| `_tf_odom_scan` / `_robot_pose` / `_robot_yaw` | 521-538 | TF 辅助 |
| `_control_loop` | 543-934 | 20Hz 主循环（见 2.2 表） |
| `_publish` / `_send_once` | 939-955 | 唯一发布出口；零速去重不刷屏 |
| `_publish_map` | 960-973 | OccupancyGrid 可视化 2Hz |
| `main` | 976-994 | Ctrl+C 先零速 1s 再退出 |

### 5.2 子模块

| 模块 | 公开接口 | 行号 | 职责 |
|---|---|---|---|
| `config.py` | `GuardConfig` / `from_params` | 32-190 | 参数容器；`__post_init__`(96) 派生矩形；`speed_scale`/`lookahead`/`stop_distance`/`emergency_stop_distance`(111-134) |
| `geometry_utils.py` | `wrap_angle` / `yaw_from_quat` / `EPS_VEL` / `ZERO_VEL` | 23-41 | 几何基础 |
| `footprint.py` | `RobotFootprint` | 37-198 | 碰撞原语（见 4.2 表） |
| `local_map.py` | `LocalMap` / `scan_to_xy` / `parse_xyz` | 30-200 | 滚动栅格 + 传感器解析 |
| `safety.py` | `evaluate_motion` / `MotionSafety` | 34-77 | 单帧运动安全评估 |
| `planner.py` | `BypassPlanner` | 38-155 | 走廊选向/障碍方位/平行判定 |
| `rescue.py` | `RescueController` / `RescueOutcome` | 45-124 | 脱困转圈状态机（CCW_RATE=0.15） |
| `motion.py` | `arc_command` / `turn_rate` / `InPlaceTurn` | 33-100 | 控制律 + 转向看门狗 |

---

## 6. 参数速查（config/chassis_guard.yaml）

| 参数 | 默认 | 含义 / 调参建议 |
|---|---|---|
| `robot_half_length/width` | 0.24/0.21 | URDF 实测（轮子外缘），勿随意改 |
| `safety_margin` | 0.02 | 规划矩形裕量 |
| `min_clearance` | 0.3 | 走廊最小净空（乘 s 后生效） |
| `emergency_dist` | 0.09 | 急停缓冲。2x 速度停距 0.28m；**勿低于 0.06**（0.19m 停距实测刹不住） |
| `emergency_clear` | 0.20 | 急停解除滞回（必须 > emergency_dist） |
| `brake_time` | 1.0 | 停距 = vx·brake_time + emergency_dist·s |
| `tuned_vx` | 0.05 | 标定速度；s=vx/tuned_vx 自动放大所有反应距离 |
| `forward_align_angle` | 0.6 | ARC/TURN 分界 ≈34°（代码默认 0.9，yaml 覆盖为 0.6） |
| `max_veer_angle` | 1.4 | 绕行最大偏航 ±80° |
| `veer_step_deg` | 5.0 | 选向步进角（角度量，不随速度缩放） |
| `parallel_x` | 0.10 | 平行判据：障碍在锚点系纵向 x ≤ 此值即已到车中心之后 |
| `return_confirm_cycles` | 3 | 锚点连续畅通 N 帧才回正；换锚需 3×N |
| `steer_kp` / `max_steer_w` | 1.2 / 0.35 | 航向 P 增益 / 转向限速（TURN 实际限速 0.15） |
| `max_rotate_time` | 12.0 | 单段转向预算（0.15rad/s 转满 80° 需 9.3s；旧值 4s 必然超时闭锁） |
| `rescue_turn_tol` | 0.05 | 脱困到位容差，**必须 < veer_step(0.087)** 否则空转 |
| `scan_timeout` | 0.5 | 雷达超时停车不盲走 |
| `scan_max_range` / `scan_beams` | 1.5 / 120 | 只关心近处 / 每帧抽样光束数 |
| `lidar/vision_obstacle_timeout` | 5.0 / 2.0 | 记忆衰减（视觉更短，应动态障碍） |
| `map_size` / `map_resolution` | 6.0 / 0.05 | 滚动栅格 |
| `bypass_states` | 4 态 | pick_place 任务旁路 |

---

## 7. 状态机（/guard/status）

```
IDLE ──意图前进──> ACTIVE(cruise) ──走廊<lookahead──> AVOIDING(TURN/ARC)
  ▲                    │  平行 + 锚点畅通 3 帧             │
  │                    └────────── RETURNING ◄────────────┘
  │  任意时刻:  实际轨迹碰撞<停距        → EMERGENCY(零速 → CCW 转圈找路)
  │            stuck≥10(连续受阻)       → EMERGENCY(强制, 绕过门控)
  │            前向半球彻底无路          → EMERGENCY(经 _abort_bypass_and_rescue)
  │            纯旋转意图圆盘被堵        → BLOCKED(零速等畅通)
  │            雷达超时                 → DEGRADED(停车)
  │            TF 失效                  → DEGRADED(仅原始点, 透传降级)
  │  pick_place 任务中                  → BYPASS(静默透传)
  └──── 意图为零 / 脱困找到路后恢复
```

---

## 8. 真机测试清单

```bash
# 1) 启动底盘+雷达(robotandlidar), 再启动守卫
ros2 run chassis_guard chassis_guard_node --ros-args \
    --params-file src/chassis_guard/config/chassis_guard.yaml

# 2) 确认底盘物理急停按钮已释放! (通信正常但轮不转的 #1 原因)

# 3) 发前进意图并观察
ros2 topic pub -r 10 /cmd_vel_in geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
ros2 topic echo /guard/status
ros2 topic echo /cmd_vel
# RViz: OccupancyGrid 订 /guard/local_map, LaserScan 订 /scan
```

**测试纪律（踩坑总结）**：
1. 放好车/障碍后人**离开雷达视野再发意图**——人腿会致全向 BLOCKED
   并可能承诺一个荒谬的绕行方向；
2. 每次测试前重启节点（或等记忆 5s 衰减）清地图；
3. 先看 `/guard/status` 首条是否为 `ACTIVE: cruise`；
4. odom 有位移 ≠ 车在动：`/odom_combined` 的 twist 才是轮速真值；
5. 撞车后先查急停按钮，再查软件；
6. 脱困排查看三类日志：`corridor ... unreachable (rotation blocked)`、
   `RESCUE FORCED`、`rescue: passable heading found`。