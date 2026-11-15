# aruco_align_node 实时跟踪与轨迹平滑改进记录

针对 `aruco_align.launch.py` 链路中"跟随滞后、走一段停一下、到位急停"的问题，
依次在驱动层、轨迹插值、速度自适应、控制架构四层做了修改。

---

## 1. 驱动层：阻塞驱动 → 流式驱动（解决走-停-走）

原 `mycobot_driver_node.py` 使用 `fresh_mode=0` + 阻塞 `sync_send_angles`，
每条指令必须执行完才返回，导致每段运动之间必然停顿。

改为使用 `mycobot_driver_update_node.py`：
- `fresh_mode=1`（`fresh_mode=1`，最新目标覆盖旧目标）
- 非阻塞 `send_angles`，40 Hz 发送
- 10 Hz trylock 读关节状态

三处启动文件已切换：
- `src/wheeltec_mycobot_aggregation/wheeltec_mycobot_bringup/launch/all.launch.py`（L262-279，`aruco_align.launch.py` 使用）
- `src/wheeltec_mycobot_aggregation/wheeltec_mycobot_bringup/launch/arm_controller.launch.py`（L132-146）
- `src/mycobot/mycobot280_moveit2_control/launch/mycobot_280_real_moveit.launch.py`（L167-198）

统一参数 `speed=50`、`stream_hz=40.0`、`gripper_speed=30`。

### 补齐急停逻辑

`src/mycobot/mycobot280_moveit2_control/src/mycobot_driver_update_node.py`

| 功能 | 位置 | 说明 |
|---|---|---|
| 订阅 `/mycobot/stop` | L170-177 | 与旧驱动同接口，grasp_node 急停链路无需改动 |
| 急停回调 `stop_cmd_callback` | L246-267 | 丢弃目标 + `mc.stop()`；串口锁最多等 0.2 s，超时也照发 |
| 指令闭锁 | L215-218 | `stop_hold_time=0.5 s` 内丢弃关节指令，防止 JTC 在途插值点把臂重新带起来 |
| 发送侧防竞态 | L298-299 | 执行线程本周期读到的旧目标也不许下发 |
| 退出日志保护 | L390-396 | `rclpy.ok()` 判断，消除退出时 rosout 报错 |

### 发送去重阈值

同文件 L83-86：`SEND_EPS 0.1° → 0.02°`。
0.1° 会漏掉低速段的最后一帧，固件走到旧目标后停顿等下一帧，是低速卡顿的一个来源。

---

## 2. 轨迹平滑：单点匀速 → smoothstep 多点插值（解决到位急停）

原 `_send_positions` 只发 1 个轨迹点，JTC 匀速插值到目标后速度直接归零，
固件定速执行最后一个 setpoint 时是"冲过去瞬间停"。

`src/wheeltec_mycobot_aggregation/wheeltec_mycobot_pick_place_aruco/wheeltec_mycobot_pick_place_aruco/aruco_align_node.py`

| 功能 | 位置 | 说明 |
|---|---|---|
| `ALIGN_WAYPOINTS` | L71 | 单条轨迹插值点数，默认 20 |
| `_smoothstep / _smooth_points` | L100-120 | smoothstep 插值，起止速度、加速度均为 0 |
| `_send_positions` 多点轨迹 | L531 起 | 以当前指令位置为起点生成多点轨迹，末段点位移极小，JTC 与固件都会在目标前自然减速 |

---

## 3. 目标速度自适应（静止时柔，移动时快）

相机固定在基座上，`marker` 在 link1 系位置的变化率就是目标真实移动速度。

`aruco_align_node.py`

| 功能 | 位置 | 说明 |
|---|---|---|
| 测速常量 | L73-77 | 单帧跳变 >0.15 m 丢弃、速度上限 1.0 m/s、EMA 0.3 |
| `_on_marker` 差分测速 | L262-272 | link1 系位置差分得到 marker 速度 |
| `_adapt_alpha / _adaptive_rate / _update_speed_hint` | L359-382 | `ω = v/dist`，`α = clamp(ω/0.4, 0, 1)`，同时驱动轨迹速率与固件 speed |
| duration 自适应 | L511-513 | `duration = max(min_move_time, dq/rate)` |

新增参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `speed_adapt` | true | 关闭则退回固定速率 |
| `max_joint_rate_fast` | 0.6 rad/s | 快速移动时轨迹速率上限 |
| `adapt_omega_ref` | 0.4 rad/s | 达到满速所需法兰角速度 |
| `speed_slow` / `speed_fast` | 40 / 80 | 固件 speed 区间 |

驱动侧：`/mycobot/speed_hint`（Int32）订阅，0.5 s 超时回落静态 `speed=50`，
见 `mycobot_driver_update_node.py` 的 `speed_hint_callback / _effective_speed`。

---

## 4. 控制架构：离散对准 → 连续视觉伺服（核心改进）

### 问题

原逻辑每步：解算 → 单条轨迹（0.5 s 起）→ 等 action 结果 → 静置 1.0 s → 再解算，
实际控制只有 0.5-0.7 Hz，延迟约 1.5 s/步。

### 改动（`aruco_align_node.py`）

| 项 | 旧 | 新 | 位置 |
|---|---|---|---|
| 控制节拍 | 10 Hz | **20 Hz** | `create_timer(0.05, ...)` L238 |
| 跟踪时是否阻塞 | `_busy` 直接 return | **不阻塞**，按 `replan_interval` 重规划 | `_tick` L395-408 |
| 轨迹抢占 | 无 | 新目标前 `cancel_goal_async`，旧结果按序号过滤 | `_send_positions` L531 起、`_cancel_active` |
| 单步幅度 | 最大 60° | `速率 × replan_interval` | `_align_step` L491-502 |
| `replan_interval` | — | **0.15 s**（当前已调为 0.2 s） | L157 |
| `min_move_time` | 0.5 s | 0.15 s | L151 |
| `settle_time` | 1.0 s | 0.15 s（仅初始化后） | L152 |
| `cmd_hold_time` | 3.0 s | 0.3 s | L154 |

### 两个关键设计点

1. **无缝拼接**
   单步幅度 = 速率 × 重规划周期，轨迹时长也等于该周期。
   smoothstep 走到头、速度已降到 0 时下一帧轨迹刚好接手，两端零速度，拼接无突变。

2. **用指令位置做基准（`_cmd_q`）**
   连续重规划从"上一次下发的目标"累加，而不是从滞后的串口反馈重新起步。
   否则反馈落后约 100 ms 时，每步都从落后位置出发，会走不动或反向。
   声明位置：`_cmd_q` 状态变量；使用位置：`_align_step` 的 `base_q`、`_send_positions` 的 `q0`。
   目标被拒或轨迹报错时清空 `_cmd_q`，下一步退回反馈基准，避免指令位置漂移。

---

## 5. 验证结果

运行 `ros2 launch wheeltec_mycobot_pick_place_aruco aruco_align.launch.py`：

| 检查项 | 结果 |
|---|---|
| `aruco_align_node` 运行 | OK，识别 ID=1 |
| `aim` 日志下发频率 | **19.91 Hz**（`_last_send` 递增，节流生效） |
| 无报错 | 20 s 窗口内 0 条 goal rejected / error_code |
| 轨迹消息 | 20 点 smoothstep，末段位移约 0.0001 rad |
| `speed_hint` | **36.8 ms 周期**（静止→40，快速移动→80） |
| 驱动发送 | 40 Hz，`send_angles` 用实时 speed |

---

## 6. 调参速查

| 现象 | 调整 | 位置 |
|---|---|---|
| 仍有轻微顿挫 | `replan_interval` 0.15 → 0.2~0.25 | `aruco_align_node.py` L157 |
| 末端不够柔 | `ALIGN_WAYPOINTS` 20 → 24 | 同文件 L71 |
| 整体太快/太慢 | `speed_slow/speed_fast` | 同文件 L168-169 |
| 驱动固定速度 | `speed=50` | `all.launch.py` L275 |

修改 `aruco_align_node.py` 后需编译（非软链接安装）：

```bash
colcon build --packages-select wheeltec_mycobot_pick_place_aruco
```

`mycobot_driver_update_node.py` 改动后：

```bash
colcon build --packages-select mycobot280_moveit2_control
```

---

## 7. 链路图

```
/camera/color/image_raw
  → aruco_ros single (ID=1)
  → /aruco_single/pose
  → aruco_align_node
       ├─ 差分测速 → /mycobot/speed_hint ──────┐
       └─ 20 Hz 重规划 + 抢占                    │
            └─ smoothstep 多点轨迹              │
                 → /arm_controller/follow_joint_trajectory
                      → JTC (100 Hz / 命令 50 Hz)
                           → mycobot_driver_update_node (40 Hz)
                                └─ send_angles(tgt, speed_hint) ←┘
```

---

## 8. 尚未做的后续项

- 跟踪时相机固定曝光/ROI 检测（降低视觉延迟与位姿抖动）
- JTC `path` 容差 0.02 → 0.005，减少末端小点被跳过
- 驱动 `_read_state` 10 Hz 抢串口锁，跟踪期间可降到 2-5 Hz
- `aruco_align_node` 急停时尚未主动取消自身轨迹（驱动只刹固件）
