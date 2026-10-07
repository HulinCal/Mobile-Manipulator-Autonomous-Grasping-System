# 4. 抓取节点 (grasp_node)

> 本文对应当前真机实现（MoveIt2 + ros2_control + pymycobot），抓取/放置动作均已在
> wheeltec 移动底盘 + mycobot280 + 自适应夹爪上验证通过。

## 功能

通过 MoveIt2 的 `MoveGroup` action 完成苹果的抓取，并通过
`GripperCommand` action 控制自适应夹爪。节点是纯编排层：自身不直接操作串口，
所有关节/夹爪指令都经 ros2_control 控制器下发（见文末[数据流](#数据流)）。
放置服务（`/pick_place/place`）保留供手动调用，但**自动流水线抓取后只收臂、回程，
不放置、不张爪**。

- 位姿来源：订阅 `/pick_place/apple_pose`（`base_link` 系，检测节点发布，latched）
- 规划组：`arm`（SRDF，6 个臂关节 + `link6_flange_to_gripper_base`，不含夹爪关节）
- 夹爪组：`gripper`（单关节 `gripper_controller`）
- 末端链接：`link6_flange`；机械臂基座：`link1`；世界系：`base_link`

## 抓取流程 (`/pick_place/pick`)

按以下顺序严格串行执行，每一步失败即返回失败（部分 MoveIt 目标会自动重试一次）：

1. **回 home** — 6 个臂关节运动到全零（`_move_to_joint_values([0]*6)`），
   作为每次抓取的统一初始姿态。`grasp_init_only:=true` 时到这一步即返回（分步调试用）。
2. **夹爪张到最大** — 发 `gripper_open_value=0.15 rad`（映射 pymycobot **100 全开**），
   在向苹果做任何运动之前完成，保证下降前手指完全张开。
3. **移动到 pre-grasp（苹果上方）** — 俯抓姿态（法兰 z 轴竖直朝下），目标在苹果
   正上方 `grasp_approach_distance`(0.10 m) 处。两个等价翻转朝向四元数
   `(y=1,w=0)`、`(x=1,w=0)` 依次尝试；先试正上方（inner=0），臂展极限不可达时再
   内收 `pre_grasp_inner`(0.06 m)。姿态容差放宽到 0.15 rad。
   **候选试错用短规划时限 `explore_planning_time`(1.5 s) 且不逐个重试**——不可达
   目标约 1.5 s 即放弃切下一个（旧逻辑每个候选烧满 5 s 且重试一次，臂展极限处
   单 pre-grasp 就要空耗 ~20 s）；全部短时限失败后，才对内收位姿做一次完整
   `planning_time`(5 s)+重试兜底。运动学求解器为 **TRAC-IK**（kinematics.yaml，
   `solve_type: Speed`），可达目标通常 <0.5 s 出解。
4. **缓慢下降到苹果高度** — 同一目标去掉上方偏移，**优先沿用 pre-grasp 已验证
   可达的法兰朝向**（不再固定从 q=y 试起），`vel_scale=0.1`（速度/加速度约为
   平时的 1/3），避免冲撞苹果；同样 1.5 s 短时限试错 + 完整时限兜底。
5. **物理到位 & 静止确认（关键）** — `_wait_flange_settled()`：MoveGroup 报成功只代表
   控制器轨迹走完，固件最后一段运动会滞后约 1~2 s。此处用 TF（`link1→link6_flange`，
   源自 `/joint_states` 真实关节）判定：
   - 法兰与目标位置误差 < 12 mm（真到位）；
   - 连续两次（间隔 50 ms）读数位移 < 2 mm（确实停稳）。
   - 超时 6 s 未满足则**补发一次目标位姿**再等，仍不满足判失败。确认停稳后才继续，
     杜绝"还在下降途中就闭合"。
6. **闭合夹爪到约 1/3 张开** — 发 `gripper_close_value=-0.42 rad`（映射 **value 33**），
   不夹到机械硬限位；到位后再 `sleep 2 s` 等机械夹紧（自适应夹爪无真实位置反馈，
   `get_gripper_value` 返回的是指令目标值，action 会提前报成功）。
7. **两段式提升（先直提、后内收）**：
   - 阶段 A：**纯竖直小幅提升**（inner=0，候选 0.08/0.05/0.03 m，vel 0.15），
     让苹果先垂直安全离地。必须单独做这一段：OMPL 自由规划走关节空间弧线，
     直接去"升高+内收"目标时轨迹起始段会先小幅下沉再上升（真机曾观察到
     "夹住后下降一点点"），夹着苹果下探有磕碰桌面风险；
   - 阶段 B：再到运输位姿，候选 `(0.15,0.10)` → `(0.20,0.12)` → `(0.10,0.08)`
     （同样 1.5 s 短时限试错，全失败后对 `(0.15,0.10)` 做一次 5 s 兜底）。
     此时苹果已有离地余量，弧线轻微下探不再碰到桌面。苹果在臂展极限
     （link1 y≈0.27）时纯竖直不可达，内收上限约 0.12 m，过大（曾用 0.22）
     会让夹着的苹果扫撞机械臂本体。阶段 A 若全部失败，阶段 B 会把纯竖直
     `lift_distance` 目标放回候选首位兜底。
8. **收臂回 home（在小车回程之前）** — 夹爪保持 33 不松爪，臂关节收回全零，再
   `sleep 2 s` 等物理收稳。随后 pick 服务才返回，由协调器让小车倒回起点，避免苹果
   举在臂展极限处随车晃动/碰撞/掉落。

## 放置流程 (`/pick_place/place`)

> 注意：**自动流水线已不再调用 place**。当前需求是回起点即结束，苹果保持夹在
> 机械臂上、夹爪不张开，由现场人工取走。以下流程仅在手动调用
> `/pick_place/place` 服务时执行。

回到起点后执行，放置点用抓取时记录的最终苹果位姿（`base_link` 系，机械臂必可达）：

1. 到放置点上方 `place_approach_distance`(0.10 m)；
2. 低速（vel 0.1）下降到放置高度；
3. 夹爪张到 0.15 rad（**100 全开**）释放苹果；
4. 上抬 `retreat_distance`(0.10 m) 退出。

## 夹爪开合度映射（重要）

`gripper_controller` 关节指令区间为 **[-0.7, 0.15] rad**，驱动节点将其线性映射为
pymycobot `set_gripper_value` 的 0..100：

| 语义 | 关节值 [rad] | pymycobot value | 用途 |
|------|-------------|-----------------|------|
| 全开 | 0.15 | 100 | 抓取前张开 / 放置时释放 |
| 约 1/3 张开 | **-0.42** | **33** | **夹取苹果（当前值）** |
| 半开（SRDF half_closed） | -0.275 | 50 | 夹不住，勿用 |
| 全闭（硬限位） | -0.7 | 0 | 物理只能到 -0.683，会停滞二次发力，勿用 |

> 真机曾观察到"夹两下"：目标给 0 时夹爪顶在硬限位上进不了到位容差，控制器在停滞
> 窗口内持续顶住，固件出现一次再收紧脉冲。改为可达的 33 后一次到位、无二次动作。

## 数据流

```
检测节点 ──/pick_place/apple_pose(PoseStamped, base_link, latched)──► grasp_node
                                                                      │  +标定偏移(pose_x_offset 等)
                                                                      │  TF: base_link → link1
                                                                      │  +TCP 补偿 tcp_flange_z
                                                                      ▼
                                                  法兰目标位姿 (link1 系)
                                                                      │
              ┌───────────────────────────────────────────────────────┴───────────────┐
              ▼ 臂运动                                                                   ▼ 夹爪
  MoveGroup action /move_action                                    GripperCommand action
  (MoveIt OMPL 规划 → FollowJointTrajectory)                       /gripper_action_controller/gripper_cmd
              │                                                                   │
              ▼ arm_controller (joint_trajectory_controller)                       ▼ gripper_action_controller
        ros2_control 硬件插件 (mycobot280_hardware_interface)               ros2_control 硬件插件
   50Hz /mycobot/cmd_joint_pos  (6 关节 rad)                    50Hz /mycobot/cmd_gripper_pos (rad)
              │                                                                   │
              ▼                              mycobot_driver_node.py                 ▼
   回调只存最新目标 → 独立 arm worker          (threading.Lock 共用串口)   回调只存最新目标 → gripper worker
   sync_send_angles(deg, speed=80, timeout=10)                   set_gripper_value(0..100, speed=30)
              │                                                          (0.15 s 稳定性闸门滤陈旧指令)
              └──────────────────────────────► /dev/mycobot_controller ◄──────────────┘

  状态回传 (20 Hz)：pymycobot get_angles() / get_gripper_value()
      → /mycobot/current_joint_pos, /mycobot/current_gripper_pos
      → ros2_control 硬件插件 → joint_state_broadcaster → /joint_states
      → robot_state_publisher → TF（grasp_node 的到位判定与 RViz 均用此真实状态）
```

关键实现要点（均在 `mycobot_driver_node.py`）：

- **异步串口 worker**：臂、夹爪的订阅回调只更新"最新目标"后立即返回，绝不阻塞
  executor 线程；各自由独立守护线程在串口空闲时下发。否则机械臂阻塞式
  `sync_send_angles(timeout=10)` 会占满 `MultiThreadedExecutor` 的 4 个线程，夹爪
  订阅/状态读被饿死，张合指令延迟十几秒。
- **命令去重**：臂目标与上次下发逐关节差 < 0.02° 跳过（到位后 50 Hz 恒定指令不再
  占串口）；夹爪目标带 0.15 s 稳定性闸门，滤除串口繁忙期积压的陈旧开合指令。
- `gripper_action_controller` 的 `stall_timeout` 已由 1.0 s 放宽到 **5.0 s**
  （夹爪 100 到 33 物理约需 1.5~2 s）。注意 `allow_stalling:=false` 时停滞仍返回
  action SUCCEEDED 且 `stalled=true`。

## 话题 / 服务 / 动作

| 类型 | 名称 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/pick_place/apple_pose` | `PoseStamped` (TRANSIENT_LOCAL) | 苹果位姿，base_link 系 |
| 订阅 | `/pick_place/place_pose` | `PoseStamped` | 放置目标位姿 |
| 发布 | `/pick_place/status` | `String` | 各阶段人读状态文本 |
| 服务 | `/pick_place/pick` | `Trigger` | 执行完整抓取（含收臂回 home） |
| 服务 | `/pick_place/place` | `Trigger` | 执行放置 |
| 服务 | `/pick_place/arm_home` | `Trigger` | 仅机械臂回 home（6 关节零位） |
| 服务 | `/pick_place/arm_stop` | `Trigger` | 机械臂急停（语音"机械臂停止"），见下 |
| 发布 | `/mycobot/stop` | `Empty` | arm_stop 时向固件发急停（mycobot_driver 订阅） |
| 动作(客户端) | `/move_action` | `MoveGroup` | MoveIt 臂规划执行 |
| 动作(客户端) | `/gripper_action_controller/gripper_cmd` | `GripperCommand` | 夹爪开合 |

### 机械臂急停（`/pick_place/arm_stop`）

两层停止，保证臂**立即不再作任何运动**：

1. **取消在途 MoveGroup goal**：正在 `_send_move_goal` 中等待的抓取/回 home
   序列拿到取消结果（非 SUCCESS）后中止，不再继续后续下降、闭合、提升；
2. **固件急停**：发布一次 `/mycobot/stop`，`mycobot_driver` 调
   `pymycobot.stop()` 立即停固件（goal 取消后固件仍可能有 1~2 s 残留运动），
   并原地把 worker 阻塞中的目标改成当前实际角，让 `sync_send_angles` 立刻
   判"到位"返回释放串口锁；随后 ros2_control 桥 50 Hz 发来的保持位置命令
   只产生一次原地目标，无可见运动。

臂静止时调用同样安全（无在途 goal 时只发一次幂等的固件 stop）。
急停后如需继续，重新调用 `/pick_place/arm_home` 或相应流程即可。

## 参数（`config/pick_place_params.yaml` 生效值）

| 参数 | 值 | 说明 |
|------|-----|------|
| `sim_mode` | `false` | 真机模式；`true` 时无需 move_group，按 `sim_duration` 假装执行 |
| `arm_group` / `gripper_group` | `arm` / `gripper` | SRDF 规划组 |
| `arm_base_frame` | `link1` | 机械臂基座系（规划目标系） |
| `eef_link` | `link6_flange` | 末端链接（到位判定用） |
| `world_frame` | `base_link` | 世界系（苹果位姿系） |
| `tcp_flange_z` | `0.12` | 法兰到夹持中心距离 [m]，补偿夹爪长度，按实测微调 |
| `pose_x/y/z_offset` | `0.09 / 0 / 0` | 视觉/手眼标定残差，base_link 系加偏移 [m] |
| `gripper_open_value` | `0.15` | 张开（value 100） |
| `gripper_close_value` | `-0.42` | 夹取（value 33，约 1/3 张开） |
| `grasp_approach_distance` | `0.10` | pre-grasp 在苹果上方高度 [m] |
| `pre_grasp_inner` | `0.06` | pre-grasp link1-y 内收 [m] |
| `lift_distance` | `0.25` | 期望提升高度 [m]（不可达时按内置候选降级） |
| `place_approach_distance` | `0.10` | 放置前上方高度 [m] |
| `retreat_distance` | `0.10` | 松爪后上抬 [m] |
| `grasp_init_only` | `false` | `true` 时 pick 到 home 即返回（分步调试） |
| `planning_time` | `5.0` | 完整规划时限 [s]（仅兜底重试/关节目标用） |
| `explore_planning_time` | `1.5` | 候选位姿试错单次规划时限 [s]，缩短不可达目标的空耗 |

## 模拟测试

```bash
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py sim_mode:=true

ros2 topic pub --once /pick_place/apple_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}"
ros2 service call /pick_place/pick std_srvs/srv/Trigger
ros2 service call /pick_place/place std_srvs/srv/Trigger
ros2 service call /pick_place/arm_home std_srvs/srv/Trigger
ros2 service call /pick_place/arm_stop std_srvs/srv/Trigger
```

## 真机使用

```bash
# 默认即真机（sim_mode 默认 false），由聚合 launch 拉起整车 bringup / move_group / ros2_control
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py
```

MoveIt 配置复用 `mycobot_moveit_config` 的 SRDF（arm / gripper 组、home / open /
half_closed 命名位姿），`all.launch.py` 已负责启动 move_group、controller_manager
并依次激活 `joint_state_broadcaster`、`arm_controller`、`gripper_action_controller`。
