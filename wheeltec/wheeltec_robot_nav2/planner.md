# 导航规划器说明（wheeltec_nav2）

本文说明 `ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py` 启动后，导航实际使用的**全局规划器**与**局部规划器（控制器）**算法，以及如何配置、切换和验证它们。

- 启动文件：`src/wheeltec/wheeltec_robot_nav2/launch/wheeltec_nav2.launch.py`
- 参数文件：`src/wheeltec/wheeltec_robot_nav2/param/wheeltec_param/param_four_wheel_diff_bs.yaml`

---

## 1. 启动链路：参数是怎么到达规划器的

| 环节 | 位置 | 说明 |
|---|---|---|
| 选择参数文件 | wheeltec_nav2.launch.py L43 | `my_param_file = 'param_four_wheel_diff_bs.yaml'` |
| 声明为 launch 参数 | wheeltec_nav2.launch.py L87-90 | `params_file`，默认 = `param/wheeltec_param/param_four_wheel_diff_bs.yaml` |
| 启动底盘与雷达 | wheeltec_nav2.launch.py L102-107 | `turn_on_wheeltec_robot.launch.py` + `wheeltec_lidar.launch.py` |
| 定位栈 | wheeltec_nav2.launch.py L151-160 | 引入 `localization_launch.py`（`map` 传入地图路径） |
| 导航栈 | wheeltec_nav2.launch.py L162-169 | 引入 `navigation_launch.py` |
| 定位节点 | localization_launch.py L46 / L89-99 | `map_server` + `amcl`，由 `lifecycle_manager_localization` 拉起 |
| 导航节点 | navigation_launch.py L45-49 / L91-129 | `controller_server`、`planner_server`、`behavior_server`、`bt_navigator`、`waypoint_follower` |
| 参数注入 | navigation_launch.py L60-64 | 用 `RewrittenYaml` 覆盖 `use_sim_time` / `autostart` / `map_subscribe_transient_local` 后下发 |

小结：**launch 文件本身不含任何规划算法参数**，所有算法与阈值都来自上表第 2 行的 YAML。改算法 = 改 YAML（或换 YAML）。

导航数据流：

```
地图(WHEELTEC.yaml) ─→ global_costmap ─→ planner_server(NavFn) ─→ /plan
                                                        │
                              bt_navigator(行为树) ──FollowPath──┘
                                                        ↓
                     local_costmap ─→ controller_server(DWB) ─→ /cmd_vel ─→ 底盘 wheeltec_robot
```

---

## 2. 全局规划器：NavFn（`planner_server`）

### 2.1 当前配置

- 节点：`nav2_planner` → `planner_server`
- 插件键名：`GridBased`，类型 `nav2_navfn_planner::NavfnPlanner`
- 位置：参数文件 **L246-255**

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5        # L253  目标点允许容差(米)
      use_astar: false      # L254  false=Dijkstra, true=A*
      allow_unknown: true   # L255  是否允许穿越未知栅格
```

### 2.2 算法原理

NavFn 是**栅格上的最短路径搜索**，不是采样式规划：

1. 把 `global_costmap` 的代价栅格（含膨胀层代价）读成一维代价值数组。
2. 从目标点向四周做**势场/代价传播**（Dijkstra 展开；`use_astar: true` 时换成 A* 启发式，更快但路径可能更贴障碍）。
3. 代价函数同时考虑**路径长度**与**代价栅格值**，因此路径会自然避开膨胀区、偏向低代价通道。
4. 起点到目标的反向梯度即为输出路径，发布到 `/plan`（`nav_msgs/Path`）。

关键参数语义：

| 参数 | 行 | 语义 | 调大/调小的影响 |
|---|---|---|---|
| `tolerance` | L253 | 目标点若落在障碍/不可达栅格，允许搜索最近的可达栅格作为终点 | 调大：更容易规划成功，但终点可能偏离设定点；调小：严格但更易规划失败 |
| `use_astar` | L254 | Dijkstra / A* 切换 | A* 快，Dijkstra 路径更平滑、代价更优 |
| `allow_unknown` | L255 | 未知栅格（-1）能否穿越 | `true`：地图边缘未建图区域也能规划通过（易"穿墙"）；`false`：陌生区域直接规划失败，更安全 |
| `expected_planner_frequency` | L248 | 期望规划频率，仅用于统计/告警 | 与 BT 重规划频率匹配即可 |

### 2.3 依赖：全局代价地图

- 位置：参数文件 **L192-225**
- 图层（L203）：`static_layer`（静态地图，L218-220）+ `obstacle_layer`（激光实时障碍，L204-217）+ `inflation_layer`（膨胀，L221-224）
- `global_frame: map`（L197）、`resolution: 0.05`（L201）、`update_frequency: 1.0`（L195）
- 障碍来源：`/scan`（L209），`obstacle_max_range: 2.5`、`raytrace_max_range: 10.0`
- 膨胀参数：`inflation_radius: 0.2`、`cost_scaling_factor: 3.0`（L222-223）——**直接影响 NavFn 生成的路径离墙多远**

---

## 3. 局部规划器 / 控制器：DWB（`controller_server`）

### 3.1 当前配置

- 节点：`nav2_controller` → `controller_server`
- 插件键名：`FollowPath`，类型 `dwb_core::DWBLocalPlanner`
- 位置：参数文件 **L75-144**

速度与运动学（L107-122）：

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  min_vel_x: -0.5      # L107 后退速度上限
  max_vel_x: 0.5       # L109 前进速度上限（主速度开关）
  max_vel_theta: 1.5   # L111 最大角速度 rad/s
  max_speed_xy: 0.5    # L113 合成线速度上限，需 >= max_vel_x
  acc_lim_x: 0.25      # L117 线加速度上限
  acc_lim_theta: 1.0   # L119 角加速度上限
  decel_lim_x: -0.5    # L120 线减速度上限
  decel_lim_theta: -1.0# L122 角减速度上限
```

采样与前瞻（L123-131）：

```yaml
  vx_samples: 20        # L123 线速度采样份数
  vy_samples: 0         # L124 差速底盘无侧移
  vtheta_samples: 20    # L125 角速度采样份数
  sim_time: 1.7         # L126 轨迹仿真前瞻时间(s)
  linear_granularity: 0.05   # L127 线速度量化步长(m/s)
  angular_granularity: 0.025 # L128 角速度量化步长(rad/s)
  transform_tolerance: 0.2   # L129 TF 允许滞后(s)
  xy_goal_tolerance: 0.25    # L130 位置到点容差(m)
  trans_stopped_velocity: 0.25 # L131 视为"已停"的速度阈值
  short_circuit_trajectory_evaluation: True  # L132 评分提前终止
  stateful: True        # L133 保留状态，避免抖动
```

### 3.2 算法原理（DWA / Dynamic Window）

DWB 是**基于采样的局部避障控制器**（Dynamic Window Approach），每周期执行：

1. **构造动态窗口**：由 `acc_lim_*` / `decel_lim_*` 限定本周期可达的速度区间，即"动态窗口"。
2. **速度空间采样**：在 `[min_vel_x, max_vel_x]` 取 `vx_samples` 份、`[-max_vel_theta, max_vel_theta]` 取 `vtheta_samples` 份（差速车 `vy_samples: 0`），并按 `linear_granularity` / `angular_granularity` 量化。
3. **前向仿真**：每条 (vx, vtheta) 组合用 `sim_time` 秒定步长前向模拟出一条轨迹，同时用机器人轮廓（footprint）做碰撞检查。
4. **Critic 打分**：每条轨迹按一组 critic 加权求和，取总分最优者。
5. **发布**：最优轨迹的首个速度指令发到 `/cmd_vel`。

当前 critic 集合与权重（L134-144）：

| Critic | 权重 | 行 | 作用 |
|---|---|---|---|
| `RotateToGoal` | 32.0 | L142-144 | 接近终点时优先原地对准目标朝向（`slowing_factor: 5.0`，`lookahead_time: -1.0`） |
| `Oscillation` | 默认 | L134 | 惩罚左右来回摆动，防抖 |
| `BaseObstacle` | 0.04 | L135 | 车体与障碍的碰撞代价，**分数低说明避障权重弱** |
| `GoalAlign` | 10.0 | L137 / L139 | 朝向目标点的加成（前视点距离 0.1m） |
| `PathAlign` | 50.0 | L136 / L138 | 与全局路径的朝向一致性（权重最高之一） |
| `PathDist` | 50.0 | L140 | 与全局路径的距离，保证贴合路径 |
| `GoalDist` | 10.0 | L141 | 与目标点的距离，保证持续推进 |

> 调参经验：路径贴合由 `PathAlign/PathDist` 主导；想更"胆大"贴障碍走 → 降低 `BaseObstacle.scale` 或缩小膨胀半径；想更保守 → 提高 `BaseObstacle.scale`、加大 `inflation_radius`。

### 3.3 两个检查器（决定"目标何时失败/成功"）

| 检查器 | 参数 | 行 | 语义 |
|---|---|---|---|
| 进度检查 | `required_movement_radius: 0.5`、`movement_time_allowance: 10.0` | L88-91 | **10 秒内必须移动 0.5m**，否则报 `Failed to make progress` 并中止本段路径 |
| 到点检查 | `xy_goal_tolerance: 0.25`、`yaw_goal_tolerance: 0.25`、`stateful: True` | L98-102 | 位置 0.25m 且朝向 0.25rad 内视为到达 |

⚠️ 二者与速度强耦合：**降速时必须同步降 `required_movement_radius`**，否则低速（如 0.05m/s）在 10 秒内走不满 0.5m，会被进度检查器判定失败。

### 3.4 依赖：局部代价地图

- 位置：参数文件 **L150-184**
- 图层（L164）：`obstacle_layer`（L169-182）+ `inflation_layer`（L165-168），**无静态层**（滚动窗口）
- `global_frame: odom_combined`（L155）、`rolling_window: true`、`width/height: 3×3m`（L158-160）、`resolution: 0.05`（L161）、`update_frequency: 5.0`（L153）
- `footprint`（L163）：当前为整车轮廓 `[[-0.15,-0.38],[-0.15,0.38],[0.67,0.38],[0.67,-0.38]]`，即 **0.82m × 0.76m**（含机械臂与云台），比底盘本体大很多

---

## 4. 行为树与恢复行为（规划失败后发生什么）

- `bt_navigator`：参数文件 **L56-70**；默认行为树为
  `nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml`（见 L67-70 注释），即"周期重规划 + 失败恢复"。
- 可用行为树（本机 `/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/`）：
  `navigate_to_pose_w_replanning_and_recovery.xml`、`navigate_through_poses_w_replanning_and_recovery.xml`、`navigate_to_pose_w_replanning_goal_patience_and_recovery.xml`、`navigate_w_replanning_distance.xml`、`navigate_w_replanning_time.xml`、`navigate_w_replanning_speed.xml`、`navigate_w_replanning_only_if_path_becomes_invalid.xml`、`nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml`、`follow_point.xml`、`odometry_calibration.xml`
- 恢复动作由 `behavior_server` 提供（L261-280）：`spin`（原地转）、`backup`（后退）、`wait`（等待）
  - `max_rotational_vel: 0.3`、`min_rotational_vel: 0.05`、`rotational_acc_lim: 0.3`（L278-280）
- ⚠️ **已知配置缺陷**：Jazzy 的 `behavior_server` 用 `local_frame` 指定局部参考系（默认 `odom`），而本车里程计帧是 **`odom_combined`**。参数文件只写了 `global_frame: odom_combined`（L273），**缺 `local_frame: odom_combined`**，运行时恢复行为会报
  `No Transform available ... target frame: "odom" ... does not exist` → `Current robot pose is not available` → 恢复动作失败。

---

## 5. 如何配置规划器

### 5.1 改参数文件（第一步）

方法一：编辑 [param_four_wheel_diff_bs.yaml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_nav2/param/wheeltec_param/param_four_wheel_diff_bs.yaml) 后重启导航栈。

方法二：启动时换文件（不改现有配置）：

```bash
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py \
  params_file:=/abs/path/to/你的.yaml
```

或改 launch 文件 L43 的 `my_param_file` 指向包内其它 YAML。注意 `param/wheeltec_param/` 下按车型分文件（`param_four_wheel_diff_bs.yaml` 等），换文件等于换整车运动学配置。

### 5.2 切换全局规划器

改 **L250-255** 两个键：`planner_plugins`（插件键名列表）+ 对应插件键。

本机已安装可用的全局规划器类型：

| 类型 | 说明 |
|---|---|
| `nav2_navfn_planner::NavfnPlanner` | 当前使用；栅格最短路径，稳、路径平滑 |
| `nav2_smac_planner::SmacPlanner2D` | 2D A*，考虑运动学约束少，速度较快 |
| `nav2_smac_planner::SmacPlannerHybrid` | **混合 A*（考虑车辆运动学）**，适合差速/阿克曼，路径更可执行 |
| `nav2_smac_planner::SmacPlannerLattice` | 状态栅格，可携带更复杂状态 |
| `nav2_theta_star_planner::ThetaStarPlanner` | Theta*，接近任意角路径，直线段更连贯 |

示例（改用混合 A*，注意该插件需要 `minimum_turning_radius` 等自身参数）：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      tolerance: 0.5
      allow_unknown: true
      minimum_turning_radius: 0.4
      angle_quantization_bins: 64
      smooth_path: true
```

### 5.3 切换局部规划器（控制器）

改 **L85** 的 `controller_plugins` 与 **L104-105** 的插件键：

| 类型 | 说明 |
|---|---|
| `dwb_core::DWBLocalPlanner` | 当前使用；DWA 采样 + 多 critic 打分，参数多、可调性最强 |
| `nav2_mppi_controller::MPPIController` | MPPI 采样优化，轨迹更顺滑，对速度自适应好，CPU 占用较高 |
| `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | 纯跟踪 + 速度调节，参数少、平滑，适合长廊 |
| `nav2_graceful_controller::GracefulController` | 平滑速度曲线，兼顾贴障碍通行 |
| `nav2_rotation_shim_controller::RotationShimController` | 转向过渡壳，通常**包在其它控制器外**用 |

示例（换 MPPI，其参数键与 DWB 完全不同，需按官方样例重写 `FollowPath:` 段）：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      vx_max: 0.5
      vx_min: -0.35
      wz_max: 1.9
```

> 注意：切控制器要**整段替换** `FollowPath` 下的参数，DWB 的 `critics`/`*_samples` 等键对新控制器无效。切换后还要确认 `xy_goal_tolerance`、`transform_tolerance` 等通用键是否被新插件识别。

### 5.4 速度、加速度、避障力度分别改哪

| 目标 | 参数 | 行 |
|---|---|---|
| 前进速度上限 | `FollowPath.max_vel_x`（同步 `max_speed_xy`） | L109 / L113 |
| 低速下可用采样精度 | `linear_granularity`（建议随速度调小，如 0.01） | L127 |
| 起步/刹车快慢 | `acc_lim_x` / `decel_lim_x` | L117 / L120 |
| 转向快慢 | `max_vel_theta` / `acc_lim_theta` | L111 / L119 |
| 离墙距离（全局） | `global_costmap.inflation_layer.inflation_radius` | L223 |
| 离墙距离（局部） | `local_costmap.inflation_layer.inflation_radius` | L168 |
| 贴障碍通行激进程度 | `FollowPath.BaseObstacle.scale` | L135 |
| 贴路径紧密程度 | `PathAlign.scale` / `PathDist.scale` | L136 / L140 |
| 到点精度 | `general_goal_checker.xy_goal_tolerance` / `yaw_goal_tolerance` | L101-102 |
| 防"卡住不算失败" | `progress_checker.required_movement_radius` / `movement_time_allowance` | L90-91 |
| 车体轮廓（决定可通行性） | `footprint`（local L163、global L202） | — |
| 换行为树 | `bt_navigator.default_nav_to_pose_bt_xml`（当前用默认，见 L67-70） | — |
| 补齐恢复行为坐标系 | `behavior_server.local_frame: odom_combined`（**建议尽快加**） | L273 附近 |

### 5.5 生效与验证

1. **生效方式**：以上参数在节点启动时读取 → 改 YAML 后必须**重启导航栈**。少量参数可用 `ros2 param set /controller_server FollowPath.max_vel_x 0.3` 在线试探，但重启更可靠。
2. **确认插件加载成功**（日志里应有对应行）：
   ```bash
   ros2 lifecycle get /planner_server
   ros2 lifecycle get /controller_server
   ```
   planner 日志：`Created global planner plugin GridBased of type nav2_navfn_planner::NavfnPlanner`
   controller 日志：`Created controller : FollowPath of type dwb_core::DWBLocalPlanner`
3. **看实际输出**：
   ```bash
   ros2 topic echo /plan --once          # 全局路径
   ros2 topic echo /cmd_vel              # 局部控制器速度指令
   ros2 topic echo /local_plan --once    # DWB 局部轨迹
   ```
4. **RViz 可视化**：Global/Local Costmap、Path、Local Plan 显示项；本工程 RViz 配置为 `wheeltec_robot_nav2/rviz/wheeltec_nav2.rviz`。
5. **典型失败信号对照**：

| 现象 | 含义 | 对应处理 |
|---|---|---|
| `Failed to make progress` | 10s 内没走够 0.5m（速度过低/被卡） | 降 `required_movement_radius` 或提速度 |
| `No valid trajectories out of N!` | 所有采样轨迹都被判碰撞 | 缩小 footprint / 减小膨胀半径 / 放宽速度 |
| `Transform data too old` | TF 滞后超过 `transform_tolerance`（L129） | 检查 `/scan` 与 TF 时间戳、加大允许值 |
| `target frame "odom" ... does not exist` | `behavior_server` 坐标系配错 | 加 `local_frame: odom_combined` |
| 规划路径穿未知区域 | `allow_unknown: true`（L255） | 改 `false` 限制在地图内规划 |

---

## 6. 与定位/地图的关系（规划器的输入前提）

- `amcl`（参数 L1-46）：`odom_frame_id: "odom_combined"`（L23）、`base_frame_id: "base_footprint"`（L9）、`laser_model_type: "likelihood_field"`（L19）、初始位姿 `(0,0,0)`（L42-46）→ 发布 `map → odom_combined`。
- TF 链：`map →(amcl)→ odom_combined →(EKF)→ base_footprint →(URDF)→ base_link → laser`。规划器与代价地图都依赖该链完整。
- 地图：`map_server` 的 `yaml_filename` 在参数文件里是 `turtlebot3_world.yaml`（L236），但**运行时被 launch 注入的地图覆盖**为 `wheeltec_nav2/map/WHEELTEC.yaml`（localization_launch.py L54 重写 `yaml_filename`）。

---

## 7. 快速调参入口一览

```text
想让小车走慢一点
  → FollowPath.max_vel_x (L109) + max_speed_xy (L113) + linear_granularity (L127)
  → 同时降 progress_checker.required_movement_radius (L90)

想让小车离墙远一点
  → local/global inflation_layer.inflation_radius (L168 / L223)

想让小车更贴障碍走
  → BaseObstacle.scale (L135) 调小 / inflation_radius 调小

想禁止/允许规划路径穿越未知区域（两个参数必须同时改，只改一个无效）
  → global_costmap.track_unknown_space (L205) + planner_server.GridBased.allow_unknown (L262)
  → 允许穿越：track_unknown_space: false + allow_unknown: true
  → 禁止穿越:track_unknown_space: true  + allow_unknown: false
    注意: track_unknown_space: false 时 unknown 已被 costmap 抹成 FREE_SPACE(0)，
          单改 allow_unknown: false 不起作用，路径照样穿过未知区域

想换更聪明的全局路径
  → GridBased.plugin = nav2_smac_planner::SmacPlannerHybrid (L252)

想换更顺滑的局部控制
  → FollowPath.plugin = nav2_mppi_controller::MPPIController (L105)

想换更省心的跟踪
  → FollowPath.plugin = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController (L105)

想换重规划策略
  → bt_navigator: default_nav_to_pose_bt_xml 指向 L4 所列行为树之一
```
