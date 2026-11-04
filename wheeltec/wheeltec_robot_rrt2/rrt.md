# RRT 自主探索建图使用与实现说明

适用包：`wheeltec_robot_rrt`（源码目录 `src/wheeltec/wheeltec_robot_rrt2`）
运行环境：ROS 2 Jazzy + Nav2 + slam_toolbox
消息包：`wheeltec_rrt_msg`

---

## 1. 功能概述

本包在 slam_toolbox 建图的基础上，用 RRT（Rapidly-exploring Random Tree）自动从当前地图中寻找**前沿点（frontier，已探索自由区与未知区的交界）**，选出一个信息增益最高、代价最小的点，交给 Nav2 导航过去；车到位后地图扩大，再产生新的前沿点，如此循环，直到地图不再有可探索的前沿。

整条链路无人干预，只需要在启动后给出**探索范围边界**（4 个点）+ **起点**（第 5 个点）。

```
/clicked_point (5点)
        │
        ▼
  global_rrt / local_rrt  ──/detected_frontiers──▶  filter  ──/filtered_goal_points──▶  assigner
                                                                                        │
                                                              /assigned_frontier_goal ◀─┘
                                                                        │
                                                                        ▼
                                                        find_robot_action_server
                                                                        │ navigate_to_pose
                                                                        ▼
                                                                     Nav2
                                                                        │ /cmd_vel
                                                                        ▼
                                                                      底盘
```

---

## 2. 包结构

| 路径 | 作用 |
|---|---|
| [launch/rrt_exploration/wheeltec_rrt_slam.launch.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/launch/rrt_exploration/wheeltec_rrt_slam.launch.py) | **总启动文件（step2）** |
| [launch/rrt_exploration/rrt_exploration.launch.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/launch/rrt_exploration/rrt_exploration.launch.py) | 启动 RRT 四节点 |
| [launch/rrt_exploration/action_servers.launch.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/launch/rrt_exploration/action_servers.launch.py) | 启动 find / pick 两个 action server |
| [launch/navigation/nav2_bringup_launch.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/launch/navigation/nav2_bringup_launch.py) | Nav2 总入口（本包自带一套参数） |
| [launch/navigation/nav2_navigation_launch.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/launch/navigation/nav2_navigation_launch.py) | 启动 Nav2 五个服务端 + 一个 lifecycle manager |
| [src/rrt_exploration/global_rrt.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/global_rrt.cpp) | 全局 RRT，在边界框内生长 |
| [src/rrt_exploration/local_rrt.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/local_rrt.cpp) | 局部 RRT，每次都从机器人当前位置重新生长 |
| [src/rrt_exploration/filter.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/filter.cpp) | 前沿点聚类、过滤 |
| [src/rrt_exploration/assigner.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/assigner.cpp) | 收益计算、选目标点下发 |
| [src/rrt_exploration/robot.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/robot.cpp) | 机器人位姿读取、目标发布 |
| [src/rrt_exploration/utils.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/utils.cpp) | 采样、Steer、Nearest、ObstacleFree、信息增益等基础算法 |
| [src/find_robot_action_server.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/find_robot_action_server.cpp) | 把目标点转成 Nav2 导航动作 |
| [src/pick_robot_action_server.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/pick_robot_action_server.cpp) | 抓取动作服务（与探索无关） |
| [src/robot_picker.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/robot_picker.cpp) | 行为树执行器（抓取流程） |
| [src/robot_pose_publisher.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/robot_pose_publisher.cpp) | 把视觉融合点云转成目标位姿（抓取用） |
| [src/rrt_exploration/wait_for_fin.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/wait_for_fin.cpp) | 探索结束判定（前沿耗尽+静止）→ 存图 → 停 assigner → 回原点（已纳入 launch，可用 `enable_wait_for_fin:=false` 关闭） |
| [scripts/boundary_publisher.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/scripts/boundary_publisher.py) | 自动发布 4+1 个边界点 |
| [config/exploration_params.yaml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/config/exploration_params.yaml) | RRT 四节点参数 |
| [config/nav2_params.yaml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/config/nav2_params.yaml) | 本包专用 Nav2 参数 |
| [behaviour_trees/navigate_w_replanning_time.xml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/behaviour_trees/navigate_w_replanning_time.xml) | 探索用的导航行为树（1Hz 重规划） |
| [behaviour_trees/robot_picker_behaviour_tree.xml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/behaviour_trees/robot_picker_behaviour_tree.xml) | 抓取行为树 |

---

## 3. 启动流程

### 3.1 前置条件

- 小车硬件急停开关处于**释放（可运动）**状态。
- 雷达、底盘串口正常。
- **不要**同时运行 `ros2 launch wheeltec_mycobot_bringup all.launch.py`：下面的 step1 已经包含了底盘、雷达、EKF、URDF，重复启动会出现两套同名节点抢串口。

### 3.2 两条命令

```bash
# step1：底盘 + 雷达 + EKF + URDF + slam_toolbox（建图）
ros2 launch wheeltec_slam_toolbox online_sync.launch.py
  或者使用cartographer建图:
  ros2 launch  wheeltec_cartographer cartographer.launch.py

# step2：Nav2 + RRT 探索四节点 + action servers + 行为树节点
ros2 launch wheeltec_robot_rrt wheeltec_rrt_slam.launch.py

# step3:发布边界点
python3 src/wheeltec/wheeltec_robot_rrt2/scripts/boundary_publisher.py
```

### 3.3 step2 内部启动了什么

[wheeltec_rrt_slam.launch.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/launch/rrt_exploration/wheeltec_rrt_slam.launch.py) 依次 include / 启动：

| 动作 | 位置 | 内容 |
|---|---|---|
| Nav2 bringup | L20-L28 | `slam:=True`（跳过 amcl），参数用本包 `config/nav2_params.yaml`，行为树用 `behaviour_trees/navigate_w_replanning_time.xml` |
| RRT 四节点 | L30-L34 | `local_rrt`、`global_rrt`、`filter`、`assigner` |
| action servers | L42-L46 | `find_robot_action_server`、`pick_robot_action_server` |
| 行为树执行器 | L48-L53 | `robot_picker`，加载 `robot_picker_behaviour_tree.xml` |
| 位姿发布器 | L55-L59 | `robot_pose_publisher`（视觉抓取链路，可忽略） |
| slam_toolbox | L36-L40 | **已注释**，由 step1 负责，重复启动会导致生命周期状态转换失败 |

Nav2 实际启动的进程（[nav2_navigation_launch.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/launch/navigation/nav2_navigation_launch.py)）：`controller_server`、`planner_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`，以及**一个** `lifecycle_manager` 负责按序激活。

---

## 4. 话题 / 动作总览

| 话题 | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox | global_rrt、local_rrt、filter、assigner |
| `/scan` | `sensor_msgs/LaserScan` | 雷达驱动 | slam_toolbox、Nav2 局部代价地图 |
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz / boundary_publisher | global_rrt、local_rrt |
| `/detected_frontiers` | `geometry_msgs/PointStamped` | global_rrt、local_rrt | filter |
| `/filtered_goal_points` | `wheeltec_rrt_msg/PointArray` | filter | assigner |
| `/assigned_frontier_goal` | `geometry_msgs/PoseStamped` | assigner（经 robot.cpp） | find_robot_action_server |
| `/_shapes` | `visualization_msgs/Marker` | global_rrt、local_rrt | RViz（RRT 树 / 边界） |
| `/unfiltered_centroids` | `visualization_msgs/Marker` | filter | RViz |
| `/filtered_centroids` | `visualization_msgs/Marker` | filter | RViz |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | planner_server | filter |
| `/plan` | `nav_msgs/Path` | planner_server | controller_server |
| `/cmd_vel` | `geometry_msgs/Twist` | controller_server | 底盘 |

动作：`navigate_to_pose`（Nav2）、`find_coloured_box`（find_robot_action_server）、`pick_coloured_box`（pick_robot_action_server）。

---

## 5. 实现流程（逐段）

### 5.1 第 0 步：等待地图与边界

`global_rrt` / `local_rrt` 启动后：

1. 订阅 `/map`，`mapData.data` 为空时每帧打印 `Waiting for the map`（[global_rrt.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/global_rrt.cpp)）。
2. 订阅 `/clicked_point`，收满 **5 个点**后锁定边界，之后收到的点全部忽略：
   - 第 1、3 点决定矩形范围 `init_map_x`、`init_map_y` 与中心 `Xstartx/Xstarty`；
   - 第 5 点作为 RRT 的起始树根 `V`。
3. `local_rrt` 额外要求 TF `map -> base_footprint` 可用，否则一直等待。

### 5.2 第 1 步：RRT 采样与前沿判定（global_rrt / local_rrt）

定时器每周期执行一次：

```
x_rand  = 边界框内均匀随机点
x_nearest = V 中离 x_rand 最近的节点
x_new   = Steer(x_nearest, x_rand, eta)        # eta = 0.3 m
checking = ObstacleFree(x_nearest, x_new)      # 1 自由 / 0 障碍 / -1 未知
```

- `checking == -1`（射线首次进入未知区）→ 该点就是**前沿点**，发布到 `/detected_frontiers`，同时画到 `/_shapes`；然后 `V.clear()`，把机器人当前位姿作为新的树根（`local_rrt`）或保留既有树（`global_rrt`）。
- `checking == 1`（自由）→ 把 `x_new` 加入树 `V`，继续生长。
- `checking == 0`（障碍）→ 丢弃。

两者区别：`local_rrt` 每次发现前沿都重置树根，只探索车周围；`global_rrt` 的树在边界框内持续累积，负责远距离前沿。

### 5.3 第 2 步：聚类与过滤（filter）

[filter.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/filter.cpp) 以 0.1 s 定时器消费积累的前沿点：

1. **Mean-shift 聚类**，带宽 0.3 m，把密集前沿点合并成少量质心。
2. 逐个质心过滤（[filter.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/filter.cpp#L133-L139)）：
   - `grid_value(global_costmap) > 70`：位于代价地图高代价/障碍区 → 丢弃；
   - 周围信息增益 `< 0.05`（`info_radius*0.5 = 0.5 m` 内几乎无未知单元）→ 丢弃。
3. 剩下的质心组成 `PointArray` 发布到 `/filtered_goal_points`。

### 5.4 第 3 步：选点（assigner）

[assigner.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/assigner.cpp#L65-L100) 每 **3 秒**执行一次：

```
对每个候选前沿点 f：
    cost      = |robot_pos - f|
    info_gain = information_gain(map, f, 1.0m) - discount(map, 上次目标, f, 1.0m)
    若 |robot_pos - f| <= 3.0m          : info_gain *= 2.0   # 近处优先
    若 |f - 上次目标|   <  3.0m          : info_gain = 增益 * 2.0  # 滞回，避免来回跳
    revenue   = info_gain * 3.0 - cost
选择 revenue 最大的点 → 发布 /assigned_frontier_goal
```

额外约束：**`|robot_pos - f| < min_goal_distance(0.5m)` 的候选直接跳过**。原因是雷达盲区/车体遮挡会在车脚下产生伪前沿，一旦被选中，Nav2 会认为"已在目标点"而不发速度，探索彻底卡死。

### 5.5 第 4 步：转成导航动作（find_robot_action_server）

[find_robot_action_server.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/find_robot_action_server.cpp) 订阅 `/assigned_frontier_goal`，收到后把它作为 `navigate_to_pose` 的目标发送给 Nav2（优先于视觉抓取点 `/box_pickup_pose`）。

### 5.6 第 5 步：Nav2 执行

- 全局规划：`planner_server`，`SmacPlannerHybrid`（[nav2_params.yaml#L233-L237](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/config/nav2_params.yaml#L233-L237)），在 `/map` 上搜索。
- 局部控制：`controller_server`，DWB `FollowPath`，`desired_linear_vel: 0.3`（[nav2_params.yaml#L102-L104](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/config/nav2_params.yaml#L102-L104)）。
- 行为树：`RateController(1Hz) → ComputePathToPose → FollowPath`（[navigate_w_replanning_time.xml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/behaviour_trees/navigate_w_replanning_time.xml)），即边走边以 1 Hz 重规划。
- 输出 `/cmd_vel` 到底盘。

车移动后地图更新 → RRT 重新产生前沿 → 循环，直到没有合格前沿点，探索自然停止。

### 5.7 可选：结束判定与回原点（wait_for_fin）

[wait_for_fin.cpp](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/src/rrt_exploration/wait_for_fin.cpp) 设计为探索收尾：统计里程计位移是否长时间不变，判定建图完成后发布 `overmap_flag=1`、`rrt_flag=1`，并向 `/goal_pose` 发回起点。

**注意**：该节点已编译安装，但**没有被任何 launch 启动**，且包内没有节点订阅这两个标志位。需要时手动运行：

```bash
ros2 run wheeltec_robot_rrt wait_for_fin
```

---

## 6. 参数说明

### 6.1 [exploration_params.yaml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/config/exploration_params.yaml)

| 节点 | 参数 | 当前值 | 含义 |
|---|---|---|---|
| filter | `costmap_clearing_threshold` | 70 | 代价地图值高于此值的前沿点被丢弃 |
| filter | `info_radius` | 1.0 | 信息增益统计半径（过滤时用 0.5 倍） |
| assigner | `info_radius` | 1.0 | 信息增益统计半径 |
| assigner | `info_multiplier` | 3.0 | 信息增益权重，越大越偏向"看得多"的点 |
| assigner | `hysteresis_radius` | 3.0 | 滞回半径，避免目标反复跳变 |
| assigner | `hysteresis_gain` | 2.0 | 滞回增益倍数 |
| assigner | `min_goal_distance` | 0.5 | **忽略离车过近的伪前沿（关键）** |

### 6.2 [nav2_params.yaml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/config/nav2_params.yaml)

| 参数 | 当前值 | 说明 |
|---|---|---|
| `FollowPath.desired_linear_vel` | 0.3 | 探索巡航速度 |
| `local_costmap.width/height` | 3 m × 3 m | 局部代价地图范围 |
| `robot_radius` | 0.20 m | 底盘半径 |
| `inflation_radius` | 0.55 m | 膨胀半径 |
| `progress_checker.required_movement_radius` | 0.15 m | 进度检查位移阈值 |
| `progress_checker.movement_time_allowance` | 20.0 s | 进度检查允许时间 |
| `gridbased.plugin` | `nav2_smac_planner::SmacPlannerHybrid` | 全局规划器 |

> 探索时车速低且常需原地调整，进度检查器若仍为 `0.5m/10s` 会频繁误判 "Failed to make progress" 并中止目标，因此本包特意放宽为 `0.15m/20s`。

### 6.3 算法常量

| 常量 | 值 | 位置 |
|---|---|---|
| `eta`（RRT 步长） | 0.3 m | global_rrt.cpp / local_rrt.cpp |
| Mean-shift 带宽 | 0.3 m | filter.cpp |
| assigner 周期 | 3 s | assigner.cpp |
| 边界点数 | 5（4 边界 + 1 起点） | global_rrt.cpp / local_rrt.cpp |

---

## 7. 使用方法

### 7.1 标准流程

```bash
# 终端 1：底盘 + 雷达 + slam_toolbox
ros2 launch wheeltec_slam_toolbox online_sync.launch.py

# 终端 2：Nav2 + RRT 探索
ros2 launch wheeltec_robot_rrt wheeltec_rrt_slam.launch.py

# 终端 3：RViz（可选，也可在虚拟机里开）
rviz2
```

RViz 建议添加：

- `Map`：`/map`
- `LaserScan`：`/scan`
- `Marker`：`/_shapes`（RRT 树 + 边界线）、`/filtered_centroids`、`/unfiltered_centroids`
- `Pose`：`/assigned_frontier_goal`（当前探索目标）

### 7.2 发布 4+1 个边界点（必须）

用 RViz 工具栏的 **Publish Point**，按**顺时针或逆时针**依次点 4 个点围出探索范围，**第 5 个点在已知地图内**（通常直接点机器人当前位置）：

| 顺序 | 含义 |
|---|---|
| 1 | 矩形角点 A（决定 `init_map_x`、中心 X） |
| 2 | 角点 B |
| 3 | 对角点 C（决定 `init_map_y`、中心 Y） |
| 4 | 角点 D |
| 5 | 起点 / 树根（放在已知自由区） |

点完第 5 个点后 RRT 才开始工作，开始产生 `/detected_frontiers`，随后自动选点、发车。

命令行发布示例（`map` 坐标系，单位 m）：

```bash
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: -2.0, y: -2.0, z: 0.0}}"
```

5 个点按顺序各发一次即可。

### 7.3 自动发布边界点（可选）

[boundary_publisher.py](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/scripts/boundary_publisher.py) 可一次发完 5 个点，但：

- 默认坐标是 `(±10, ±10)` 与原点，**远超室内范围**，需先改文件里的 `boundary_points`；
- 该脚本**未安装为可执行**，直接跑源文件：

```bash
python3 src/wheeltec/wheeltec_robot_rrt2/scripts/boundary_publisher.py
```

### 7.4 观察是否正常工作

```bash
# 1. 前沿点是否产生（应持续有输出）
ros2 topic hz /detected_frontiers

# 2. 过滤后的候选点
ros2 topic echo /filtered_goal_points --once

# 3. 当前探索目标（应每 3s 左右更新）
ros2 topic echo /assigned_frontier_goal

# 4. Nav2 是否收到目标
ros2 topic echo /plan --once

# 5. 底盘是否在动
ros2 topic echo /cmd_vel
```

正常表现：`/detected_frontiers` 高频输出 → `/filtered_goal_points` 有点 → `/assigned_frontier_goal` 每 3 秒更新且**不是机器人当前位置** → `/cmd_vel` 有非零线速度 → 小车前进、地图扩大。

### 7.5 建图完成与回原点（自动）

step2 默认带 `wait_for_fin` 节点，满足以下**双条件**自动判定建图完成：

1. **前沿耗尽**：超过 `frontier_timeout_sec`（默认 15s）没有新前沿（需先收到过至少一批前沿，防止启动初期误判）；
2. **小车静止**：线/角速度低于 `still_*_eps` 并持续 `still_duration_sec`（默认 5s）。

触发后依次执行：保存地图（`map_saver_cli` 存 `/map`，默认 `~/maps/rrt_map.pgm/.yaml`）→ `pkill` 停掉 assigner → 向 `/goal_pose` 重发 5 次 map 原点目标 → 小车自动回初始位姿 → 发布 `overmap_flag=1`。

参数见 [exploration_params.yaml](file:///home/admi/wheeltec_mycobot_ws/src/wheeltec/wheeltec_robot_rrt2/config/exploration_params.yaml) 的 `wait_for_fin` 段；不需要自动回原点可关：

```bash
ros2 launch wheeltec_robot_rrt wheeltec_rrt_slam.launch.py enable_wait_for_fin:=false
```

手动保存地图仍可用：

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f ~/maps/my_map --ros-args -p map_subscribe_transient_local:=true
```

### 7.6 结束探索

- 直接 `Ctrl+C` 结束 step2（Nav2 + RRT），step1 保留可继续手动补建图；
- 或先 `Ctrl+C` 结束 step1，再结束 step2。

---

## 8. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| RRT 一直打印 `Waiting for the map` | slam_toolbox 未出图（无 `/scan`、TF 未通） | 确认 step1 正常、雷达到位 |
| 不产生任何前沿点 | 未发布 5 个边界点 | 按 7.2 发点 |
| 有前沿点但 `/assigned_frontier_goal` 一直在车脚下 | 伪前沿 | 已由 `min_goal_distance: 0.5` 过滤；若仍出现可适当调大 |
| 目标正常但车不动 | Nav2 判定已到点 / 进度检查中止 / 底盘急停 | 查 `controller_server` 日志中的 `Failed to make progress`；确认硬件急停已释放 |
| 小车走 10 秒后目标被中止 | 进度检查过严 | `required_movement_radius` 调小、`movement_time_allowance` 调大 |
| 一个目标重复发送、原地打转 | 前沿点未更新 / 目标被过滤 | 检查 `/filtered_goal_points` 是否持续有输出 |
| 启动即报 lifecycle / bond 错误 | 重复启动了两个同名 `slam_toolbox` | 只跑 step1，`wheeltec_rrt_slam.launch.py` 内 slam 保持注释 |
| `/assigner` 出现两次、`already registered` 警告 | 节点内嵌 `Robot` 另建同名 rosout 发布 | 仅日志噪声，不影响功能 |

---

## 9. 本包在 Jazzy 下的适配与修复记录

原包基于 ROS 2 Galactic，迁移到 Jazzy 后做了以下修改：

**启动与 Nav2 接口**

1. `wheeltec_rrt_slam.launch.py`：`from ament_index_python import get_package_share_directory` → `from ament_index_python.packages import ...`（Jazzy 下前者直接 ImportError）。
2. `wheeltec_rrt_slam.launch.py`：`wheeltec_slam`（slam_toolbox）保持**注释**，避免与 step1 重复启动同名节点。
3. `nav2_navigation_launch.py`：`nav2_recoveries` → `nav2_behaviors`（Jazzy 已删除前者）。
4. `nav2_navigation_launch.py`：BT 参数 `default_bt_xml_filename` → `default_nav_to_pose_bt_xml`。
5. `nav2_navigation_launch.py`：5 个独立 lifecycle manager 合并为 1 个按序激活，否则 `bt_navigator` 激活时自定义行为树找不到 `compute_path_to_pose` 等 action 而崩溃。
6. `nav2_params.yaml`：`progress_checker_plugin` / `goal_checker_plugin` → `*_plugins`（复数）。
7. `nav2_params.yaml`：`nav2_smac_planner/SmacPlannerHybrid` → `nav2_smac_planner::SmacPlannerHybrid`。
8. `nav2_params.yaml`：`recoveries_server` 段 → `behavior_server` 段，插件改为 `nav2_behaviors::Spin/BackUp/Wait`，`recovery_plugins` → `behavior_plugins`，补 `local_frame: odom_combined`。
9. `nav2_params.yaml`：删除 `plugin_lib_names` 列表（Jazzy 重复注册会导致 bt_navigator 崩溃）。
10. `robot_picker.cpp`：行为树黑板上补 `wait_for_service_timeout`（Jazzy 的 `BtActionNode` 强制读取），设为 60 s 以覆盖 SmacPlannerHybrid 初始化。

**算法缺陷**

11. `global_rrt.cpp` / `local_rrt.cpp`：`ObstacleFree` 用 **-1** 表示未知区，原代码判断 `checking == 255`。`char` 是有符号类型，`-1 != 255` 恒成立 → **前沿分支永远进不去**，一条 frontier 都发不出来，探索完全不动。已改为 `checking == -1`。
12. `assigner.cpp`：新增 `min_goal_distance`（默认 0.5 m），跳过车脚下的伪前沿。否则 Nav2 认为"已在目标点"，`/cmd_vel` 恒为零。
13. `nav2_params.yaml`：进度检查器由 `0.5m/10s` 放宽为 `0.15m/20s`，避免低速探索被误判"未取得进展"而中止。

---

## 10. 已知限制

- `wait_for_fin` 未纳入 launch，探索完成后的自动回原点与标志位发布默认不生效。
- `boundary_publisher.py` 未安装为可执行文件，且默认坐标不适用于室内。
- 雷达盲区会在车体周围产生少量伪前沿，靠 `min_goal_distance` 抑制，未做地图层面清理。
- 本包自带一套 Nav2 参数与行为树，**不要**与 `wheeltec_nav2` 的导航 launch 同时运行。
