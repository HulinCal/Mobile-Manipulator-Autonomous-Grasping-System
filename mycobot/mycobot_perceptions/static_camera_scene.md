# static_camera_clound 节点使用：
## 1) astra_camera（发布点云 + 相机 TF: camera_link → camera_color_optical_frame 等）
ros2 launch astra_camera astra_pro.launch.py

## 2) move_group + 你的节点（PSM + worker）
ros2 launch mycobot_perceptions static_scene.launch.py

## 3) 如果 base_link → camera_link 缺失，单独发个 static_transform_publisher
ros2 run tf2_ros static_transform_publisher \
  --x -0.29 --y 0.22 --z 0.13 --qx 0.5 --qy -0.5 --qz 0.5 --qw 0.5 \
  --frame-id base_link --child-frame-id camera_link

  ros2 run tf2_ros static_transform_publisher \
  --x -0.29 --y 0.22 --z 0.13 --qx 0.7071 --qy -0 --qz 0 --qw 0.7071 \
  --frame-id base_link --child-frame-id camera_link

  ros2 run tf2_ros static_transform_publisher \
  --x -0.29 --y 0.35 --z 0.15 --qx 0.7071 --qy -0 --qz 0 --qw 0.7071 \
  --frame-id base_link --child-frame-id camera_link

  ros2 run tf2_ros static_transform_publisher \
  --x -0.6 --y 0.35 --z 0.15 --qx 0.7071 --qy -0 --qz 0 --qw 0.7071 \
  --frame-id base_link --child-frame-id camera_link

  ros2 run tf2_ros static_transform_publisher \
  --x -0.6 --y 0.25 --z 0.15 --qx 0.7071 --qy -0 --qz 0 --qw 0.7071 \
  --frame-id base_link --child-frame-id camera_link


# static_camera_clound 节点说明

> 文件：[`src/static_camera_clound.cpp`](src/static_camera_clound.cpp)
> Launch：[`launch/static_scene.launch.py`](launch/static_scene.launch.py)
> 包：`mycobot_perceptions`

## 1. 节点功能概览

`static_camera_clound` 节点把一个静态相机的点云实时转换为 MoveIt PlanningScene 中的碰撞物体，参与运动规划的碰撞检测。核心流水线：

```
/camera/depth_registered/points  ──►  [订阅线程]
                                            │  O(1) 缓存 + 唤醒
                                            ▼
                                      [worker 线程]
                                            │
                ┌───────────────────────────┴───────────────────────────┐
                │ 1) ROS msg → PCL XYZRGB                                  │
                │ 2) TF 变换: camera_color_optical_frame → target_frame    │
                │ 3) VoxelGrid 体素降采样                                  │
                │ 4) NormalEstimation 法线估计                             │
                │ 5) RegionGrowing 区域增长聚类                            │
                │ 6) 每类 getMinMax3D → AABB(轴对齐包围盒)                  │
                │ 7) AABB → CollisionObject(BOX, ADD)                     │
                └───────────────────────────┬───────────────────────────┘
                                            │  publish
                                            ▼
                            /test_planning_scene  ──remap──►  /planning_scene
                                                              │
                                                              ▼
                                          move_group 的 PlanningSceneMonitor
                                                              │
                                                              ▼
                                              /monitored_planning_scene
                                                              │
                                                              ▼
                                                       RViz MotionPlanning
```

### 设计要点：避免回调堆积

| 阶段 | 线程 | 操作 | 耗时 |
|---|---|---|---|
| `pointCloudCallback` | ROS executor 线程 | 仅 O(1) 缓存最新帧 + `notify_one()` | μs 级 |
| `workerLoop` | 独立 worker 线程 | `wait` → 取最新帧 → PCL 处理 → publish | 数百 ms |

- **回调不处理 PCL**：保证订阅永不阻塞、永不堆积。
- **单 worker 串行处理**：处理慢时新帧覆盖旧帧（实时性优先于完整性），不会同时跑多个聚类实例。
- **事件驱动发布**：处理完一帧立即 publish，PSM 立即收到，不再依赖周期定时器。
- **启动即发初始静态 cube**：让 PSM 在点云到来前就有初始障碍物（latched QoS）。

### 关键解决的两个坑

1. **PSM 订阅 QoS 是 VOLATILE**：传统"周期定时器发布"模式下，节点启动时只发一次 latched 消息，move_group 启动后拿不到历史帧。本节点用**事件驱动**：每处理完一帧立即重发，PSM 一订阅就能拿到下一帧。
2. **点云 frame_id (`camera_color_optical_frame`) 不在 moveit TF 树里**：PSM 报 `Unknown frame`。本节点用 `tf2_ros::Buffer` 把点云从相机 frame 变换到 `target_frame`（默认 `base_link`）后再聚类，CollisionObject 的 `header.frame_id` 也用 `target_frame`，PSM 即可正常合并。

## 2. ROS 接口

### 2.1 订阅

| Topic | 类型 | QoS | 用途 |
|---|---|---|---|
| `/camera/depth_registered/points`（可参数覆盖） | `sensor_msgs/PointCloud2` | 默认 (10, volatile, reliable) | 输入点云 |

### 2.2 发布

| Topic | 类型 | QoS | 用途 |
|---|---|---|---|
| `/test_planning_scene`（launch remap 到 `/planning_scene`） | `moveit_msgs/PlanningScene` | `KeepLast(1) + transient_local + reliable` | 输出场景 |

### 2.3 TF

- 节点内部创建 `tf2_ros::Buffer` + `TransformListener`。
- 处理每帧点云前查询 `lookupTransform(target_frame, cloud_frame, TimePointZero)`。
- 查不到时打 WARN 跳过本帧（不污染场景）。

## 3. 参数完整列表

### 3.1 话题与基础

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `point_cloud_topic` | string | `/camera/depth_registered/points` | 订阅的点云话题 |
| `planning_scene_topic` | string | `/test_planning_scene` | 发布的 PlanningScene 话题（launch 会 remap 到 `/planning_scene`） |
| `target_frame` | string | `base_link` | 点云变换到此 frame 后再聚类；必须是 moveit TF 树里存在的 frame |

### 3.2 初始静态 cube（启动时发一次）

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `cube_frame_id` | string | `base_link` | 静态 cube 的 frame |
| `cube_id` | string | `static_cube` | CollisionObject id |
| `cube_size_x/y/z` | double | `0.10 / 0.10 / 0.10` | 立方体尺寸 (m) |
| `cube_pos_x/y/z` | double | `0.30 / 0.30 / 0.10` | 立方体中心位置 (m) |

### 3.3 PCL 处理

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `voxel_leaf_size` | double | `0.01` | 体素滤波叶大小 (m)，0 跳过滤波 |
| `normal_search_radius` | double | `0.03` | 法线估计搜索半径 (m) |
| `region_num_neighbors` | int | `30` | 区域增长邻居数 (knn) |
| `region_smooth_threshold` | double | `7.0` | 平滑阈值 (度) |
| `region_curvature_threshold` | double | `1.0` | 曲率阈值 |
| `min_cluster_size` | int | `100` | 类最小点数（去噪） |
| `max_cluster_size` | int | `100000` | 类最大点数（去地面/墙面） |
| `bbox_padding` | double | `0.01` | 包围盒每边外扩尺寸 (m)，避免规划紧贴 |
| `bbox_max_size` | double | `0.5` | 包围盒最大边长 (m)，超过跳过该类 |

## 4. `static_scene.launch.py` 说明

### 4.1 启动内容

1. **Include `mycobot_moveit_config/move_group.launch.py`** — 透传 `use_sim_time` / `robot_name` / `use_rviz`。move_group 内部的 PSM 会订阅 `/planning_scene`、合并后转发到 `/monitored_planning_scene` 供 RViz 显示。
2. **`static_camera_clound` 节点** — 自动 remap `/test_planning_scene` → `/planning_scene`，让 PSM 直接消费。

### 4.2 Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `use_sim_time` | `true` | 是否使用仿真时钟 |
| `robot_name` | `mycobot_280` | 传给 move_group 的机器人名 |
| `use_rviz` | `true` | 是否由 move_group.launch.py 启动 RViz |
| `cube_frame_id` | `base_link` | 静态 cube 的 frame |
| `cube_pos_x/y/z` | `0.30 / 0.30 / 0.10` | 静态 cube 中心位置 (m) |
| `cube_size_x/y/z` | `0.10 / 0.10 / 0.10` | 静态 cube 尺寸 (m) |
| `publish_rate` | `1.0` | 兼容参数（事件驱动模式下已无实际作用，保留供回退） |

### 4.3 启动方式

```bash
source /home/hl/mycobot/mycobot_ros2-main/install/setup.bash

# 默认
ros2 launch mycobot_perceptions static_scene.launch.py

# 真机/不用仿真时钟
ros2 launch mycobot_perceptions static_scene.launch.py use_sim_time:=false

# 自定义 cube 位置/尺寸
ros2 launch mycobot_perceptions static_scene.launch.py \
    cube_pos_x:=0.4 cube_pos_z:=0.15 cube_size_x:=0.2 cube_size_y:=0.2 cube_size_z:=0.2
```

## 5. 运行前置条件

### 5.1 TF 树必须完整

节点需要查到 `camera_color_optical_frame → base_link` 的变换。验证：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

如果报 "could not find a connection"，需启动：
- astra_camera 的 launch（发布 `camera_link → camera_color_optical_frame` 等）
- `base_link → camera_link` 的 static_transform_publisher 或把相机 URDF 挂到机器人 URDF

```bash
# 单独发一个 base_link → camera_link 的静态变换（位置按实际安装调整）
ros2 run tf2_ros static_transform_publisher \
  --x 0.15 --y 0 --z 0.30 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id base_link --child-frame-id camera_link
```

### 5.2 完整启动顺序

```bash
# 1) astra_camera（发布点云 + 相机 TF）
ros2 launch astra_camera astra_pro.launch.py

# 2) move_group + static_camera_clound + RViz
ros2 launch mycobot_perceptions static_scene.launch.py
```

## 6. 验证方法

```bash
# 1) 节点在跑
ros2 node list | grep static_camera_clound

# 2) PSM 是 /planning_scene 的订阅者
ros2 topic info /planning_scene -v
#    Publisher count: 1   (static_camera_clound)
#    Subscription count: 1  (move_group 的 PSM)

# 3) 抓一帧看 cluster
ros2 topic echo /monitored_planning_scene --once | grep -A3 cluster_

# 4) 查日志看聚类结果
ros2 topic echo /rosout --once | grep static_camera_clound
```

正常运行的典型日志：

```
[static_camera_clound] INFO: Published initial static cube 'static_cube' ...
[static_camera_clound] INFO: Region growing found 3 cluster(s).
[static_camera_clound] INFO: Cluster 0: 150 pts, AABB (0.05 x 0.05 x 0.05) m @ (0.30, 0.00, 0.15) frame=base_link
[static_camera_clound] INFO: Cluster 1: ...
[static_camera_clound] INFO: Published scene with 3 collision object(s) (#1).
```

## 7. 故障排查

| 现象 | 可能原因 | 解决 |
|---|---|---|
| PSM 报 `Unknown frame: camera_color_optical_frame` | 节点未做 TF 变换或 `target_frame` 不在 TF 树 | 确认 `target_frame` 参数值在 moveit TF 树里；确认 astra_camera TF 已发布 |
| RViz 看不到立方体 | `/test_planning_scene` 没 remap 到 `/planning_scene`；或 move_group 没启动 | 用 launch 启动（自动 remap）；`ros2 node list` 看 `/move_group` |
| RViz "No Planning Scene Loaded" | move_group 没启动，`/monitored_planning_scene` 不存在 | 启动 move_group；或 RViz 改 PlanningScene Display 的话题为 `/planning_scene` |
| `processCloud failed for this frame` | TF 查不到、点云空、聚类为空 | 看 WARN 日志定位具体阶段 |
| 聚类结果噪声多 | `min_cluster_size` 太小、`voxel_leaf_size` 太小 | 调大 `min_cluster_size`（如 200）或 `voxel_leaf_size`（如 0.02） |
| 地面/墙面被识别为大块 | `bbox_max_size` 太大、`max_cluster_size` 太大 | 调小 `bbox_max_size`（如 0.3）或 `max_cluster_size` |
| 规划贴着物体失败 | 包围盒太紧 | 调大 `bbox_padding`（如 0.02） |

## 8. 依赖

### 8.1 包依赖（package.xml）

- `rclcpp`、`sensor_msgs`、`moveit_msgs`、`shape_msgs`、`geometry_msgs`
- `pcl_conversions`、`pcl_ros`（PCL ROS 桥）
- `tf2_ros`、`tf2_eigen`、`tf2_geometry_msgs`
- `Eigen`（通过 `find_package(Eigen3)`）

### 8.2 构建目标（CMakeLists.txt）

可执行 `static_camera_clound` 安装到 `lib/mycobot_perceptions/static_camera_clound`。

## 9. 后续可扩展方向

- **点云预处理**：加 `PassThrough` 滤波限定工作空间范围；加 `StatisticalOutlierRemoval` 去除离群点。
- **聚类算法**：可选 Euclidean Cluster Extraction（更快）替代 Region Growing；或欧氏聚类 + 法线方向过滤。
- **包围盒精化**：用 PCA + OBB（有向包围盒）替代 AABB，更紧贴物体。
- **物体持久化**：跨帧匹配相同 id（基于位置/特征匹配），避免每帧 id 变化。
- **REMOVE 操作**：定期发 `CollisionObject.operation = REMOVE` 清理已不存在的物体。
- **服务接口**：加 `~enable_processing` 服务，外部可暂停/恢复点云处理。
