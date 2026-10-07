# 从相机点云到 MoveIt 场景并在 RViz 显示的完整流程

本文档描述 `mycobot_mtc_pick_place_demo` 包中，相机点云如何被处理、转换为 MoveIt 碰撞对象、应用到 planning scene，并最终在 RViz 中显示的完整流程。

## 整体架构

整个流程由两个节点协作完成，通过一个 ROS 2 service 通信：

```
┌──────────────────────┐     service call     ┌──────────────────────┐
│  mtc_node            │ ───────────────────> │ get_planning_scene_  │
│  (MoveIt 客户端)     │ <──── response ───── │  server (感知端)     │
│                      │                      │                      │
│  - 调用 service       │                      │  - 订阅点云/图像      │
│  - 应用碰撞对象       │                      │  - PCL 处理           │
│  - 规划+执行         │                      │  - 形状拟合           │
└──────────────────────┘                      └──────────────────────┘
        ↓                                               ↓
  PlanningSceneInterface                          /camera_head/...
  → /monitored_planning_scene                    Gazebo RGBD 插件
        ↓
  RViz PlanningScene 显示
```

### 关键组件

| 组件 | 文件/位置 | 作用 |
|------|-----------|------|
| Service 接口定义 | [mycobot_interfaces/srv/GetPlanningScene.srv](mycobot_interfaces/srv/GetPlanningScene.srv) | 定义请求/响应消息结构 |
| 感知服务端 | [get_planning_scene_server.cpp](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp) | 点云处理与形状拟合 |
| MoveIt 客户端 | [mtc_node.cpp](mycobot_mtc_pick_place_demo/src/mtc_node.cpp) | 调用 service 并应用到场景 |
| 感知参数配置 | [get_planning_scene_server.yaml](mycobot_mtc_pick_place_demo/config/get_planning_scene_server.yaml) | 裁剪、聚类、拟合参数 |
| Server 启动 | [get_planning_scene_server.launch.py](mycobot_mtc_pick_place_demo/launch/get_planning_scene_server.launch.py) | 加载节点与参数 |

### Service 接口

```text
# Request
string target_shape          # 目标形状: "box" 或 "cylinder"
float64[] target_dimensions  # 目标近似尺寸，用于识别

---

# Response
moveit_msgs/PlanningSceneWorld scene_world  # 所有检测到的碰撞对象
sensor_msgs/PointCloud2 full_cloud          # 完整场景点云
sensor_msgs/Image rgb_image                 # 场景 RGB 图像
string target_object_id                     # 目标对象在 PlanningSceneWorld 中的 ID
string support_surface_id                   # 支撑面在 PlanningSceneWorld 中的 ID
bool success                                # 操作是否成功
```

## 详细流程（13 步）

### 阶段 A：感知端数据采集

#### 步骤 1：订阅相机话题

`get_planning_scene_server` 节点订阅两个话题（配置在 [get_planning_scene_server.yaml](mycobot_mtc_pick_place_demo/config/get_planning_scene_server.yaml#L3-L5)）：

```yaml
point_cloud_topic: "/camera_head/depth/color/points"   # Gazebo RGBD 相机点云
rgb_image_topic: "/camera_head/color/image_raw"        # RGB 图像
target_frame: "base_link"                              # 目标坐标系
```

**数据来源链路**：Gazebo 中的相机插件 → `ros_gz_bridge` 桥接 → ROS 2 话题 → server 回调函数缓存最新一帧到 `latest_point_cloud` 和 `latest_rgb_image`。

#### 步骤 2：坐标变换到 base_link

收到 service 请求后，[transformPointCloud()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L423-L476) 用 tf2 把点云从相机坐标系 `camera_head_depth_optical_frame` 转换到 `base_link`（机器人基座），便于在机器人坐标系中描述物体位置。

#### 步骤 3：裁剪感兴趣区域

通过 PCL `CropBox` 过滤（[yaml 配置](mycobot_mtc_pick_place_demo/config/get_planning_scene_server.yaml#L8-L15)）：

```yaml
enable_cropping: true
crop_min_x: 0.10   # 只保留机器人前方 0.1m ~ 1.1m
crop_max_x: 1.1
crop_min_y: 0.00   # 左 0 ~ 右 0.9m
crop_max_y: 0.90
```

去除机器人自身、天花板等无关点云，只保留桌面工作区。

#### 步骤 4：点云格式转换

[convertToPCL()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L479-L516) 把 ROS `PointCloud2` 转为 PCL `PointCloudXYZRGB`，并保存调试 PCD 到 `/tmp/4_convertToPCL_debug_cloud.pcd`。

### 阶段 B：平面与物体分割

#### 步骤 5：分离桌面和物体

[segmentPlaneAndObjects()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L747-L770) 执行：

- **RANSAC 平面分割**：找出最大平面（桌面/支撑面），输出平面点云 `support_plane_cloud`、平面方程 `plane_coefficients`、以及去除平面后的物体点云 `objects_cloud`
- 保存调试文件：`/tmp/5_support_plane_debug_cloud.pcd`、`/tmp/5_objects_cloud_debug_cloud.pcd`

#### 步骤 6：创建支撑面碰撞对象

[createSupportSurfaceObject()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L519-L593) 将桌面转为 Box 碰撞对象：

- 计算平面点云的 AABB 包围盒（长宽）
- 用平面法向量计算 Box 朝向（让 Box 平面与桌面平行）
- 强制最小厚度 `min_surface_thickness`（避免 Z 维度为 0）
- id 设为 `support_surface`，加到 `response->scene_world.collision_objects`

#### 步骤 7：估计法线/曲率/RSD

[estimateNormalsCurvatureAndRSD()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L795-L810) 为每个点计算：

- **法向量**：判断表面朝向
- **曲率**：表面弯曲程度
- **RSD（Radius Surface Descriptor）**：局部表面半径，用于区分圆柱/平面

#### 步骤 8：区域生长聚类

[extractClusters()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L826-L844) 用区域生长算法（基于法线/曲率相似性）把物体点云分成多个独立 cluster，每个 cluster 对应一个物体。

#### 步骤 9：形状拟合生成碰撞对象

[segmentObjects()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L858-L875) + [fitShapeToCluster()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L596-L739) 对每个 cluster：

- 尝试用 RANSAC 拟合 **Box** 和 **Cylinder** 两种基本体
- 选择 inlier 比例高的作为最佳拟合
- 创建对应 `shape_msgs::SolidPrimitive`（BOX 或 CYLINDER），并计算位姿
- 生成 `moveit_msgs::CollisionObject`（id = `cylinder_0`、`box_1` 等）

#### 步骤 10：识别目标物体

[identifyTargetObject()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L741-L791) 根据 `mtc_node` 传入的 `target_shape` 和 `target_dimensions`：

- 形状匹配权重 0.7
- 尺寸相似度权重 0.3
- 选出综合分数最高的对象作为 `target_object_id`

#### 步骤 11：组装响应

[assemblePlanningSceneWorld()](mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp#L793-L821) 把所有 `CollisionObject` 装入 `PlanningSceneWorld`，连同 `full_cloud`、`rgb_image`、`target_object_id`、`support_surface_id` 一起作为 service response 返回。

### 阶段 C：客户端应用场景并显示

#### 步骤 12：mtc_node 调用 service 并应用

[mtc_node.cpp](mycobot_mtc_pick_place_demo/src/mtc_node.cpp#L288-L313) 收到 response 后：

```cpp
scene_world_ = response.scene_world;        // 取得 PlanningSceneWorld
target_object_id_ = response.target_object_id;
support_surface_id_ = response.support_surface_id;

// 关键：应用到 MoveIt planning scene
if (!psi.applyCollisionObjects(scene_world_.collision_objects)) { ... }
```

`psi` 是 `PlanningSceneInterface`，它通过 `/apply_collision_objects` service 把所有碰撞对象（支撑面 + 物体）注入 MoveIt 的 planning scene。

#### 步骤 13：MoveIt 自动发布到 RViz

在 [move_group.launch.py](mycobot_moveit_config/launch/move_group.launch.py#L110-L117) 中配置了 PlanningScene 监控：

```python
.planning_scene_monitor(
    publish_planning_scene=True,         # ← 关键：发布 /monitored_planning_scene
    publish_robot_description_semantic=True,
    publish_robot_description=False,
)
```

**数据流**：

```
PlanningSceneInterface.applyCollisionObjects()
   ↓ 通过 /apply_collision_objects service
move_group 的 PlanningSceneMonitor
   ↓ 更新内部 PlanningScene
   ↓ 发布到 /monitored_planning_scene topic
RViz 的 PlanningScene 显示插件
   ↓ 订阅 /monitored_planning_scene
显示彩色 Box/Cylinder 物体到 3D 场景
```

RViz 配置文件中若启用了 `PlanningScene` 显示项（mtc_demos.rviz 中默认开启），就能实时看到：

- 桌面 Box（支撑面）
- 物体 Cylinder/Box（目标物体）
- 随机械臂运动时的碰撞检测实时反馈

## 关键点总结

| 阶段 | 节点 | 关键 API/话题 |
|------|------|---------------|
| 数据采集 | get_planning_scene_server | 订阅 `/camera_head/depth/color/points` |
| 点云处理 | 同上 | PCL CropBox + RANSAC + 区域生长 |
| 形状拟合 | 同上 | `shape_msgs::SolidPrimitive` (BOX/CYLINDER) |
| 通信桥梁 | service `get_planning_scene_mycobot` | `mycobot_interfaces/srv/GetPlanningScene` |
| 应用场景 | mtc_node | `PlanningSceneInterface::applyCollisionObjects()` |
| RViz 显示 | move_group | `PlanningSceneMonitor` → `/monitored_planning_scene` |

## 简化数据流（一张图）

```
Gazebo 相机插件
  → ros_gz_bridge → /camera_head/depth/color/points
  → get_planning_scene_server (PCL 处理 + 形状拟合)
  → service response (PlanningSceneWorld)
  → mtc_node 调用 PlanningSceneInterface.applyCollisionObjects()
  → move_group PlanningSceneMonitor 更新
  → /monitored_planning_scene topic
  → RViz PlanningScene 显示插件
  → 你在 RViz 中看到的彩色 Box/Cylinder 物体
```

## 调试技巧

### 查看中间点云

在运行 `get_planning_scene_server` 后，`/tmp/` 目录下会生成调试 PCD 文件：

```bash
# 查看所有调试点云
ls /tmp/*debug_cloud*.pcd

# 用 point_cloud_viewer 可视化
ros2 launch mycobot_mtc_pick_place_demo point_cloud_viewer.launch.py \
    file_name:=/tmp/5_objects_cloud_debug_cloud.pcd
```

### 关键调试文件

| 文件 | 含义 |
|------|------|
| `/tmp/4_convertToPCL_debug_cloud.pcd` | 坐标变换 + 裁剪后的点云 |
| `/tmp/5_support_plane_debug_cloud.pcd` | 分割出的支撑面点云 |
| `/tmp/5_objects_cloud_debug_cloud.pcd` | 去除支撑面后的物体点云 |

### 常见问题

- **RViz 中看不到物体**：确认 PlanningScene 显示项已启用，且 `/monitored_planning_scene` 话题有数据
- **物体识别错误**：调整 [get_planning_scene_server.yaml](mycobot_mtc_pick_place_demo/config/get_planning_scene_server.yaml) 中的聚类/拟合参数
- **点云为空**：检查 Gazebo 相机是否工作，`ros2 topic echo /camera_head/depth/color/points` 验证
- **TF 变换失败**：确保 `base_link` 到 `camera_head_depth_optical_frame` 的 TF 树完整
