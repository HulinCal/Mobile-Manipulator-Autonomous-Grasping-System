# 3. 导航节点 (navigation_node)

## 功能

提供三个核心服务：
1. **`navigate_near`**：给定物体位姿（苹果），**视觉伺服**接近物体——行驶中
   持续保持苹果在相机视野中央，直到停在苹果前 `approach_distance` 处。
2. **`navigate_to`**：导航到指定目标位姿（Nav2 或内置 cmd_vel 驱动）。
3. **`go_home`**：返回起始位姿（需先调用 `record_start` 记录）。

导航路线按优先级自动选择：
| 场景 | 路线 |
|------|------|
| `sim_mode:=true` | 模拟等待 `sim_duration` 秒 |
| Nav2 可用（`map` 帧 TF 存在 + navigator 已启动） | `BasicNavigator` action |
| 其余情况（本仓库默认） | **内置 cmd_vel 视觉伺服 / 点位驱动** |

## 数据流：从检测到导航到苹果旁边

```
                 ┌────────────────────────┐
 语音/测试触发   │  coordinator_node       │
 "把苹果拿过来"  │  STATE: DETECTING       │
 ─────────────► │  调用 detect_apple 服务  │
                └───────┬────────────────┘
                        │ /pick_place/detect_apple (Trigger, 同步)
                        ▼
                ┌────────────────────────────────────────┐
                │  apple_detection_node                   │
                │  1. YOLO 检测彩色图 → 苹果框             │
                │  2. 深度图反投影 → base_link 3D 位姿     │
                │    · 响应携带位姿 (success + message)    │
                └───────┬────────────────────────────────┘
                        │ /pick_place/apple_pose (TRANSIENT_LOCAL, latched)
                        ▼
                ┌────────────────────────┐
                │  coordinator_node       │
                │  STATE: NAV_TO_APPLE    │
                │  发布 object 位姿        │
                └───────┬────────────────┘
                        │ /pick_place/nav_object (PoseStamped, base_link)
                        ▼
                ┌─────────────────────────────────────────────┐
                │  navigation_node  /pick_place/navigate_near │
                │  视觉伺服接近 (详见下节)                      │
                └───────┬─────────────────────────────────────┘
                        │ /pick_place/nav_result (Bool)
                        ▼
                coordinator STATE: GRASPING → grasp_node
                (抓取使用 latched 的最终苹果位姿)
```

要点：
- 苹果位姿**始终在 `base_link` 系**（相机与底盘经 URDF 固定连接），
  车动之后旧位姿自然失效，必须靠行驶中重新检测更新。
- `apple_pose` 用 **TRANSIENT_LOCAL (latched)** QoS：后连接的
  `grasp_node` / `navigation_node` 启动即可拿到最后一次检测位姿。

## 视觉伺服逻辑 (`_visual_servo_drive`)

控制循环 10 Hz，检测更新约 1 次 / 3 s（CPU 推理耗时），两者解耦：

```
 初始: 苹果位姿 (base_link) → 设第一个"锚点" anchor
       anchor = {车 odom 位置, 苹果方位角(odom系), 视觉距离}

 每拍 (0.1s):
   1. est_dist = anchor.dist - 车沿苹果方位的 odom 位移投影
      (里程计推算, 保证观测失效时仍能正确停车)
   2. 雷达安全检查 (前方 ±29° 最小距离 obs_d):
        obs_d < 0.10 m                        → 硬停, 失败
        obs_d < 0.20 m 且 est_dist ≤ 0.55 m   → 最后保险: 停车, 判定成功到达
        obs_d < 0.20 m 且 est_dist > 0.55 m   → 真障碍, 停车失败
        obs_d < 0.30 m                        → 减速一半
   3. 限程: 本次行驶总里程 > travel_limit → 停车失败
   4. 未锁定最终位姿时: 异步调用 detect_apple (1s 一次,
      上次未完成不发新请求), 完成后:
        · 更新 anchor (重置里程推算基准)
        · ndist ≤ 0.55 m → 锁定最终位姿 (停发检测,
          latched pose 供抓取使用; <0.5m 结构光不可靠)
   5. 到达判定: est_dist ≤ approach_distance + tol_xy (0.40 m) → 停车成功
   6. 方向 P 控制: w = P(anchor.bearing − 当前车头朝向),
      死区 0.12 rad (观测 3s 才更新, 小偏差持续转向会震荡);
      角速度上限 0.1 rad/s 与观测周期匹配 (0.1 × 3s ≈ 0.3 rad/周期);
      线速度 v = min(max_lin_vel, 0.8 × (est_dist − approach)),
      方位偏差 > 0.35 rad 时先转向 (v × 0.3)
```

### 为什么这样设计

| 问题 | 对策 |
|------|------|
| 苹果红色表面深度空洞，框内有效像素多为背景 | 检测端取"最近的显著深度聚类"（苹果/桌面是前景） |
| astra 结构光 <0.5m 不可靠 | 0.5–1.5m 精度带内锁定最终位姿，最后 0.2m 纯里程计 |
| 检测 ~3s/次，观测滞后导致转向转过头 | 角速度上限与观测周期匹配 + 0.12 rad 死区 |
| 观测停滞时旧数据导致撞上苹果 | 里程计推算 est_dist + **雷达最后保险停车** |
| 回调内调用服务/等订阅导致 future 永不完成 | detect client 与 apple_pose 订阅放独立 `ReentrantCallbackGroup` |

### 停车的四层保险（由先到后）

1. `est_dist ≤ 0.40 m`（里程推算正常到达）
2. `雷达 < 0.20 m 且 est_dist ≤ 0.55 m`（最后保险，判定成功）
3. 行驶总里程 > `travel_limit`（失败停车）
4. `雷达 < 0.10 m`（真障碍硬停，失败）

## 话题与服务

| 类型 | 名称 | 消息类型 | QoS | 说明 |
|------|------|----------|-----|------|
| 订阅 | `/pick_place/nav_goal` | `PoseStamped` | RELIABLE | 目标位姿（配合 navigate） |
| 订阅 | `/pick_place/nav_object` | `PoseStamped` | RELIABLE | 要靠近的物体位姿（配合 navigate_near） |
| 订阅 | `/pick_place/apple_pose` | `PoseStamped` | TRANSIENT_LOCAL | 检测节点 latched 的最新苹果位姿 |
| 订阅 | `/scan` | `LaserScan` | SENSOR_DATA | 雷达避障 / 最后保险停车 |
| 发布 | `/pick_place/approach_pose` | `PoseStamped` | RELIABLE | 计算出的靠近位姿 |
| 发布 | `/pick_place/nav_result` | `Bool` | RELIABLE | 导航结果 |
| 发布 | `/cmd_vel` | `Twist` | RELIABLE | 底盘速度指令 |
| 客户端 | `/pick_place/detect_apple` | `Trigger` | — | 行驶中异步请求重新检测 |
| 服务 | `/pick_place/navigate_near` | `Trigger` | — | 靠近物体（视觉伺服） |
| 服务 | `/pick_place/navigate` | `Trigger` | — | 导航到 nav_goal |
| 服务 | `/pick_place/record_start` | `Trigger` | — | 记录当前位姿为起点 |
| 服务 | `/pick_place/go_home` | `Trigger` | — | 返回起点 |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sim_mode` | `true` | 模拟模式 |
| `approach_distance` | `0.35` | 停在苹果前的距离 [m] |
| `goal_tolerance_xy` | `0.05` | 位置到达容差 [m] |
| `goal_tolerance_yaw` | `0.15` | 航向到达容差 [rad] |
| `sim_duration` | `2.0` | 模拟导航耗时 [s] |
| `map_frame` | `map` | 地图坐标系 |
| `base_frame` | `base_link` | 底盘坐标系 |
| `odom_frame` | `odom_combined` | 里程计坐标系（cmd_vel 驱动反馈） |
| `cmd_vel_topic` | `/cmd_vel` | 速度指令话题 |
| `max_lin_vel` | `0.05` | 最大线速度 [m/s] |
| `max_ang_vel` | `0.05` | 点位导航最大角速度 [rad/s] |
| `nav_timeout` | `30.0` | 点位导航超时 [s] |
| `max_travel_dist` | `0.5` | 点位导航最大行驶里程 [m] |
| `min_obstacle_dist` | `0.1` | 雷达硬停距离 [m] |
| `scan_topic` | `/scan` | 雷达话题 |
| `servo_max_ang_vel` | `0.1` | 伺服转向角速度上限 [rad/s]（与检测周期匹配） |
| `servo_timeout` | `90.0` | 伺服总超时 [s] |
| `servo_max_travel` | `1.0` | 伺服最大行驶里程下限 [m]（实际取 max(此值, 初始距离−0.35+0.25)） |
| `final_fix_dist` | `0.55` | 最终位姿锁定距离（结构光可信带）[m] |
| `lidar_stop_dist` | `0.2` | 雷达最后保险停车距离 [m] |
| `detect_period` | `1.0` | 检测请求间隔 [s]（单次检测 ~3s，未完成不重发） |
| `servo_stale_timeout` | `8.0` | 观测过期警告阈值 [s]（过期后仅警告，靠里程推算继续） |

## 测试方法

```bash
# 无语音测试: 直接检测 + 导航 (小车会以 0.05 m/s 移动)
python3 /tmp/test_nav.py
# 等价手动流程:
ros2 service call /pick_place/detect_apple std_srvs/srv/Trigger
ros2 topic pub --once /pick_place/nav_object geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.9, y: 0.3, z: 0.25}, orientation: {w: 1.0}}}"
ros2 service call /pick_place/navigate_near std_srvs/srv/Trigger

# 测试返回起点
ros2 service call /pick_place/record_start std_srvs/srv/Trigger
ros2 service call /pick_place/go_home std_srvs/srv/Trigger
```

> Nav2 路线仅当 `map` 帧 TF 存在时启用；本仓库默认不带 Nav2 栈，
> `navigate_near` 始终走视觉伺服（`navigate` / `go_home` 走点位驱动）。
