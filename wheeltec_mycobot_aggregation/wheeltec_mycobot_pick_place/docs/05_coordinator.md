# 5. 协调节点 (coordinator_node)

## 功能

状态机节点，串联整个"触发 → 识别 → 导航靠近 → 抓取（含收臂）→ 倒车返回起点"流程。
收到触发信号后按顺序调用各功能节点的服务，驱动完整流水线。
**回到起点即结束：不放置苹果、不张开夹爪，苹果保持夹在机械臂上**（由现场人工取走）。

## 状态机

```
                    ┌──────────┐
                    │   IDLE   │ ← 等待触发
                    └────┬─────┘
                         │ 收到 /pick_place/trigger 或 /pick_place/start
                         ▼
                    ┌──────────┐
                    │ DETECTING│ → 调用 /pick_place/detect_apple 获取苹果位姿
                    └────┬─────┘
                         │ 记录起点 /pick_place/record_start
                         ▼
                 ┌──────────────┐
                 │ NAV_TO_APPLE │ → 发布苹果位姿→/pick_place/nav_object
                 └──────┬───────┘   调用 /pick_place/navigate_near
                        │
                        ▼
                 ┌──────────┐
                 │ GRASPING │ → 调用 /pick_place/pick
                 └────┬─────┘     (抓取→提升→收臂回 home, 夹爪保持夹紧)
                      │
                      ▼
              ┌────────────────┐
              │ NAV_TO_DELIVERY│ → 调用 /pick_place/go_home
              └───────┬────────┘   (cmd_vel 兜底时只倒退不转弯)
                      │   回到起点 (位置误差 <0.05m)
                      ▼
                ┌──────────┐
                │   DONE   │ → 回到 IDLE (苹果仍夹着, 不放置/不张爪)
                └──────────┘
```

任一阶段失败则进入 `ERROR` 状态并停止。

> `/pick_place/place` 服务和 `/pick_place/place_pose` 话题在 grasp_node 中仍保留，
> 需要单独放置时可手动调用，但自动流水线不再执行放置阶段。

## 话题与服务

| 类型 | 名称 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `/pick_place/trigger` | `Bool` | 触发流程 |
| 发布 | `/pick_place/state` | `String` | 当前状态名 |
| 发布 | `/pick_place/status` | `String` | 人类可读状态 |
| 发布 | `/pick_place/nav_object` | `PoseStamped` | 传给导航的物体位姿 |
| 发布 | `/pick_place/nav_goal` | `PoseStamped` | 导航目标位姿 |
| 发布 | `/pick_place/apple_pose` | `PoseStamped` | 苹果位姿（转发） |
| 发布 | `/pick_place/place_pose` | `PoseStamped` | 放置位姿 |
| 服务 | `/pick_place/start` | `Trigger` | 手动启动流程 |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sim_mode` | `true` | 模拟模式 |
| `delivery_offset_x` | `0.5` | 送达点 X 偏移 [m] |
| `delivery_offset_y` | `0.5` | 送达点 Y 偏移 [m] |
| `delivery_offset_yaw` | `0.0` | 送达点偏航角偏移 [rad] |
| `auto_return` | `true` | 是否自动返回起点 |

## 测试方法

```bash
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py sim_mode:=true

# 启动流程
ros2 service call /pick_place/start std_srvs/srv/Trigger

# 实时监控状态
ros2 topic echo /pick_place/state
ros2 topic echo /pick_place/status
```

## 监控完整流程

```bash
# 终端1：启动
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py sim_mode:=true

# 终端2：监控状态
ros2 topic echo /pick_place/state

# 终端3：触发
ros2 service call /pick_place/trigger_manual std_srvs/srv/Trigger
```
