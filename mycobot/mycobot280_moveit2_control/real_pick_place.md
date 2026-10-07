# mycobot280 真机 MTC 抓取放置全流程

本文档记录 `mycobot280_moveit2_control` 包通过 MoveIt Task Constructor (MTC) 规划抓取放置任务、并驱动真实 mycobot280 M5 机械臂（含夹爪）执行的全链路实现。适合想学习"从规划到真机执行"完整代码实现的读者。

## 1. 总览

### 1.1 目标

在真实桌面场景下完成一次完整的 pick & place：
- 感知（或硬编码）目标物体位姿
- MTC 规划出包含 20+ 子阶段的完整轨迹
- 通过 ros2_control 硬件接口下发给真实机械臂 + 夹爪
- RViz 同步显示真机位姿

### 1.2 启动入口

```bash
ros2 launch mycobot280_moveit2_control mycobot_280_real_pick_place.launch.py
```

单条命令拉起整个栈：真机驱动、controller_manager、move_group、RViz、（假或真）感知 server、mtc_node。

## 2. 数据链路总图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          mtc_node (C++)                                  │
│  1. setupPlanningScene() -> service call -> 拿到目标物体 CollisionObject│
│  2. createTask() -> 串联 20+ MTC stages (CurrentState..MoveHome)        │
│  3. task_.plan() -> 求解每个 stage 的轨迹                                │
│  4. task_.execute(*solution) -> 通过 trajectory_execution manager 下发  │
└───────────────┬─────────────────────────────────────┬──────────────────┘
                │ FollowJointTrajectory goal          │ GripperCommand goal
                ▼                                     ▼
   /arm_controller/follow_joint_trajectory   /gripper_action_controller/gripper_cmd
                │                                     │
                ▼                                     ▼
   ┌────────────────────────────┐   ┌────────────────────────────┐
   │ arm_controller              │   │ gripper_action_controller   │
   │ (JointTrajectoryController) │   │ (GripperActionController)   │
   └──────────────┬─────────────┘   └──────────────┬─────────────┘
                  │ 写入 command_interface          │ 写入 command_interface
                  ▼                                 ▼
   ┌──────────────────────────────────────────────────────────────────┐
   │ MyCobot280HardwareInterface (ros2_control plugin, C++)             │
   │  - read()   : 从 latest_state_* 把状态拷到 hw_states_position_     │
   │  - write()  : 把 hw_commands_position_ 拷到 pending_cmd_ /        │
   │                pending_gripper_cmd_                                │
   │  - wall_timer (20Hz): 把 pending_* 通过 RealtimePublisher 发出     │
   └──────────────┬───────────────────────────────────┬───────────────┘
                  │ Float64MultiArray                  │ Float64MultiArray
                  ▼ /mycobot/cmd_joint_pos            ▼ /mycobot/cmd_gripper_pos
   ┌──────────────────────────────────────────────────────────────────┐
   │ mycobot_driver_node.py  (Python, pymycobot)                       │
   │  - cmd_callback        : 弧度 -> 度 -> mc.send_angles()           │
   │  - gripper_cmd_callback: 弧度 -> 0..100 -> mc.set_gripper_value()│
   │  - publish_state (20Hz): mc.get_angles() -> 弧度 -> /current_*   │
   │                          mc.get_gripper_value() -> 弧度 -> 同上    │
   └──────────────┬───────────────────────────────────────────────────┘
                  │ 串口 /dev/ttyACM0  (115200 baud)
                  ▼
          ┌──────────────────┐
          │  mycobot280 M5   │
          │  6DOF arm + grip │
          └──────────────────┘
```

数据回流（状态）：
```
真机 -> pymycobot.get_angles()/get_gripper_value() -> driver publish_state
-> /mycobot/current_joint_pos + /current_gripper_pos
-> HardwareInterface state_callback / gripper_state_callback
-> latest_state_ / latest_gripper_state_ (mutex 保护)
-> read() 拷到 hw_states_position_
-> StateInterface -> joint_state_broadcaster -> /joint_states
-> move_group PlanningScene monitor + RViz 显示
```

## 3. 关键节点详解

### 3.1 mtc_node（任务规划器，C++）

**源码**：[mycobot_mtc_pick_place_demo/src/mtc_node.cpp](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_mtc_pick_place_demo/src/mtc_node.cpp)

MTC 任务由 `MTCTaskNode::createTask()` 串联 20+ stage 组成。每个 stage 是一个独立的规划子问题，前一个 stage 的终点是后一个 stage 的起点。

#### 3.1.1 三类 stage

MTC 把 stage 分三类：
- **PropagatingForward**：输入起点，规划到终点（轨迹向前传播）
- **Connect**：把两个任意状态连接起来（前向 + 后向）
- **Generator**：自己生成多个候选起点/终点（如 grasp pose 候选）

#### 3.1.2 任务结构（完整 stage 顺序）

```
pick_place_task (SerialContainer)
├── current state                 [CurrentState]         起点快照
├── open gripper                  [MoveTo]               张开夹爪
├── move to pick                  [Connect]              大臂到 pre-grasp
└── pick object (SerialContainer)
    ├── approach object           [MoveRelative, Cart]    沿 z 直线接近
    ├── grasp pose IK             [ComputeIK(GenerateGraspPose)]
    │                             绕物体 z 轴每 7.5° 采样 48 个候选姿态
    ├── allow collision (gripper,object) [ModifyPlanningScene]
    ├── close gripper             [MoveTo]               半闭合夹紧
    ├── allow collision (object,support_surface)
    ├── attach object             [ModifyPlanningScene]  物体挂到 gripper
    ├── lift object               [MoveRelative, Cart]   沿 z 抬起
    └── forbid collision (object,support_surface)
├── move to place                 [Connect]              搬运到放置点上方
└── place object (SerialContainer)
    ├── lower object              [MoveRelative, Cart]   沿 z 下放
    ├── place pose IK             [ComputeIK(GeneratePlacePose)]
    ├── open gripper              [MoveTo]               松开
    ├── forbid collision (gripper,object)
    ├── detach object             [ModifyPlanningScene]  物体脱离
    └── retreat after place       [MoveRelative, Cart]   沿 -z 退离
└── move home                     [MoveTo]               回 home
```

#### 3.1.3 三种规划器

```cpp
// mtc_node.cpp:447-466
ompl_planner_arm       = PipelinePlanner (OMPL RRTConnect)  // 大臂运动规划
interpolation_planner  = JointInterpolationPlanner            // 夹爪开合
cartesian_planner      = CartesianPath                        // 接近/抬起/下放/退离
```

#### 3.1.4 关键参数（mtc_node_params.yaml）

**源码**：[mycobot_mtc_pick_place_demo/config/mtc_node_params.yaml](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_mtc_pick_place_demo/config/mtc_node_params.yaml)

```yaml
# 物体（圆柱）
object_dimensions: [0.35, 0.0125]    # [height, radius] (m)
object_pose: [0.22, 0.12, 0.10, 0.0, 0.0, 0.0]    # xyz + RPY, base_link

# 夹爪 TCP 相对 link6_flange 的变换
#   xyz=[0,0,0.13]  z=0.13m 让指尖正好夹住物体侧面（0.096 会碰撞，0.13 合适）
#   RPY=[0, 1.5708, 0] pitch=90° 让 gripper 水平接近物体
grasp_frame_transform: [0.0, 0.0, 0.13, 0.0, 1.5708, 0.0]

# 放置点
place_pose: [-0.183, -0.14, 0.10, 0.0, 0.0, 0.0]

# 抓取姿态候选
grasp_pose_angle_delta: 0.1309   # 7.5° 一个候选，48 个候选
grasp_pose_max_ik_solutions: 10

# SRDF group_state 名
gripper_open_pose: "open"          # 对应 gripper_controller=0.15 (100% 张开)
gripper_close_pose: "half_closed"  # 对应 gripper_controller=-0.275 (50% 半开)

# 执行开关
execute: true   # false 时只规划不执行，可在 RViz 检查 Solution
```

#### 3.1.5 执行入口（不经过 move_group）

关键点：**MTC 执行不走 move_group 的 action**，而是直接通过 `trajectory_execution` manager 下发：

```cpp
// mtc_node.cpp:355-365
auto result = task_.execute(*task_.solutions().front());
```

`task_.execute()` 内部读取 stage 的 `trajectory_execution_info` 属性，根据 `controller_names` 把每段子轨迹直接发到对应 controller 的 action server：
- `arm_controller` 段 → `/arm_controller/follow_joint_trajectory` (FollowJointTrajectory action)
- `gripper_action_controller` 段 → `/gripper_action_controller/gripper_cmd` (GripperCommand action)

所以只要 controller_manager + driver 在跑，MTC 就能直接驱动真机。

### 3.2 MyCobot280HardwareInterface（C++ 桥接插件）

**源码**：
- [mycobot280_moveit2_control/include/.../mycobot280_hardware_interface.hpp](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/include/mycobot280_moveit2_control/mycobot280_hardware_interface.hpp)
- [mycobot280_moveit2_control/src/mycobot280_hardware_interface.cpp](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/src/mycobot280_hardware_interface.cpp)

这是个 `hardware_interface::SystemInterface` 插件，承担两个角色：
1. **ros2_control 与 Python driver 之间的桥**：通过 topic 互传数据
2. **实时层与非实时层隔离**：write() 只做内存拷贝，publish 在独立 wall_timer 线程

#### 3.2.1 on_init：识别关节、解析参数

```cpp
// mycobot280_hardware_interface.cpp:65-79
// 6 个 arm 关节 + 可选的 gripper_controller
gripper_joint_index_ = kNoGripper;
for (size_t i = 0; i < joints.size(); ++i) {
  if (joints[i].name == "gripper_controller") {
    gripper_joint_index_ = i;
    break;
  }
}
```

从 URDF `<param>` 读：`cmd_topic`、`state_topic`、`gripper_cmd_topic`、`gripper_state_topic`、`cmd_publish_rate`（默认 20Hz）、`dry_run`。

#### 3.2.2 on_export_state_interfaces / on_export_command_interfaces

```cpp
// 为每个 joint 暴露 position command/state interface
// gripper 额外暴露 velocity state interface（值恒为 0），
//   因为 GripperActionController 要求它
```

#### 3.2.3 on_configure：建 publisher/subscriber/wall_timer

```cpp
// mycobot280_hardware_interface.cpp:196-213
cmd_pub_ = node->create_publisher<Float64MultiArray>(cmd_topic_, 5);
realtime_cmd_pub_ = make_shared<RealtimePublisher<>>(cmd_pub_);
state_sub_ = node->create_subscription<Float64MultiArray>(
    state_topic_, 5, &MyCobot280HardwareInterface::state_callback, this);

if (gripper_joint_index_ != kNoGripper) {
  gripper_cmd_pub_ = ...;  realtime_gripper_cmd_pub_ = ...;
  gripper_state_sub_ = ...;
}
```

wall_timer 周期 = 1/cmd_publish_rate_ 秒。timer 回调做实际 publish：
```cpp
// mycobot280_hardware_interface.cpp:215-245
{
  std::lock_guard<std::mutex> g(pending_cmd_mutex_);
  cmd = pending_cmd_;   // 拷贝
}
if (realtime_cmd_pub_->trylock()) {
  realtime_cmd_pub_->msg_.data.assign(cmd.begin(), cmd.end());
  realtime_cmd_pub_->unlockAndPublish();
}
// gripper 同上
```

#### 3.2.4 read() / write()

```cpp
// read() : latest_state_ -> hw_states_position_
// write(): hw_commands_position_ -> pending_cmd_  (纯内存拷贝，非阻塞)
```

线程安全用 mutex 保护 `latest_state_`、`latest_gripper_state_`、`pending_cmd_`、`pending_gripper_cmd_`。

### 3.3 mycobot_driver_node.py（Python 串口驱动）

**源码**：[mycobot280_moveit2_control/src/mycobot_driver_node.py](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/src/mycobot_driver_node.py)

直接调 pymycobot 库与 M5 串口通信。两个 callback + 一个定时器：

#### 3.3.1 cmd_callback（手臂）
```python
# mycobot_driver_node.py:91-99
angles_deg = [math.degrees(a) for a in msg.data]   # 弧度 -> 度
self.mc.send_angles(angles_deg, 30)                # 速度 30
```

#### 3.3.2 gripper_cmd_callback（夹爪）
```python
# mycobot_driver_node.py:102-116
radians = msg.data[0]
# URDF joint limit [-0.7, 0.15] -> pymycobot 0..100
value = int(round((radians - (-0.7)) / (0.15 - (-0.7)) * 100))
self.mc.set_gripper_value(value, 30)
```

#### 3.3.3 publish_state（20Hz 定时器）
```python
# 20Hz 太低会卡顿；50Hz 会让 M5 来不及响应触发 "device disconnected"
angles_deg = self.mc.get_angles()
if isinstance(angles_deg, (list, tuple)) and len(angles_deg) == 6:
    angles_rad = [math.radians(a) for a in angles_deg]
    self.state_pub.publish(Float64MultiArray(data=angles_rad))

gripper_value = self.mc.get_gripper_value()
if isinstance(gripper_value, (int, float)) and 0 <= gripper_value <= 100:
    # 0..100 -> 弧度 [-0.7, 0.15]
    gripper_radians = gripper_value / 100.0 * 0.85 + (-0.7)
    self.gripper_state_pub.publish(Float64MultiArray(data=[gripper_radians]))
```

`isinstance` 检查是因为 pymycobot 在串口异常时返回 int 错误码（0/1），不是 list。

### 3.4 controller_manager 与三个控制器

**配置**：[mycobot280_moveit2_control/config/mycobot_280_real_controllers.yaml](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/config/mycobot_280_real_controllers.yaml)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100   # Hz

    arm_controller:               # JointTrajectoryController
      type: joint_trajectory_controller/JointTrajectoryController
    gripper_action_controller:    # GripperActionController
      type: position_controllers/GripperActionController
    joint_state_broadcaster:     # 发布 /joint_states
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints: [link1_to_link2, ..., link6_to_link6_flange]
    command_interfaces: [position]
    state_interfaces: [position]
    goal: [0.05]    # 0.05 rad (~2.9°) 容差
    path: [0.05]

gripper_action_controller:
  ros__parameters:
    joint: gripper_controller
    goal_tolerance: 1.0    # 1 单位（0..100 中的 1）
    max_effort: 100.0
```

**激活顺序**（launch 文件里 RegisterEventHandler 串联）：
1. `joint_state_broadcaster`（先激活，让 /joint_states 开始流）
2. `arm_controller`（delay 2s）
3. `gripper_action_controller`（再 delay 1s）

总耗时约 3s，所以 mtc_node 启动延迟设为 8s 给足余量。

## 4. URDF 与 SRDF 改动

### 4.1 URDF：gripper_controller 关节

**源码**：[mycobot_description/urdf/control/mycobot_280_ros2_control.urdf.xacro](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_description/urdf/control/mycobot_280_ros2_control.urdf.xacro)

真机分支（`use_gazebo:=false`）下新增：

```xml
<joint name="gripper_controller">
  <command_interface name="position">
    <param name="min">-0.7</param>
    <param name="max">0.15</param>
  </command_interface>
  <state_interface name="position"/>
  <state_interface name="velocity"/>   <!-- GripperActionController 要求 -->
</joint>
```

范围 `[-0.7, 0.15]` 弧度对应 pymycobot 0..100：
- `-0.7` → 0（完全闭合）
- `0.15` → 100（完全张开）

### 4.2 SRDF：三个 gripper group_state

**源码**：[mycobot_moveit_config/config/mycobot_280/mycobot_280.srdf](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_moveit_config/config/mycobot_280/mycobot_280.srdf)

```xml
<group_state name="open" group="gripper">
  <joint name="gripper_controller" value="0.15"/>      <!-- 100 张开 -->
</group_state>
<group_state name="half_closed" group="gripper">
  <joint name="gripper_controller" value="-0.275"/>    <!-- 50 半开，夹取时用 -->
</group_state>
<group_state name="closed" group="gripper">
  <joint name="gripper_controller" value="-0.7"/>      <!-- 0 闭合 -->
</group_state>
```

MTC 的 `open gripper` stage 调 `setGoal("open")` 就是设到这些值。

### 4.3 SRDF：夹爪自碰撞禁用

新增两对 disable_collisions，避免 `CheckStartStateCollision` 报 `gripper_left1 - gripper_right1` 等碰撞：

```xml
<disable_collisions link1="gripper_right1" link2="gripper_left1" reason="Never"/>
<disable_collisions link1="gripper_right1" link2="gripper_right2" reason="Never"/>
```

## 5. launch 文件分解

**源码**：[mycobot280_moveit2_control/launch/mycobot_280_real_pick_place.launch.py](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/launch/mycobot_280_real_pick_place.launch.py)

组合两个现有 launch + 一个延迟启动的 mtc_node：

### 5.1 启动真机栈（复用本包的 mycobot_280_real_moveit.launch.py）

```python
real_stack_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(pkg_share_control, 'launch', 'mycobot_280_real_moveit.launch.py')
    ]),
    launch_arguments={
        'robot_name': robot_name, 'use_rviz': use_rviz,
        'use_sim_time': 'false',
        'driver_port': driver_port, 'driver_baud': driver_baud,
    }.items(),
)
```

这一步拉起：`robot_state_publisher`、`controller_manager`、`mycobot_driver_node.py`、`move_group`、`rviz2`。

### 5.2 启动（假或真）感知 server

```python
# 真感知：use_perception:=true
real_perception_launch = IncludeLaunchDescription(...,
    condition=IfCondition(use_perception))

# 假感知：use_perception:=false (默认)
fake_perception_node = Node(
    package='mycobot280_moveit2_control',
    executable='fake_planning_scene_server.py',
    parameters=[mtc_node_params_file_path],
    condition=IfCondition(PythonExpression(["'", use_perception, "' == 'false'"])))
```

两个互斥。默认用 fake，立即响应 service 返回硬编码 CollisionObject，避免 mtc_node 卡在 service 等待。

### 5.3 延迟启动 mtc_node

```python
mtc_node_action = OpaqueFunction(function=_build_mtc_node)
delayed_mtc = TimerAction(period=mtc_start_delay, actions=[mtc_node_action])   # 8s
```

`_build_mtc_node` 内部用 `MoveItConfigsBuilder` 构造完整的 MoveIt 参数（SRDF、kinematics、joint_limits、planning_pipelines 等）注入 mtc_node。

延迟 8s 是因为 controller 顺序激活需要 ~3s，mtc_node 一启动就立刻调 service + plan + execute，没激活会失败。

## 6. 真实场景感知接入

默认 `use_perception:=false` 用假 server，物体位姿硬编码在 `mtc_node_params.yaml` 的 `object_pose`。要接入真实场景感知，启动 `use_perception:=true` 即可，整条链路如下：

```
                      真实场景
                         │
                         ▼
              ┌─────────────────────┐
              │ RGB-D 相机           │
              │ (RealSense / Astra)  │
              └──────────┬──────────┘
                         │ sensor_msgs/PointCloud2
                         ▼
              /camera_head/depth/color/points
                         │
                         ▼
   ┌──────────────────────────────────────────────────┐
   │ get_planning_scene_server (C++)                  │
   │  1. crop_box 裁掉机械臂自身                       │
   │  2. SAC 平面分割 -> support_surface (桌面)        │
   │  3. 欧式聚类 + 区域生长 -> 物体 cluster            │
   │  4. 形状拟合 (cylinder/box RANSAC)               │
   │  5. 用 w_inliers/size/distance/orientation 打分  │
   │     选最佳匹配 target_shape 的物体               │
   │  6. 把物体 + 桌面打包成 PlanningSceneWorld        │
   └──────────────────┬───────────────────────────────┘
                      │ service response: GetPlanningScene
                      ▼
   ┌──────────────────────────────────────────────────┐
   │ mtc_node                                          │
   │  setupPlanningScene():                            │
   │    planning_scene_client->call_service(           │
   │        target_shape="cylinder",                   │
   │        target_dimensions=[0.35, 0.0125])          │
   │  -> updateObjectParameters() 从响应里读真实       │
   │     object_name / type / dimensions / pose        │
   │  -> psi.applyCollisionObjects(scene_world_)       │
   │     把真实场景加入 PlanningScene                  │
   │  -> createTask() / plan() / execute()             │
   └──────────────────────────────────────────────────┘
```

### 6.1 相机配置

`get_planning_scene_server.yaml` 里指定了相机 topic 和处理参数：

```yaml
# mycobot_mtc_pick_place_demo/config/get_planning_scene_server.yaml
point_cloud_topic: "/camera_head/depth/color/points"
rgb_image_topic:   "/camera_head/color/image_raw"
target_frame:      "base_link"

# 裁掉机械臂自身（只保留桌面区域）
enable_cropping: true
crop_min_x: 0.10   crop_max_x: 1.1
crop_min_y: 0.00   crop_max_y: 0.90

# 平面分割
max_iterations: 100
distance_threshold: 0.01
z_tolerance: 0.03

# 聚类
min_cluster_size: 150
cluster_tolerance: 0.02

# 形状拟合
shape_fitting_min_radius: 0.01
shape_fitting_max_radius: 0.1
```

**前提**：相机标定正确，`base_link` 与相机 frame 之间有有效的 TF。如果你的相机 frame 不叫 `/camera_head/...`，需要改 `point_cloud_topic` 和 `rgb_image_topic`。

### 6.2 启用真实感知

```bash
# 1. 启动相机驱动（保证 /camera_head/depth/color/points 有数据）
#    RealSense: ros2 launch realsense2_camera rs_launch.py ...
#    Astra:     ros2 launch orbbec_camera astra.launch.py ...

# 2. 启动 pick & place（开启感知）
ros2 launch mycobot280_moveit2_control mycobot_280_real_pick_place.launch.py \
    use_perception:=true
```

`use_perception:=true` 时 launch 文件做两件事：
1. 启动 `get_planning_scene_server.launch.py`（真 server）
2. 给 mtc_node 注入 `use_perception:=true` 参数（mtc_node 会调 service 拿真实场景）

### 6.3 感知 service 协议

**接口定义**：[mycobot_interfaces/srv/GetPlanningScene](file:///home/hl/mycobot/mycobot_ros2-main)

```
# Request
string   target_shape          # "cylinder" / "box" / ...
float64[] target_dimensions    # 期望的物体尺寸，用于匹配

# Response
PlanningSceneWorld scene_world  # 桌面 + 所有物体的 CollisionObject
PointCloud2         full_cloud   # 原始点云（调试用）
Image               rgb_image    # RGB 图（调试用）
string              target_object_id    # 匹配上的目标物体 id
string              support_surface_id  # 桌面 id
bool                success
```

mtc_node 的 `updateObjectParameters()` 会从响应里把真实物体的 `id`、`type`、`dimensions` 写回 node 参数，覆盖 `mtc_node_params.yaml` 的硬编码值——后续 stage 用的是真实物体信息。

### 6.4 感知调试

`get_planning_scene_server.yaml` 里 `output_directory: "/tmp/"` 会把每个中间点云存成 PCD：

```
/tmp/1_cropBox_debug_cloud.pcd
/tmp/2_downsample_debug_cloud.pcd
/tmp/3_removeOutliers_debug_cloud.pcd
/tmp/4_convertToPCL_debug_cloud.pcd
/tmp/5_support_plane_debug_cloud.pcd    # 桌面
/tmp/5_objects_cloud_debug_cloud.pcd     # 物体 cluster
/tmp/6_object_cloud_debug_cloud.pcd      # 最终匹配物体
```

用 `point_cloud_viewer.launch.py` 可视化任意中间结果：

```bash
ros2 launch mycobot_mtc_pick_place_demo point_cloud_viewer.launch.py \
    file_name:=/tmp/5_objects_cloud_debug_cloud.pcd
```

### 6.5 感知与硬编码的对比

| 维度 | `use_perception:=false`（假） | `use_perception:=true`（真） |
|---|---|---|
| 启动节点 | `fake_planning_scene_server.py` (Python) | `get_planning_scene_server` (C++) |
| 依赖 | 无 | RGB-D 相机 + 标定 TF |
| service 响应时间 | 即时 | 1~3 秒（分割 + 拟合） |
| object_pose 来源 | `mtc_node_params.yaml` 硬编码 | 相机感知真实位姿 |
| 支持桌面（support_surface） | 无（id 为空） | 有（参与碰撞约束） |
| 适用场景 | 调试、固定工位抓取 | 动态场景、物体位置未知 |

## 7. 调试经验（踩坑记录）

按解决问题顺序排列。每个都是真机调试时实际遇到的卡点。

### 7.1 SRDF group_state 用错单位

**现象**：`open gripper` stage 报 `Start state is out of bounds!`

**根因**：SRDF 里 `gripper_controller` 的值写成弧度 `-0.34/-0.50`，但 URDF limit 是 `[-0.7, 0.15]`（不是 0..100，那是早期版本）。

**修复**：改成 `open=0.15 / half_closed=-0.275 / closed=-0.7`。

### 7.2 Python driver `get_angles()` 返回 int

**现象**：`Error reading arm state: object of type 'int' has no len()`

**根因**：pymycobot 串口异常时返回错误码 `int(0/1)`，不是 list。

**修复**：driver 加 `isinstance(angles_deg, (list, tuple))` 判断。

### 7.3 Arm action server 超时

**现象**：`Arm action server not available, is the launch file running?`

**根因**：3 个 controller 顺序激活需要 ~6s，10s 超时有时不够。

**修复**：`test_arm_movement.py` 的 `wait_for_server` 超时改 30s。

### 7.4 CheckStartStateCollision 失败

**现象**：`gripper_left1 - gripper_right1, gripper_right1 - gripper_right2 contact(s) detected`

**根因**：SRDF 缺夹爪 link 之间的 disable_collisions。

**修复**：加两对 `<disable_collisions ... reason="Never"/>`。

### 7.5 IK 无解（z 太低）

**现象**：`grasp pose IK (0/48): no IK found`

**根因**：`object_pose` z=0.0 是桌面高度，机械臂要极度弯腰才能到，6DOF 臂超限。

**修复**：z 改 0.10（物体抬高 10cm）。

### 7.6 IK 无解（姿态太苛刻）

**现象**：抬高 z 后仍 `no IK found`

**根因**：`grasp_frame_transform` RPY=`[1.5708, 0, 0]`（roll=90°）要求 gripper 侧向抓取，6DOF 臂这种姿态约束超工作空间。

**修复**：改成 `[0, 1.5708, 0]`（pitch=90°，gripper 水平接近物体）。

### 7.7 eef in collision

**现象**：`eef in collision: gripper_right1 - object`

**根因**：`grasp_frame_transform` z=0.096 让指尖正好压在物体侧面。

**修复**：z 改 0.13（指尖退后 3.4cm，夹住物体侧面而不是压上）。

### 7.8 串口多进程占用

**现象**：`device reports readiness to read but returned no data (device disconnected or multiple access on port?)`

**根因**：`lsof /dev/ttyACM0` 显示两个 PID 同时打开串口——上次 launch 没杀干净。

**修复**：`kill -9 <PID>` 清理残留进程后再启动。

### 7.9 RViz 不动但真机动

**现象**：真机执行了但 RViz 模型不更新。

**根因**：同 7.8，串口被占导致 `get_angles()` 读不到状态，`/joint_states` 停在最后一帧。

**修复**：同 7.8。

### 7.10 串口 50Hz 来不及响应

**现象**：50Hz 读写时偶发 `device disconnected`。

**根因**：M5 串口 115200 baud 每秒最多 ~1000 次往返，50Hz × 2（读+写）= 100 次/秒，压力太大。

**修复**：driver `publish_state` 与 HardwareInterface `cmd_publish_rate` 都降到 20Hz。

### 7.11 launch `PythonExpression` 表达式错误

**现象**：`name 'false' is not defined`

**根因**：`PythonExpression([use_perception, ' == false'])` 里 `false` 没引号，Python 当成未定义变量。

**修复**：改成 `PythonExpression(["'", use_perception, "' == 'false'"])`，两边都加引号做字符串比较。

### 7.12 mtc_node 卡在 service 等待

**现象**：`[get_planning_scene_client]: Service not available, waiting again...` 一直循环。

**根因**：真感知 server 启动慢（要等点云），或者根本没启动（默认 `use_perception:=false`）。

**修复**：新增 `fake_planning_scene_server.py`，默认模式下立即响应 service。两个 server 通过 `IfCondition(use_perception)` 互斥切换。

## 8. 常用调试命令

```bash
# 检查 controller 状态
ros2 control list_controllers

# 检查硬件 interface
ros2 control list_hardware_interfaces

# 看 /joint_states（应有 7 个 joint：6 arm + 1 gripper）
ros2 topic echo /joint_states

# 手动发夹爪命令（测试 GripperActionController）
ros2 action send_goal /gripper_action_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.15, max_effort: 100.0}}"

# 看 service 是否 advertised
ros2 service list | grep get_planning_scene

# 看串口占用
lsof /dev/ttyACM0

# 临时把 execute 关掉只规划
# 改 mtc_node_params.yaml: execute: false
# 重启 launch，在 RViz 里检查 MTC Solution
```

## 9. 相关文件清单

| 文件 | 作用 |
|---|---|
| [mycobot_280_real_pick_place.launch.py](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/launch/mycobot_280_real_pick_place.launch.py) | 主 launch，组合真机栈 + 感知 + mtc_node |
| [mycobot_280_real_moveit.launch.py](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/launch/mycobot_280_real_moveit.launch.py) | 真机栈 launch（driver + controllers + move_group + RViz） |
| [mycobot280_hardware_interface.cpp](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/src/mycobot280_hardware_interface.cpp) | C++ 硬件接口插件 |
| [mycobot_driver_node.py](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/src/mycobot_driver_node.py) | Python 串口驱动 |
| [fake_planning_scene_server.py](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/scripts/fake_planning_scene_server.py) | 假感知 server（无相机时用） |
| [mycobot_280_real_controllers.yaml](file:///home/hl/mycobot/mycobot_ros2-main/mycobot280_moveit2_control/config/mycobot_280_real_controllers.yaml) | controller_manager 配置 |
| [mycobot_280_ros2_control.urdf.xacro](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_description/urdf/control/mycobot_280_ros2_control.urdf.xacro) | URDF ros2_control block |
| [mycobot_280.srdf](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_moveit_config/config/mycobot_280/mycobot_280.srdf) | SRDF（含 gripper group_state 与 disable_collisions） |
| [mtc_node.cpp](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_mtc_pick_place_demo/src/mtc_node.cpp) | MTC 任务节点 |
| [mtc_node_params.yaml](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_mtc_pick_place_demo/config/mtc_node_params.yaml) | MTC 任务参数 |
| [get_planning_scene_server.cpp](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp) | 真感知 server |
| [get_planning_scene_server.yaml](file:///home/hl/mycobot/mycobot_ros2-main/mycobot_mtc_pick_place_demo/config/get_planning_scene_server.yaml) | 感知参数（topic、分割阈值等） |
