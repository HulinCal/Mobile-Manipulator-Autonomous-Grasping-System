# wheeltec_mycobot_description

聚合 **WHEELTEC R680 移动底盘**（SDK 中的 `senior_4wd_bs_robot` 四轮差速模型）与
**myCobot 280 六轴机械臂**（含自适应夹爪）的机器人描述包，可在 RViz2 中显示，
并为后续 MoveIt 2 运动规划保留完整的物理与关节特性。Gazebo 默认不启动，
在未安装 Gazebo 的主机（如 NUC11 i5）上同样可以正常编译和运行。

## 功能

- 将 myCobot 280 直接安装在 WHEELTEC 底盘的 `base_link` 坐标系上
  （固定关节 `base_link_to_link1`）；机械臂**最底下的 G 型独立安装底座
  （g_shape_base）已移除**，装车时不使用该底座。
  安装位姿（base_link 系）：
  `xyz="0.08583451891873 0.000745202407130424 0.167204"`、
  `rpy="0 0 -1.5707963267948966"`，即在原 `camera_link` 安装点
  （`xyz="0.17583451891873 0.000745202407130424 0.192704"`、`rpy="0 0 0"`）
  基础上：
  **X 向（小车正前方）后移 0.09 m、Z 向（高度）净降低 0.0255 m**
  （先降 0.035 m，经装车确认偏低后回升 0.0095 m）；
  **绕 Z 轴顺时针旋转 90°（yaw = −π/2，俯视方向）**，使机械臂正前方
  （其 +Y 轴）与小车正前方（base_link +X 轴）重合。
- 一个 launch 文件启动完整可视化：`robot_state_publisher` +
  `joint_state_publisher_gui`（关节滑块）+ RViz2。
- 机械臂、夹爪、底盘轮子均可通过滑块在 RViz2 中拖动查看。
- 所有 link 的质量/惯量、关节类型/轴向/限位（effort、velocity、position）、
  dynamics 均直接复用两个原始包，不做任何数值修改，保证与实物及后续
  MoveIt 2 配置一致。
- 不修改 `wheeltec_robot_urdf`、`mycobot_description` 等任何原有包，
  仅通过 xacro 包含/宏实例化的方式聚合。

## 包结构

```
wheeltec_mycobot_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── view_wheeltec_mycobot.launch.py   # 可视化启动文件（Gazebo 默认关闭）
├── rviz/
│   └── wheeltec_mycobot.rviz             # RViz2 配置（Fixed Frame: base_footprint）
└── urdf/
    └── wheeltec_mycobot.urdf.xacro       # 聚合机器人模型入口
```

## 模型组成与坐标关系

聚合模型通过 [urdf/wheeltec_mycobot.urdf.xacro](urdf/wheeltec_mycobot.urdf.xacro) 组装：

1. 直接包含原始底盘文件
   `wheeltec_robot_urdf/urdf/senior_4wd_bs_robot.urdf`（原样、未改动），
   提供 `base_link`、四个轮子（`lf/lb/rf/rb_wheel_link`）、`camera_link`、
   `laser_link`、`controller_link` 及全部 mesh 与惯量参数。
2. 机械臂部分通过 `mycobot_description` 的原始 xacro 宏实例化：
   - `mycobot_280_arm.urdf.xacro`（6 个转动关节 `link1_to_link2` …
     `link6_to_link6_flange`，限位 ±2.879793 rad / 末端 ±3.05 rad），
     宏的根链接参数传 `base_link`，由宏生成固定关节
     **`base_link_to_link1`** 把机械臂第一节直接挂到底盘上；安装偏移
     为在原 camera_link 点位基础上 X 后移 0.09 m、Z 净降低 0.0255 m
     （先降 0.035 m，再回升 0.0095 m），并绕 Z 顺时针转 90°
     （yaw = −π/2）使机械臂 +Y 朝向与小车 +X 正前方重合
   - **不包含** `g_shape_base_v2_0.urdf.xacro`：装车不使用独立 G 型底座
   - `adaptive_gripper.urdf.xacro`（自适应夹爪，含 mimic 关节）
   - `mycobot_280_ros2_control.urdf.xacro`（ros2_control 接口声明，RViz 下不加载硬件）
3. 额外补充 `base_footprint → base_link` 的固定变换（`xyz="0 0 0.05"`），
   与 `turn_on_wheeltec_robot.launch.py` 中的静态发布保持一致，使整棵 TF
   树都由 `robot_state_publisher` 发布。

TF 树（节选）：

```
base_footprint
└── base_link
    ├── link1 → … → link6_flange → gripper_*   # base_link_to_link1 固定连接，无 G 型底座
    ├── camera_link                            # 底盘自带相机，与机械臂平级
    ├── laser_link
    ├── controller_link
    ├── lf/lb/rf/rb_wheel_link
```

活动关节（共 17 个）：

- 底盘：`lf/lb/rf/rb_wheel_joint`（continuous）
- 机械臂：`link1_to_link2`、`link2_to_link3`、`link3_to_link4`、
  `link4_to_link5`、`link5_to_link6`、`link6_to_link6_flange`（revolute）
- 夹爪：`gripper_controller` 及 5 个联动（mimic）关节

> 说明：移除 G 型底座后，机械臂直接挂在底盘的 `base_link` 下（与底盘
> 自带的 `camera_link` 平级）；机械臂 link/joint 名称与独立
> myCobot 280 模型 **完全一致**，MoveIt 2 的 SRDF/控制器配置可以直接复用。

## 依赖

必需（NUC 主机上具备即可）：

- ROS 2（已在 Jazzy 验证）
- `xacro`、`robot_state_publisher`、`joint_state_publisher`、
  `joint_state_publisher_gui`、`rviz2`
- 工作区内的 `mycobot_description`、`wheeltec_robot_urdf`（提供宏与 mesh）

可选：

- `ros_gz_sim`（新版 Gazebo/Ignition）：**仅当**使用 `use_gazebo:=true` 时需要。
  本包未把它写入 package.xml 硬依赖，默认启动链路不会查找它，因此没有安装
  Gazebo 的主机也能正常 `colcon build` 与 RViz 可视化。

## 编译

在工作空间根目录：

```bash
cd ~/wheeltec/wheeltec_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wheeltec_mycobot_description
source install/setup.bash
```

单独校验模型（不依赖启动文件）：

```bash
xacro src/wheeltec_mycobot_aggregation/wheeltec_mycobot_description/urdf/wheeltec_mycobot.urdf.xacro \
  -o /tmp/wheeltec_mycobot.urdf
check_urdf /tmp/wheeltec_mycobot.urdf
```

## 使用方法

### 1. RViz2 可视化（默认，无需 Gazebo）

```bash
source ~/wheeltec/wheeltec_ws/install/setup.bash
ros2 launch wheeltec_mycobot_description view_wheeltec_mycobot.launch.py
```

启动后包含：robot_state_publisher、joint_state_publisher_gui（关节滑块）、RViz2。
在 RViz2 中可看到完整的底盘 + 机械臂 + 夹爪，拖动滑块可控制各关节。

### 2. 启动参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `jsp_gui` | `true` | 是否启动关节滑块 joint_state_publisher_gui |
| `use_jsp` | `false` | 是否启动无界面的 joint_state_publisher |
| `use_gripper` | `true` | 是否安装自适应夹爪 |
| `use_sim_time` | 随 `use_gazebo` | 是否使用仿真时钟 |
| `rviz_config_file` | 包内 rviz 配置 | 自定义 RViz2 配置路径 |
| `urdf_model` | 包内聚合 xacro | 自定义模型文件路径 |
| `use_gazebo` | **`false`** | 是否启动 Gazebo 并生成模型（仅显示） |
| `headless` | `false` | Gazebo 仅运行 server（无 GUI），需配合 `use_gazebo:=true` |
| `world_file` | 空（默认空世界） | Gazebo world 文件路径 |
| `x`/`y`/`z` | `0`/`0`/`0` | Gazebo 中模型生成位置 |
| `robot_name` | `wheeltec_mycobot` | Gazebo 实体名称 |

示例：

```bash
# 不带夹爪、无滑块，仅 RViz
ros2 launch wheeltec_mycobot_description view_wheeltec_mycobot.launch.py \
  use_gripper:=false jsp_gui:=false

# 完全无头（只发 TF/模型话题，不弹窗）
ros2 launch wheeltec_mycobot_description view_wheeltec_mycobot.launch.py \
  use_rviz:=false jsp_gui:=false use_jsp:=true

# 已安装 ros_gz_sim 的主机上，在 Gazebo 中显示（Gazebo 为可选功能）
ros2 launch wheeltec_mycobot_description view_wheeltec_mycobot.launch.py \
  use_gazebo:=true
```

### 3. 话题与参数

- `/robot_description`：聚合后的 URDF（由 xacro 生成）
- `/joint_states`：关节状态（滑块发布）
- `/tf`、`/tf_static`：完整运动链坐标变换

## 后续 MoveIt 2 规划说明

- 机械臂 6 个关节的名称、轴向、限位、速度/力矩上限与独立
  `mycobot_280` 完全相同，`mycobot_moveit_config` 中的 SRDF、
  `joint_limits.yaml`、kinematics 配置可直接复用。
- 新建 MoveIt 配置或生成 SRDF 时注意：
  - 机械臂运动链从固定关节 `base_link_to_link1` 开始，规划组的
    活动关节仍是 6 个 `link*_to_link*` 关节；在 Setup Assistant 中
    机械臂 base link 可直接选 `link1`（其上方 `base_link → link1`
    为固定连接，含装车偏移）；
  - 末端法兰仍为 `link6_flange`，夹爪基座为 `gripper_base`；
  - 模型已包含 `ros2_control` 声明：真实硬件使用
    `mycobot280_moveit2_control/MyCobot280HardwareInterface`，
    生成 MoveIt 配置时 xacro 传 `use_gazebo:=false` 即可。
- 模型默认不包含 `world` 链接与虚拟关节；如 MoveIt 需要，
  可在后续配置中添加 `world → base_footprint`（或 odom）的虚拟关节。

## 备注

- 本包只做“描述聚合 + 可视化”，不包含底盘/机械臂的硬件驱动与控制器激活。
- 若实际 R680 车型在 `robot_mode_description.launch.py` 中对应其他 URDF
  （而非当前激活的 `senior_4wd_bs_robot`），只需修改
  [urdf/wheeltec_mycobot.urdf.xacro](urdf/wheeltec_mycobot.urdf.xacro) 中的
  `<xacro:include>` 底盘文件名，并相应核对安装关节的父链接即可。
