# wheeltec_mycobot_bringup

## 简介

本包为聚合机器人 **WHEELTEC R680 + myCobot 280** 提供 bringup（启动）launch 文件。

聚合机器人由 WHEELTEC R680 四驱移动底盘（`senior_4wd_bs`）与 myCobot 280 六自由度机械臂组合而成，机器人描述位于 `wheeltec_mycobot_description` 包。本包负责将描述与各功能节点串联起来，提供相机、激光雷达、机械臂控制器、底盘控制器以及全功能 all-in-one 启动方式。

> **重要**：本包仅新建 launch / rviz 文件，**不修改** `wheeltec_mycobot_description`、`turn_on_wheeltec_robot`、`astra_camera`、`lslidar_driver`、`mycobot280_moveit2_control`、`mycobot_moveit_config` 等任何其它包的文件。所有传感器、底盘、机械臂驱动与配置均直接复用各自原包的资源。

## ROS2 版本与构建

- ROS2 发行版：**Jazzy**
- 构建系统：`colcon` + `ament_cmake`

## 依赖

本包在运行时依赖以下包（见 `package.xml`）：

| 依赖包 | 用途 |
| --- | --- |
| `wheeltec_mycobot_description` | 聚合机器人 URDF / xacro / 网格 |
| `turn_on_wheeltec_robot` | 底盘串口驱动 `base_serial.launch.py`、EKF 配置 `config/ekf.yaml` |
| `astra_camera` | Astra Pro 深度相机驱动 `astra_pro.launch.xml` |
| `lslidar_driver` | LSLidar N10 激光雷达驱动 `lsn10_launch.py` |
| `mycobot280_moveit2_control` | 机械臂 ros2_control 硬件接口、Python 驱动节点、控制器配置 |
| `mycobot_moveit_config` | MoveIt2 `move_group.launch.py` |
| `robot_localization` | EKF 状态估计节点 |
| `rviz2` | 可视化 |
| `joint_state_publisher` | 无头关节状态发布（真实硬件） |
| `joint_state_publisher_gui` | GUI 关节滑块（可选，相机/雷达 launch 中 `jsp_gui:=true` 时启动） |
| `tf2_ros` | 静态坐标变换发布 |
| `controller_manager` | ros2_control 控制器管理节点 |
| `robot_state_publisher` | 发布 `/robot_description` 与 TF |
| `xacro` | xacro → URDF 处理 |

## 构建

```bash
cd ~/wheeltec/wheeltec_ws
colcon build --packages-select wheeltec_mycobot_bringup
source install/setup.bash
```

## 目录结构

```
wheeltec_mycobot_bringup/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── camera.launch.py
│   ├── lidar.launch.py
│   ├── arm_controller.launch.py
│   ├── car_controller.launch.py
│   └── all.launch.py
├── rviz/
│   ├── wheeltec_mycobot_camera.rviz
│   ├── wheeltec_mycobot_lidar.rviz
│   ├── wheeltec_mycobot_car_controller.rviz
│   └── wheeltec_mycobot_all.rviz
└── README.md
```

## 启动文件说明

所有 launch 文件均使用 `FindPackageShare` + `PathJoinSubstitution` 惰性求值定位资源包，即使某些包未安装也不会在 launch 解析阶段报错（仅在实际启动对应节点时才需要对应包）。

聚合机器人 xacro 固定以 `use_gazebo:=false` `use_gripper:=true` 参数处理，确保加载真实硬件插件与夹爪。

### 1. `camera.launch.py` — 相机可视化

启动 RSP + 无头 `joint_state_publisher`（默认）+ Astra Pro 相机驱动 + RViz2，用于在 RViz 中同时查看机器人模型与彩色图像。

- 复用 `astra_camera/launch/astra_pro.launch.xml`（通过 `AnyLaunchDescriptionSource` 引入）。
- 默认启动无头 `joint_state_publisher` 发布零位关节状态，保证四个轮子与机械臂各 link 的 TF 完整、模型全部显示。
- `joint_state_publisher_gui` **默认不启动**；需要在无控制器时拖动机械臂关节调试时，加 `jsp_gui:=true` 开启滑块界面（此时自动替代无头版）。
- RViz 配置：`rviz/wheeltec_mycobot_camera.rviz`（含 Image 显示）。

```bash
ros2 launch wheeltec_mycobot_bringup camera.launch.py
ros2 launch wheeltec_mycobot_bringup camera.launch.py use_rviz:=false
ros2 launch wheeltec_mycobot_bringup camera.launch.py jsp_gui:=true
ros2 launch wheeltec_mycobot_bringup camera.launch.py use_jsp:=false
```

**参数**

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `use_jsp` | `true` | 是否启动无头 joint_state_publisher（发布零位，保证模型完整） |
| `jsp_gui` | `false` | 是否启动 joint_state_publisher_gui 关节滑块界面（启动时替代无头版） |

### 2. `lidar.launch.py` — 激光雷达可视化

启动 RSP + 无头 `joint_state_publisher`（默认）+ `base_to_laser` 静态 TF + LSLidar N10 驱动 + RViz2，用于在 RViz 中同时查看机器人模型与激光点云。

- `base_to_laser` 静态 TF：`base_footprint → laser`，`xyz='0.0911 0 0.155'` `rpy='0 0 0'`（与 `robot_mode_description.launch.py` 中 `senior_4wd_bs_robot` 一致）。
- 复用 `lslidar_driver/launch/lsn10_launch.py`。
- 默认启动无头 `joint_state_publisher` 发布零位关节状态，保证轮子与机械臂模型完整显示。
- `joint_state_publisher_gui` **默认不启动**；需要拖动机械臂关节调试时，加 `jsp_gui:=true` 开启滑块界面（此时自动替代无头版）。
- RViz 配置：`rviz/wheeltec_mycobot_lidar.rviz`（含 LaserScan 显示）。

```bash
ros2 launch wheeltec_mycobot_bringup lidar.launch.py
ros2 launch wheeltec_mycobot_bringup lidar.launch.py use_rviz:=false
ros2 launch wheeltec_mycobot_bringup lidar.launch.py jsp_gui:=true
ros2 launch wheeltec_mycobot_bringup lidar.launch.py use_jsp:=false
```

**参数**

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `use_jsp` | `true` | 是否启动无头 joint_state_publisher（发布零位，保证模型完整） |
| `jsp_gui` | `false` | 是否启动 joint_state_publisher_gui 关节滑块界面（启动时替代无头版） |

### 3. `arm_controller.launch.py` — 机械臂控制器栈

在 `mycobot_280_real_moveit.launch.py` 基础上改用聚合 URDF，启动 myCobot 280 机械臂的 ros2_control + MoveIt2 运动规划。

- 启动 RSP（聚合 URDF）、`controller_manager`（加载 `mycobot280_moveit2_control/config/mycobot_280_real_controllers.yaml`）、`mycobot_driver_node.py`。
- 依次 spawn 控制器：`joint_state_broadcaster → arm_controller → gripper_action_controller`（使用 `TimerAction` + `RegisterEventHandler` 串联）。
- 默认启动无头 `joint_state_publisher`：JSB 只发布机械臂 6+夹爪关节，JSP 自动合并 JSB 的真实机械臂状态并为 4 个轮子关节补零位，保证 RViz 中整车完整显示（不覆盖真实关节值）。
- 可选启动 MoveIt2 `move_group`（复用 `mycobot_moveit_config/launch/move_group.launch.py`），并随其启动自带 RViz（携带 MotionPlanning 面板）。

```bash
ros2 launch wheeltec_mycobot_bringup arm_controller.launch.py
ros2 launch wheeltec_mycobot_bringup arm_controller.launch.py \
    use_rviz:=false driver_port:=/dev/ttyUSB0
```

**参数**

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否随 move_group 启动 RViz2 |
| `use_move_group` | `true` | 是否启动 MoveIt2 move_group 节点 |
| `driver_port` | `/dev/ttyACM0` | 机械臂串口设备路径 |
| `driver_baud` | `115200` | 机械臂串口波特率 |
| `use_sim_time` | `false` | 是否使用仿真时钟（真实硬件为 false） |
| `use_jsp` | `true` | 是否启动无头 joint_state_publisher（为轮子关节补零位，机械臂状态仍来自 JSB） |

### 4. `car_controller.launch.py` — 底盘控制器栈

在 `turn_on_wheeltec_robot.launch.py` 基础上改用聚合 URDF，启动 WHEELTEC R680 底盘的串口驱动与 EKF 状态估计。

- 启动 RSP（聚合 URDF，已内嵌 `base_footprint → base_link`，**不再需要** `base_to_link` 静态 TF）。
- 复用 `turn_on_wheeltec_robot/launch/base_serial.launch.py`（`akmcar:=false`）。
- 启动 `base_to_gyro`（`base_link → gyro_link`，单位变换）与 `base_to_laser`（`base_footprint → laser`，`xyz='0.0911 0 0.155'`）静态 TF。
- 启动 EKF 节点（`turn_on_wheeltec_robot/config/ekf.yaml`，`carto_slam:=true` 时禁用）。
- 启动 `joint_state_publisher`（无头模式，真实硬件提供关节状态）。
- RViz 配置：`rviz/wheeltec_mycobot_car_controller.rviz`（含 Odometry 显示）。

```bash
ros2 launch wheeltec_mycobot_bringup car_controller.launch.py
ros2 launch wheeltec_mycobot_bringup car_controller.launch.py \
    use_rviz:=false carto_slam:=true
```

**参数**

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `carto_slam` | `false` | 是否使用 Cartographer SLAM（为 true 时禁用 EKF） |

### 5. `all.launch.py` — 全功能 all-in-one

为避免重复 RSP，该 launch 文件**内联**所有组件，不通过子 launch 引入相机/激光/机械臂/底盘 launch。一次启动底盘 + 相机 + 激光 + EKF + 机械臂 ros2_control + MoveIt2 + RViz。

- 单一 RSP 发布聚合机器人描述。
- 底盘串口驱动（`base_serial.launch.py`，`akmcar:=false`）。
- Astra Pro 相机驱动（`astra_pro.launch.xml`）。
- LSLidar N10 驱动（`lsn10_launch.py`）。
- `base_to_gyro` 与 `base_to_laser` 静态 TF。
- EKF 节点（`turn_on_wheeltec_robot/config/ekf.yaml`）。
- `controller_manager`（加载 `mycobot_280_real_controllers.yaml`）。
- `mycobot_driver_node.py`（默认 `/dev/ttyACM0`，`115200`）。
- 依次 spawn `joint_state_broadcaster → arm_controller → gripper_action_controller`。
- `joint_state_publisher`（无头模式，发布轮子等非 ros2_control 关节状态；机械臂关节由 `joint_state_broadcaster` 发布）。
- 可选启动 MoveIt2 `move_group`（向其传 `use_rviz:=false`，因为本包自己启动 RViz）。
- RViz 配置：`rviz/wheeltec_mycobot_all.rviz`（含 Image + LaserScan + Odometry 显示）。

```bash
ros2 launch wheeltec_mycobot_bringup all.launch.py
ros2 launch wheeltec_mycobot_bringup all.launch.py \
    use_rviz:=false use_move_group:=false driver_port:=/dev/ttyUSB0
```

**参数**

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `use_move_group` | `true` | 是否启动 MoveIt2 move_group 节点 |
| `driver_port` | `/dev/ttyACM0` | 机械臂串口设备路径 |
| `driver_baud` | `115200` | 机械臂串口波特率 |

## 硬件连接与预期报错说明

启动底盘（`car_controller`、`all`）需要 WHEELTEC 底盘串口设备 `/dev/wheeltec_controller`；启动机械臂（`arm_controller`、`all`）需要 myCobot 串口设备（默认 `/dev/ttyACM0`）；启动相机需要 Astra Pro USB 连接；启动激光雷达需要 LSLidar N10 连接。

**在硬件设备未连接的情况下，对应节点启动后会出现串口/USB 打开失败或设备未找到等报错，属正常现象**。可视化类 launch（`camera`、`lidar`）不依赖底盘与机械臂硬件，可在无底盘/机械臂时使用（但相机/雷达本身仍需连接才能获取数据）。

机械臂串口路径可通过 `driver_port` 参数覆盖，例如：

```bash
ros2 launch wheeltec_mycobot_bringup arm_controller.launch.py \
    driver_port:=/dev/ttyUSB0
```

## 不修改其它包的说明

本包严格遵守“只新增、不改动”原则：

- 聚合机器人描述（URDF/xacro/网格）全部来自 `wheeltec_mycobot_description`，本包不复制也不修改。
- 底盘驱动、EKF 配置直接复用 `turn_on_wheeltec_robot`。
- 相机、激光驱动直接复用 `astra_camera`、`lslidar_driver`。
- 机械臂硬件接口、Python 驱动、控制器配置直接复用 `mycobot280_moveit2_control`。
- MoveIt2 配置与 `move_group.launch.py` 直接复用 `mycobot_moveit_config`。

如需调整机器人模型、传感器标定、控制器参数等，请到对应原包修改，本包不承担这些职责。
