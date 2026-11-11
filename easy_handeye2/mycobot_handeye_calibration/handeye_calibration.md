# mycobot280 M5 手眼标定使用说明

本包基于 `easy_handeye2` + `aruco_ros`，对 mycobot280 M5 进行手眼标定。
当前硬件为**相机固定、ArUco 标定板装在法兰上**，对应**眼在手外（eye-to-hand / eye_on_base）**。

---

## 1. 当前配置一览

| 项目 | 值 |
|------|-----|
| 标定类型 | 眼在手外 `eye_on_base` |
| 机器人模型 | **聚合模型**：`wheeltec_mycobot_description` 的 WHEELTEC R680 底盘 + myCobot 280 |
| ArUco 标定板 | ID **2**，黑框边长 **0.026 m**，Original ArUco 字典 |
| 标定板安装 | 法兰 `link6_flange` 的 **-Z** 方向（`marker_thickness` 通常取负值，见第 8 节） |
| 机械臂基坐标系 | **`link1`** |
| 末端坐标系 | `link6_flange` |
| 相机光学坐标系 | `camera_color_optical_frame`（Astra Pro 彩色流） |
| 机械臂串口 | `/dev/mycobot_controller`，115200 |
| 标定结果 | **`link1 -> camera_color_optical_frame`** |
| 工作空间 | `/home/admi/wheeltec_mycobot_ws` |

标定结果直接以 `link1` 为父帧发布，使用方无需再做 `base_link` 转换。

---

## 2. 环境准备

`easy_handeye2`、`aruco_ros` 与本项目包都在同一个工作空间内，只需 source 一次：

```bash
source /opt/ros/jazzy/setup.bash
source /home/admi/wheeltec_mycobot_ws/install/setup.bash
```

首次编译（或修改 C++ 后）：

```bash
cd /home/admi/wheeltec_mycobot_ws
colcon build --packages-select mycobot_handeye_calibration --symlink-install
```

> 只改 `config/calibration_params.yaml` 时**不需要重新编译**：`--symlink-install` 下 install 里的 yaml 是指向源码的软链，下次启动即生效。

检查依赖是否齐全：

```bash
ros2 pkg list | grep -E "easy_handeye2|aruco_ros|astra_camera"
```

---

## 3. 启动标定（一条命令）

```bash
ros2 launch mycobot_handeye_calibration calibrate_eye_to_hand.launch.py
```

该 launch 是**一站式**启动，会自动拉起：

| 子系统 | 节点 | 作用 |
|--------|------|------|
| 机械臂 bringup | `wheeltec_mycobot_bringup/arm_controller.launch.py` | 使用**聚合机器人模型**（WHEELTEC R680 底盘 + myCobot 280，来自 `wheeltec_mycobot_description`） |
| 机器人模型 | `robot_state_publisher` | 发布 `base_footprint -> base_link -> link1 -> … -> link6_flange` |
| ros2_control | `controller_manager` + `mycobot_driver_node.py` | 通过 `/dev/mycobot_controller` 驱动机器人 |
| 控制器 | `joint_state_broadcaster` / `arm_controller` / `gripper_action_controller` | 接收轨迹并下发真机 |
| MoveIt2 | `move_group` + `RViz` | 运动规划与可视化（`use_rviz:=true`），同样加载聚合模型 |
| Astra Pro | `astra_camera` | 发布 `/camera/color/image_raw`、`/camera/color/camera_info` 及 `camera_link -> camera_color_optical_frame` TF |
| 关节滑块 GUI | `joint_slider_gui.py` | 拖动滑块直接控制机械臂（`use_joint_gui:=true`） |
| ArUco | `aruco_ros/single` | 检测 ID 2，发布 `camera_color_optical_frame -> aruco_marker_frame` |
| 补偿 TF | `static_transform_publisher` | 发布 `aruco_marker_frame -> marker_mount_frame`（板厚/偏心补偿） |
| 标定 | `easy_handeye2/handeye_server` + `rqt_calibrator.py` | 采样、求解、保存 |

启动后会弹出 **RViz**、**关节滑块 GUI**、**easy_handeye2 Calibration tool** 三个窗口。

---

## 4. 移动机械臂（两种方式）

标定时需要把机械臂摆到多个不同位姿，本包提供两种拖拽方式，可同时使用：

### 方式 A：关节滑块 GUI（默认启动）

- 6 个滑块分别对应 `link1_to_link2` … `link6_to_link6_flange`，范围约 ±2.8 rad。
- **拖动滑块松手即下发**目标到 `/arm_controller/joint_trajectory`（不经过 MoveIt 规划，2 秒到位）。
- 顶部实时显示当前关节角；按钮：`发送当前目标`、`回零`、`获取当前位姿为目标`。
- 不需要时用 `use_joint_gui:=false` 关闭。

### 方式 B：RViz 6D 拖动手柄

- RViz 左侧 `MotionPlanning` 面板中，末端会有红绿蓝箭头 + 彩色圆环的 **InteractiveMarker**。
- 拖动箭头/圆环设置目标位姿 → 点 `Plan & Execute` 让真机运动。
- 该方式经 MoveIt 规划，会自动避开关节限位，适合大范围移动。

> 移动过程中 `Take Sample` 按钮会变灰；等机械臂和 marker 都静止后按钮才会重新可用，这是正常行为。

---

## 5. 采集样本并求解

在 **easy_handeye2 Calibration tool** 窗口中操作：

1. 先确认 ArUco 能稳定检测到（见 5.1 的验证命令）。
2. 用第 4 节任一方式把机械臂移到新位姿，**停稳**。
3. 点击 **`Take Sample`**：采样并**自动重新计算**当前标定（本分支没有单独的 Compute 按钮）。
4. 重复 2~3 步，至少 **10~15 组**；位姿之间要有明显的旋转和平移变化（只平移或只旋转会降低精度）。
5. 观察 `Translation` / `Rotation` 输出，确认数值稳定且合理后，点击 **`Save`** 保存。
   - 采样点多余或异常时，选中列表项点 `Remove Sample` 删除后会自动重算。

结果文件：

```
~/.ros2/easy_handeye2/calibrations/mycobot280_calibration.yaml
~/.ros2/easy_handeye2/samples/mycobot280_calibration.samples
```

### 5.1 启动后自检

另开终端（已 source）：

```bash
# 相机话题
ros2 topic list | grep camera/color

# ArUco 是否检测到（有输出即检测到）
ros2 topic echo /aruco_single/pose --once

# 关键 TF（聚合模型：base_footprint -> base_link -> link1 -> link6_flange）
ros2 run tf2_ros tf2_echo base_link link1
ros2 run tf2_ros tf2_echo link1 link6_flange
ros2 run tf2_ros tf2_echo link1 camera_color_optical_frame
ros2 run tf2_ros tf2_echo camera_color_optical_frame aruco_marker_frame
```

最后一条只有在相机看得到标定板时才有数值。

---

## 6. 发布标定结果并验证

标定完成后 **Ctrl-C 关闭标定 launch**，再启动结果发布节点：

```bash
ros2 launch mycobot_handeye_calibration publish.launch.py
```

它读取 `~/.ros2/easy_handeye2/calibrations/mycobot280_calibration.yaml`，以静态 TF 形式发布
**`link1 -> camera_color_optical_frame`**（实测值，用于修正 URDF 中的名义安装位置）。

另开终端验证：

```bash
ros2 run tf2_ros tf2_echo link1 camera_color_optical_frame
ros2 run tf2_tools view_frames      # 生成 frames.pdf 查看整棵 TF 树
```

`publish.launch.py` 的 `name` 必须与标定时一致（默认 `mycobot280_calibration`），否则找不到结果文件。

---

## 7. 参数说明

所有默认值来自 [config/calibration_params.yaml](config/calibration_params.yaml) 的 `mycobot280` 段，
launch 启动时用 `yaml.safe_load` 读取；命令行参数仍可覆盖单个值。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `marker_id` | `2` | ArUco 标记 ID |
| `marker_size` | `0.026` | 标记**黑框**边长（米，不含白边） |
| `robot_base_frame` | `link1` | 机械臂基坐标系（标定结果的父帧） |
| `robot_effector_frame` | `link6_flange` | 末端法兰坐标系 |
| `camera_optical_frame` | `camera_color_optical_frame` | ArUco marker TF 的父帧 |
| `image_topic` | `/camera/color/image_raw` | 图像话题（由 `camera_namespace` + `image_topic` 拼接） |
| `camera_info_topic` | `/camera/color/camera_info` | 相机内参话题 |
| `marker_frame` | `aruco_marker_frame` | ArUco 发布的 marker 子帧名 |
| `name` | `mycobot280_calibration` | 标定名（结果文件名） |
| `use_rviz` | `true` | 是否启动 RViz |
| `driver_port` | `/dev/mycobot_controller` | 机械臂串口 |
| `driver_baud` | `115200` | 串口波特率 |
| `marker_offset_x/y` | `0.0 / 0.0` | 标定板中心相对法兰中心的 X/Y 偏移（米） |
| `marker_thickness` | `0.0` | 标定板表面到法兰端面的距离（**允许负数**） |
| `use_joint_gui` | `true` | 是否启动关节滑块 GUI |

覆盖示例：

```bash
# 换一块更大的标定板，并调整标定板相对法兰的厚度补偿
ros2 launch mycobot_handeye_calibration calibrate_eye_to_hand.launch.py \
    marker_size:=0.05 marker_thickness:=-0.026
```

> 相机与机械臂之间的名义变换由聚合 URDF 提供
> （`base_link -> camera_link` + Astra 驱动的 `camera_link -> camera_color_optical_frame`），
> 标定前 RViz 中看到的就是模型位置；标定完成后由 `publish.launch.py`
> 以实测值覆盖。

---

## 8. marker_thickness 的符号（允许负数）

**can be negative**：代码没有任何范围校验。launch 会发布静态 TF
`aruco_marker_frame -> marker_mount_frame`，平移为：

```
T = (-marker_offset_x, -marker_offset_y, -marker_thickness)
```

这个平移是在 **aruco marker 坐标系**下表达的，而 marker 坐标系的 Z 轴由 aruco_ros 估计，
**始终指向 marker 外表面法向（朝相机）**，并不跟随法兰。

因此符号判据是：

| flange 原点相对 marker 坐标系 | `marker_thickness` |
|------|------|
| 在 marker 的 **-Z** 侧 | 正 |
| 在 marker 的 **+Z** 侧 | **负** |

**实测确定方法**（检测到 marker 后）：

```bash
ros2 run tf2_ros tf2_echo aruco_marker_frame link6_flange
```

读取输出的 `Translation.z`，则

```
marker_thickness = -z
```

例如输出 `z = 0.026` → 填 `marker_thickness: -0.026`。
标定板装在法兰 -Z 方向时，通常就是需要填负值的情况。

---

## 9. 眼在手内标定（Eye-in-Hand，可选）

仅在把相机改装到法兰上时使用。**配置**：相机固定在 `link6_flange`，ArUco 标定板固定在世界中。

```bash
ros2 launch mycobot_handeye_calibration calibrate_eye_in_hand.launch.py
```

- 该 launch 同样是一站式启动，静态 TF 改为 `link6_flange -> camera_link`（名义值）。
- 采样、求解、保存流程与第 5、6 节完全相同。
- 求解结果为 `link6_flange -> camera_color_optical_frame`，验证命令：

```bash
ros2 launch mycobot_handeye_calibration publish.launch.py
ros2 run tf2_ros tf2_echo link6_flange camera_color_optical_frame
```

- 注意：`calibration_params.yaml` 里的 marker/相机偏置只用于眼在手外；眼在手内请用 CLI 覆盖需要的参数。

---

## 10. 常见问题

| 现象 | 原因 / 解决 |
|------|-------------|
| `aruco_marker_frame` TF 不出现 | 相机内参未加载；ID 不是 2；`marker_size` 与实际不符；光照不足或相机看不到标定板 |
| `handeye_server` 一直 "Waiting for TF initialization" | 缺 `link1 -> link6_flange`（驱动/ros2_control 未起）、缺 `base_link -> camera_link`（聚合 URDF 未发布，检查 robot_state_publisher）或 `camera_color_optical_frame -> aruco_marker_frame`（ArUco 未检测到） |
| `Take Sample` 一直灰着 | 机械臂或 marker 仍在运动；等两者静止后按钮自动恢复 |
| 提示 "Too few samples" | 至少需要 3 组（`>2`），建议 10~15 组 |
| 标定平移量明显错误 | 相机内参不准；标定板未刚性固定；采样的位姿差异过小 |
| RViz 中相机点云与实物错位 | 未运行 `publish.launch.py`，或聚合 URDF 中 `camera_link` 的实际安装位姿需要修正 |
| 拖动滑块机械臂不动 | 检查 `arm_controller` 是否 active（`ros2 control list_controllers`） |
| 串口打不开 | 确认设备存在：`ls -l /dev/mycobot_controller`；权限不足时把用户加入 `dialout` 组 |
| rqt 窗口没出现 | 运行 `rqt`，手动加载 `easy_handeye2 Calibration tool` 插件 |

---

## 11. 结果文件

```
~/.ros2/easy_handeye2/calibrations/mycobot280_calibration.yaml   # 标定结果
~/.ros2/easy_handeye2/samples/mycobot280_calibration.samples     # 采样数据
```

结果 yaml 内含 `transformation`（四元数 + 平移）及 `parameters`（各坐标系名），
可被 `handeye_publisher` 加载发布为 TF，也可被其他程序读取后写入 URDF。

---

## 12. 包结构

```
mycobot_handeye_calibration/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── calibration_params.yaml         # 标定默认参数（launch 直接读取）
├── launch/
│   ├── calibrate_eye_to_hand.launch.py # 眼在手外标定入口
│   ├── calibrate_eye_in_hand.launch.py # 眼在手内标定入口
│   └── publish.launch.py               # 发布标定结果 TF
├── scripts/
│   └── joint_slider_gui.py             # 关节滑块 GUI（拖动手柄控制机械臂）
└── handeye_calibration.md              # 本文档
```

---

## 13. 快速命令速查

```bash
# 0) 每次开终端
source /opt/ros/jazzy/setup.bash
source /home/admi/wheeltec_mycobot_ws/install/setup.bash

# 1) 启动眼在手外标定（一站式：机械臂 + MoveIt/RViz + 相机 + ArUco + rqt + 滑块 GUI）
ros2 launch mycobot_handeye_calibration calibrate_eye_to_hand.launch.py

# 2) 用关节滑块 GUI 或 RViz 6D 手柄移动机械臂 -> 停稳 -> Take Sample
#    重复 10~15 次 -> 数值合理后点 Save

# 3) Ctrl-C 关闭标定 launch，发布结果
ros2 launch mycobot_handeye_calibration publish.launch.py

# 4) 验证
ros2 run tf2_ros tf2_echo link1 camera_color_optical_frame
```

常用覆盖参数：

```bash
marker_size:=0.026          # 标定板黑框边长
marker_thickness:=-0.026    # 标定板相对法兰的 Z 偏移（允许负值）
robot_base_frame:=link1     # 基坐标系
driver_port:=/dev/mycobot_controller
use_joint_gui:=false        # 不需要滑块 GUI 时关闭
use_rviz:=false             # 无显示器/不需要 RViz 时关闭
```
