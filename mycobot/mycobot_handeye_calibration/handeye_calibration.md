#  手眼标定使用流程：
## 1) 
source ~/mycobot/easy_handeye2_ws/install/setup.bash 
source ~/mycobot/mycobot_ros2-main/install/setup.bash
ros2 launch mycobot_handeye_calibration calibrate_eye_to_hand.launch.py

注：识别aruco板及计算手眼标定的包在easy_handeye2_ws中,所以要先导入此环境easy_handeye2_ws

## 2) 启动发布节点
先关闭calibrate_eye_to_hand.launch.py节点
source ~/mycobot/easy_handeye2_ws/install/setup.bash
source ~/mycobot/mycobot_ros2-main/install/setup.bash
ros2 launch mycobot_handeye_calibration publish.launch.py

## 3) 验证（另开终端）
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_tools view_frames   # 生成 frames.pdf 查看 TF 树

## 4 注意
- handeye_publisher 用 StaticTransformBroadcaster ，TF 是 latched 的——节点启动一次后即使关闭，TF 在 DDS 缓存中仍可见一段时间，但 不建议依赖 这个行为，应让 publish.launch.py 持续运行。
- 若要修改相机名义安装位姿让 RViz 显示更接近实际，标定前调 camera_x/y/z/roll/pitch/yaw ；标定后这些参数不再起作用，以 publish.launch.py 发布的实测值为准。
- publish.launch.py 的 name 参数必须与标定时使用的 name （默认 mycobot280_calibration ）一致，否则找不到 yaml 文件。


# mycobot280 M5 手眼标定使用说明

本包基于 `easy_handeye2` + `aruco_ros`，对 mycobot280 M5 机械臂进行手眼标定。
标定板（ArUco，**ID 582**）已安装在机械臂法兰（`link6_flange`）上，因此**眼在手外**（eye-to-hand）
可直接使用；**眼在手内**（eye-in-hand）则需要将相机改装到法兰上（见下文）。

---

## 1. 关键坐标系

| 用途 | 坐标系名称 | 来源 |
|------|-----------|------|
| 机械臂基座 | `base_link` | `mycobot_280.urdf.xacro` |
| 末端法兰 | `link6_flange` | `mycobot_280.urdf.xacro`（`flange_link` 参数默认值） |
| MoveIt 规划组 | `arm` | `mycobot_280.srdf` |
| Astra Pro 相机外壳 | `camera_link` | `astra_pro.launch.xml`（`camera_name:=camera` 时） |
| Astra Pro 彩色光学坐标系 | `camera_color_optical_frame` | `astra_pro.launch.xml`（astra_camera 驱动发布） |
| Astra Pro 深度光学坐标系 | `camera_depth_optical_frame` | `astra_pro.launch.xml`（astra_camera 驱动发布） |
| ArUco 标定板 ID | `582` | 用户指定 |

> 注意：Astra Pro 的光学帧名为 `camera_color_optical_frame`，**不是** mycobot_description URDF 中 D435 的 `camera_head_color_optical_frame`。Astra 自带的 TF 链 `camera_link → camera_depth_frame → camera_color_optical_frame` 与机械臂 URDF 是**断开的**，由标定 launch 文件额外发布一个静态 TF（`base_link → camera_link` 或 `link6_flange → camera_link`）把它接到机械臂上。

---

## 2. 依赖安装

`easy_handeye2` 和 `aruco_ros` 不在 ROS2 二进制源中，需自行克隆到工作空间编译。

```bash
# 进入你的 ROS2 工作空间 src 目录（任选其一）
cd ~/mycobot/colcon_ws/src   # 或 ~/trac_ik_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2
下载地址：https://github.com/pal-robotics/aruco_ros/tree/humble-devel

# 编译
cd ..
colcon build --packages-select easy_handeye2 aruco_ros

# 刷新环境
source install/setup.bash
```

> 若 `humble` 分支不可用，请改用 `ros2` 或 `master` 分支。本包仅依赖其 launch 入口，不依赖具体版本接口。

验证安装：

```bash
ros2 pkg list | grep -E "easy_handeye2|aruco_ros"
```

---

## 3. 标定板准备（ArUco ID 582）

1. 生成 ArUco 标记 ID 582：
   - 访问 https://chev.me/arucogen/ ，选择 **Dictionary: Original ArUco**，**Marker ID: 582**，下载并打印。
   - **Original ArUco** 对应 `aruco_marker_dict=0`（已在 launch 中默认设置）。
2. 测量标记**黑色方框边长**（不含白边），单位：米。本包默认 `marker_size:=0.035`（3.5cm）。
   若你的打印尺寸不同，启动标定时通过参数覆盖：
   ```bash
   ros2 launch mycobot_handeye_calibration calibrate_eye_to_hand.launch.py marker_size:=0.05
   ```
3. 将标记**刚性固定**到机械臂法兰上，标记平面与法兰轴线尽量垂直。
   - 眼在手外：标记贴在法兰端面（已由用户完成）。
   - 眼在手内：标记固定在工作台前方的世界中（不动）。

---

## 4. 前置：相机内参标定

手眼标定**必须**先有准确的相机内参。使用 `camera_calibration` 包：

```bash
# 启动 Astra Pro 相机
ros2 launch astra_camera astra_pro.launch.xml

# 用棋盘格标定内参（假设 8x6 内角点，每格 0.025m）
ros2 run camera_calibration cameracalibrator \
    --size 8x6 --square 0.025 \
    --ros-args -r image:=/camera/color/image_raw -r camera:=/camera/color
```

标定完成后将结果写入相机的 `camera_info`。Astra Pro 默认从
`astra_camera/config/camera_info_color.yaml` 加载内参，把标定结果写入这个文件即可。
**内参不准会导致手眼标定完全失效。**

---

## 5. 眼在手外标定（Eye-to-Hand，推荐，对应你的当前硬件）

**配置**：Astra Pro 相机固定在世界中，ArUco ID 582 贴在法兰 `link6_flange` 上。
**求解**：`base_link -> camera_color_optical_frame`。

### 5.1 一条命令启动（推荐）

`calibrate_eye_to_hand.launch.py` 是**一站式** launch：它会在同一个 launch 里同时拉起
真机机械臂 + MoveIt + Astra Pro 相机 + ArUco + easy_handeye2，**只需运行一条命令**：

```bash
# 一次性 source 两个工作空间（mycobot_ros2-main 和 easy_handeye2_ws）
source ~/mycobot/easy_handeye2_ws/install/setup.bash
source ~/mycobot/mycobot_ros2-main/install/setup.bash

# 启动眼在手外标定（含机械臂、MoveIt、相机、ArUco、rqt GUI、RViz）
ros2 launch mycobot_handeye_calibration calibrate_eye_to_hand.launch.py
```

启动后会拉起：

| 子系统 | 节点 | 作用 |
|--------|------|------|
| 机械臂 bringup | `robot_state_publisher` | 发布 URDF 与 `base_link -> link6_flange`（关节编码器实时驱动，**真实准确**） |
| ros2_control | `controller_manager` + `mycobot_driver_node.py` | 加载 `MyCobot280HardwareInterface` 硬件插件，串口驱动真机 |
| 控制器 | `joint_state_broadcaster`, `arm_controller`, `gripper_action_controller` | 接收 MoveIt 的 `follow_joint_trajectory` action 并下发给机械臂 |
| MoveIt2 | `move_group` + `RViz` | 提供运动规划服务，rqt_calibrator 通过它自动到达采样位姿 |
| Astra Pro | `astra_camera_node` | 发布 `/camera/color/image_raw`、`/camera/color/camera_info`，以及 TF `camera_link -> camera_color_optical_frame` |
| 静态 TF | `static_transform_publisher` | 发布 `base_link -> camera_link`，把 Astra 接到机械臂基座（**名义值**，标定结果会覆盖它） |
| ArUco | `aruco_ros/single` | 检测 ID 582，发布 TF `camera_color_optical_frame -> aruco_marker_frame` |
| 标定 | `easy_handeye2/handeye_server` + `rqt_calibrator.py` | 采样、求解、保存 |

> 静态 TF 的作用：Astra 驱动发布的 TF 链是 `camera_link → camera_depth_frame → camera_color_optical_frame`，**与机械臂 URDF 没有连接**。若不补这条静态 TF，`base_link → camera_color_optical_frame` 这条链在 `/tf` 中不存在，easy_handeye2 会一直卡在 "Waiting for TF initialization"。这条静态 TF 给出的是相机在 `base_link` 下的**名义安装位姿**（默认 `camera_x/y/z/roll/pitch/yaw`，可通过 CLI 覆盖）。手眼标定的目的就是用**实测值**替换这条名义值——标定完成后用 `publish.launch.py` 发布的 `base_link → camera_color_optical_frame` 会覆盖这条静态 TF。

> 内部 `calibration_type` 设为 `eye_on_base`（easy_handeye2 中"眼在手外"的官方名称）。

### 5.2 修改默认相机安装位姿（可选）

如果你的 Astra Pro 物理安装位置与默认值差距较大，可通过 CLI 覆盖 `camera_x/y/z/roll/pitch/yaw`
（单位：米 / 弧度）。这只影响标定前的名义 TF，不影响最终标定结果：

```bash
ros2 launch mycobot_handeye_calibration calibrate_eye_to_hand.launch.py \
    camera_x:=0.25 camera_y:=-0.20 camera_z:=0.45 camera_pitch:=0.35
```

### 5.3 验证启动成功

启动后另开终端检查（假设已 source）：

```bash
# 1) 相机话题
ros2 topic list | grep camera/color
#   应看到 /camera/color/image_raw 和 /camera/color/camera_info

# 2) 关键 TF 链
ros2 run tf2_ros tf2_echo base_link link6_flange          # 机械臂关节（实时）
ros2 run tf2_ros tf2_echo base_link camera_link           # 静态 TF（名义）
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame  # 完整链
ros2 run tf2_ros tf2_echo camera_color_optical_frame aruco_marker_frame  # ArUco 检测
```

最后一条只有当相机能看到 ID 582 标记时才有输出。

### 5.4 采样与求解

1. 在 RViz 中确认相机能看到标记且 `aruco_marker_frame` TF 稳定跳动。
2. 通过 easy_handeye2 的 rqt 界面（`Check pose`）让 MoveIt 规划到多个不同位姿。
   每到位一次点击 `Take sample` 采集一组（机器人位姿 + 标记位姿）。
3. 至少采集 **10~15 组**，位姿之间要有明显的旋转/平移变化（不同姿态差异越大标定越准）。
4. 点击 `Compute` 求解，结果会保存到 `~/.ros2/easy_handeye2/calibrations/mycobot280_calibration.yaml`。

### 5.5 发布结果并验证

标定完成后，关闭标定 launch（Ctrl-C），然后启动结果发布节点：

```bash
# 启动结果发布节点（发布 base_link -> camera_color_optical_frame 的 TF）
ros2 launch mycobot_handeye_calibration publish.launch.py
```

它会读取 `~/.ros2/easy_handeye2/calibrations/mycobot280_calibration.yaml` 并发布
标定后的 `base_link -> camera_color_optical_frame` TF，**覆盖**标定 launch 发布的静态 TF。

另开终端验证：

```bash
# 查看完整 TF 树
ros2 run tf2_tools view_frames

# 查看标定结果（应与相机实际安装位置一致）
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

`frames.pdf` 中应出现 `base_link -> camera_link -> camera_color_optical_frame` 的链路，
其中 `base_link -> camera_link` 用的是**实测值**（标定结果）。

---

## 6. 眼在手内标定（Eye-in-Hand）

**配置**：Astra Pro 相机刚性安装在法兰 `link6_flange` 上，ArUco ID 582 固定在世界中（不动）。
**求解**：`link6_flange -> camera_color_optical_frame`。

> 注意：眼在手内需要把 Astra Pro **物理固定到法兰上**（例如用法兰夹具把相机绑到 `link6_flange`）。
> URDF 不需要修改——标定 launch 会自动发布 `link6_flange -> camera_link` 的静态 TF（名义值），
> 标定完成后用 `publish.launch.py` 用实测值覆盖。

### 6.1 一条命令启动

```bash
source ~/mycobot/easy_handeye2_ws/install/setup.bash
source ~/mycobot/mycobot_ros2-main/install/setup.bash

ros2 launch mycobot_handeye_calibration calibrate_eye_in_hand.launch.py
```

与眼在手外一样，此 launch 是**一站式**启动：同时拉起真机机械臂 + ros2_control + MoveIt + RViz
+ Astra Pro 相机 + ArUco + easy_handeye2。内部 `calibration_type` 设为 `eye_in_hand`，
静态 TF 改为发布 `link6_flange -> camera_link`（相机装在法兰上的名义位姿）。

### 6.2 修改默认相机法兰位姿（可选）

```bash
ros2 launch mycobot_handeye_calibration calibrate_eye_in_hand.launch.py \
    camera_z:=0.05 camera_pitch:=0.5
```

### 6.3 采样、求解、发布

步骤与第 5.4、5.5 节一致，只是求解出的变换是 `link6_flange -> camera_color_optical_frame`：

```bash
ros2 launch mycobot_handeye_calibration publish.launch.py
ros2 run tf2_ros tf2_echo link6_flange camera_color_optical_frame
```

---

## 7. 参数说明

`config/calibration_params.yaml` 记录了所有默认值，可通过 CLI 参数覆盖（实际可用参数以 `ros2 launch ... --show-args` 为准）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `marker_id` | `582` | ArUco 标记 ID |
| `marker_size` | `0.035` | 标记黑色方框边长（米） |
| `robot_base_frame` | `base_link` | 机械臂基坐标系 |
| `robot_effector_frame` | `link6_flange` | 末端法兰坐标系 |
| `camera_optical_frame` | `camera_color_optical_frame` | Astra Pro 彩色光学坐标系（ArUco marker TF 的父帧） |
| `image_topic` | `/camera/color/image_raw` | 图像话题（绝对路径） |
| `camera_info_topic` | `/camera/color/camera_info` | 相机内参话题（绝对路径） |
| `marker_frame` | `aruco_marker_frame` | ArUco 发布的 marker TF 子帧名 |
| `name` | `mycobot280_calibration` | 标定名（结果 yaml 文件名） |
| `use_rviz` | `true` | 是否启动 RViz（通过 MoveIt launch） |
| `driver_port` | `/dev/ttyACM0` | mycobot 串口设备路径 |
| `driver_baud` | `115200` | mycobot 串口波特率 |
| `camera_x/y/z` | `0.22/-0.30/0.50`（眼在手外） | 相机的名义安装位置（米，眼在手外相对 base_link，眼在手内相对 link6_flange） |
| `camera_roll/pitch/yaw` | `0/0.436/1.5708`（眼在手外） | 相机的名义安装姿态（弧度） |

> MoveIt 的运动规划通过 rqt_calibrator GUI 调用 `/easy_handeye2/calibration/...` 服务完成，规划组名由 MoveIt 自身的 SRDF 决定（`arm`），不在本 launch 中作为参数传递。

示例：覆盖标记尺寸与相机光学帧

```bash
ros2 launch mycobot_handeye_calibration calibrate_eye_to_hand.launch.py \
    marker_size:=0.05 \
    camera_x:=0.25 camera_pitch:=0.35
```

---

## 8. 常见问题

| 现象 | 原因 / 解决 |
|------|-------------|
| `aruco_marker_frame` TF 不出现 | 相机内参未加载；标记 ID 不对；标记尺寸与 `marker_size` 不符；光照不足；相机看不到标记 |
| `easy_handeye2` 找不到 | 未 `source` 工作空间的 `install/setup.bash`（需 source easy_handeye2_ws） |
| `handeye_server` 一直 "Waiting for TF initialization" | `/tf` 中缺 `base_link -> link6_flange`（ros2_control 或 driver 未启动）、缺 `base_link -> camera_link`（静态 TF 节点未起来）或缺 `camera_color_optical_frame -> aruco_marker_frame`（ArUco 没检测到标记） |
| `move_group` 服务调用失败 | MoveIt 未启动或规划组名不是 `arm`（见 mycobot_280.srdf） |
| 标定结果平移量明显错误 | 内参不准；标记未刚性固定；采样位姿过少或差异过小 |
| RViz 中相机点云与实物错位 | 未启动 `publish.launch.py`，或 `base_link -> camera_link` 静态 TF 的名义值与实际差距过大（先用 `camera_x/y/z/...` 修正） |
| rqt GUI 没出现 | 运行 `rqt`，手动加载 `Handeye Calibration` 插件 |

---

## 9. 结果文件位置

标定完成后，结果保存在：

```
~/.ros2/easy_handeye2/calibrations/mycobot280_calibration.yaml
```

采样过程数据保存在 `~/.ros2/easy_handeye2/samples/mycobot280_calibration.samples`。

文件内含 `transformation`（四元数 + 平移）等字段，可被 `easy_handeye2` 的 `handeye_publisher`
节点加载并发布为 TF（见 `publish.launch.py`），也可被其他程序读取后写入 URDF 的
`<joint origin="..."/>`。

---

## 10. 包结构

```
mycobot_handeye_calibration/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── calibration_params.yaml        # 默认参数（ID 582、TF 帧、话题）
├── launch/
│   ├── calibrate_eye_to_hand.launch.py # 眼在手外标定入口
│   ├── calibrate_eye_in_hand.launch.py # 眼在手内标定入口
│   └── publish.launch.py              # 发布标定结果 TF
└── camera_calibration.md              # 本文档
```

---

## 11. 快速命令速查

```bash
# 1) 安装 easy_handeye2 + aruco_ros（一次性，见第 2 节）
cd ~/mycobot/easy_handeye2_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2
# aruco_ros 从 https://github.com/pal-robotics/aruco_ros/tree/humble-devel 下载
cd .. && colcon build --packages-select easy_handeye2 aruco_ros
source install/setup.bash   # 以后每次开终端都要 source 这个工作空间

# 同时确保 mycobot_handeye_calibration + astra_camera 已编译
cd ~/mycobot/mycobot_ros2-main
colcon build --packages-select mycobot_handeye_calibration astra_camera astra_camera_msgs
source install/setup.bash

# 2) 启动眼在手外标定（终端1，一站式启动机械臂+MoveIt+相机+ArUco+easy_handeye2）
source ~/mycobot/easy_handeye2_ws/install/setup.bash
source ~/mycobot/mycobot_ros2-main/install/setup.bash
ros2 launch mycobot_handeye_calibration calibrate_eye_to_hand.launch.py
#   -> 在 rqt GUI 中采 10~15 组样本 -> Compute -> Save
#   -> Ctrl-C 关闭标定 launch

# 3) 发布结果（终端1）
ros2 launch mycobot_handeye_calibration publish.launch.py

# 4) 验证（终端2）
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

眼在手内把第 2 步换成：

```bash
ros2 launch mycobot_handeye_calibration calibrate_eye_in_hand.launch.py
```

并确保相机已刚性安装到 `link6_flange` 上。
