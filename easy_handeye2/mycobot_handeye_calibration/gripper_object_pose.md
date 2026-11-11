# 从相机目标位姿到 MoveIt 抓取目标位姿

> 适用场景：eye-to-hand 手眼标定已完成，`publish.launch.py` 正在发布 `base_link → camera_color_optical_frame`（
   这里应该是发布了base_link ->camera_link,因为相机发布了camera_link->camera_color->camera_color_optical_frame,所以这里发布的是base_link->camera_link,camera_color_optical_frame不能有两个父节点）
> 机械臂：mycobot_280，末端带 adaptive_gripper。

---

## 1. TF 链总览

```
base_link ─────────FK─────────▶ link6_flange ──fixed──▶ gripper_base
                                    │                    (xyz=0,0,0.034; rpy=1.579,0,0)
                                    │
  标定发布 ────────────────────────▶│                    视觉检测
  T_base_cam                        │                    T_cam_obj
                                    ▼
                          camera_color_optical_frame ───▶ object_frame
```

| 变换 | 含义 | 来源 |
|------|------|------|
| `T_base_cam` | `base_link → camera_color_optical_frame` | handeye_publisher（标定结果） |
| `T_cam_obj` | `camera_color_optical_frame → object_frame` | ArUco 检测 / YOLO+depth |
| `T_base_flange` | `base_link → link6_flange` | 机器人 FK（MoveIt `/tf`） |
| `T_flange_gripper` | `link6_flange → gripper_base` | URDF 固定常量（0, 0, 0.034; 1.579, 0, 0） |

---

## 2. 推导（3 步）

### 第 1 步：物体在基座下的位姿

```
T_base_obj = T_base_cam × T_cam_obj
```

物理意义：把相机看到的物体位置，搬到基座坐标系下表达。**直接左乘，不需要求逆。**

### 第 2 步：定义抓取几何常量 T_grasp

T_grasp = `gripper_base → object_frame`，表示"夹爪成功抓住物体时，两者的相对位姿"。这是**一次性标定的常量**，与当前物体位置无关。

示例：夹爪两指中点对准物体顶部中心，夹爪 Z 轴朝下指向物体：

```
T_grasp.translation = (0.0, 0.0, -0.05)   # 夹爪原点在物体上方 5mm
T_grasp.rotation    = (0, 0, 0, 1)        # 夹爪 Z 轴对准物体
```

### 第 3 步：计算 MoveIt 目标位姿

**推导**：抓取成功时 `T_gripper_obj = T_grasp`，而 `T_gripper_obj = T_gripper_base × T_base_obj`，所以：

```
T_gripper_base × T_base_obj = T_grasp
T_gripper_base = T_grasp × T_base_obj⁻¹
T_base_gripper_desired = T_base_obj × T_grasp⁻¹     ← 两边右乘 T_grasp⁻¹
```

最终一行公式：

```
T_base_gripper_desired = T_base_cam × T_cam_obj × T_grasp⁻¹
```

如果 MoveIt 规划组的末端是 `link6_flange`（而不是 `gripper_base`），则还需：

```
T_base_flange_desired = T_base_gripper_desired × T_flange_gripper⁻¹
```

其中 `T_flange_gripper` 是 URDF 固定常量 `xyz=(0,0,0.034), rpy=(1.579,0,0)`。

---

## 3. C++ 实现

```cpp
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// 全局成员
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// ===== 步骤 1: 获取 T_base_obj =====
geometry_msgs::msg::TransformStamped T_base_cam =
    tf_buffer_->lookupTransform(
        "base_link",                    // target（父）
        "camera_color_optical_frame",   // source（子）
        tf2::TimePointZero);

geometry_msgs::msg::TransformStamped T_cam_obj =
    tf_buffer_->lookupTransform(
        "camera_color_optical_frame",
        "object_frame",                 // 视觉检测发布的物体 frame
        tf2::TimePointZero);

tf2::Transform T_base_obj;
tf2::fromMsg(tf2::doTransform(T_cam_obj, T_base_cam), T_base_obj);
// 等价于 T_base_obj = T_base_cam * T_cam_obj

// ===== 步骤 2: 定义 T_grasp（一次性标定的常量）=====
tf2::Transform T_grasp;
T_grasp.setOrigin(tf2::Vector3(0.0, 0.0, -0.05));   // 夹爪在物体上方 5mm
T_grasp.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

// ===== 步骤 3: 计算夹爪目标位姿 =====
tf2::Transform T_base_gripper_desired =
    T_base_obj * T_grasp.inverse();

// ===== 步骤 4: 如果 MoveIt 规划组末端是 link6_flange =====
// URDF: link6_flange → gripper_base: xyz=(0,0,0.034), rpy=(1.579,0,0)
tf2::Transform T_flange_gripper;
T_flange_gripper.setOrigin(tf2::Vector3(0.0, 0.0, 0.034));
tf2::Quaternion q_fg;
q_fg.setRPY(1.579, 0.0, 0.0);
T_flange_gripper.setRotation(q_fg);

tf2::Transform T_base_flange_desired =
    T_base_gripper_desired * T_flange_gripper.inverse();

// ===== 步骤 5: 转成 Pose 发给 MoveIt =====
geometry_msgs::msg::Pose target_pose;
target_pose.position = tf2::toMsg(T_base_flange_desired.getOrigin());
target_pose.orientation = tf2::toMsg(T_base_flange_desired.getRotation());

// move_group_interface.setPoseTarget(target_pose);
// move_group_interface.move();
```

---

## 4. Python 实现

```python
import numpy as np
import tf2_ros
from geometry_msgs.msg import Pose
from tf2_ros import TransformException

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def transform_to_matrix(t):
    """geometry_msgs Transform -> 4x4 SE(3) matrix"""
    trans = t.transform.translation
    rot = t.transform.rotation
    q = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    q[:3, 3] = [trans.x, trans.y, trans.z]
    return q

# ===== 步骤 1 =====
T_base_cam = tf_buffer.lookup_transform(
    'base_link', 'camera_color_optical_frame',
    rclpy.time.Time()).transform

T_cam_obj = tf_buffer.lookup_transform(
    'camera_color_optical_frame', 'object_frame',
    rclpy.time.Time()).transform

M_base_cam = transform_to_matrix(T_base_cam)
M_cam_obj  = transform_to_matrix(T_cam_obj)
M_base_obj = M_base_cam @ M_cam_obj

# ===== 步骤 2: T_grasp =====
M_grasp = np.array([
    [1, 0, 0, 0.0],
    [0, 1, 0, 0.0],
    [0, 0, 1, -0.05],    # 夹爪在物体上方 5mm
    [0, 0, 0, 1]
])

# ===== 步骤 3 =====
M_base_gripper_desired = M_base_obj @ np.linalg.inv(M_grasp)

# ===== 步骤 4: 如果 MoveIt 用 link6_flange =====
M_flange_gripper = np.array([
    [0, 0, 1, 0.0],      # rpy=(1.579,0,0)
    [0, 1, 0, 0.034],    # z 偏移 34mm
    [-1,0, 0, 0.0],
    [0, 0, 0, 1]
])
M_base_flange_desired = M_base_gripper_desired @ np.linalg.inv(M_flange_gripper)

# ===== 步骤 5: 转成 Pose =====
from tf_transformations import quaternion_from_matrix
pose = Pose()
pose.position.x = M_base_flange_desired[0, 3]
pose.position.y = M_base_flange_desired[1, 3]
pose.position.z = M_base_flange_desired[2, 3]
qx, qy, qz, qw = quaternion_from_matrix(M_base_flange_desired)
pose.orientation.x = qx
pose.orientation.y = qy
pose.orientation.z = qz
pose.orientation.w = qw
```

---

## 5. T_grasp 的标定方法

T_grasp 是一次性标定的常量，两种获取方式：

### 方式 A：手动示教

1. 把物体放在一个位置，用相机或卷尺确定它在 `base_link` 下的位姿 `T_base_obj_known`
2. 手动拖动机械臂，让夹爪**成功抓住**物体
3. 记录此时 MoveIt 读出的 `link6_flange` 在 `base_link` 下的位姿 `T_base_flange_actual`
4. 计算：
   ```
   T_base_gripper_actual = T_base_flange_actual × T_flange_gripper
   T_grasp = T_base_gripper_actual⁻¹ × T_base_obj_known
   ```

### 方式 B：几何测量

1. 从 CAD 或实际测量，得到夹爪两指中点相对于 `gripper_base` 原点的偏移
2. 测量物体抓取点相对于物体坐标系的偏移
3. 两者合成 T_grasp

---

## 6. 数据流图

```
┌──────────────────┐
│ handeye_publisher │──▶ T_base_cam (静态) ──┐
│  (标定结果)        │                        │
└──────────────────┘                        ▼
                                    ┌───────────────┐
┌──────────────────┐                │ T_base_cam    │
│ 视觉检测 (ArUco)   │──▶ T_cam_obj ──▶ × T_cam_obj    │──▶ T_base_obj
│ / YOLO+depth      │                └───────────────┘        │
└──────────────────┘                                         │
                                                              ▼
┌──────────────────┐                                       ┌───────────────┐
│ T_grasp (常量)    │──▶ T_grasp⁻¹ ───────────────────────▶│ T_base_obj    │
│ gripper→object   │                                       │ × T_grasp⁻¹    │──▶ T_base_gripper_desired
└──────────────────┘                                       └───────────────┘        │
                                                                                     │
                                                                     MoveIt IK / 规划  │
                                                                                     ▼
                                                                              ┌───────────────┐
                                                                              │ 关节空间轨迹   │
                                                                              │ J1..J6        │
                                                                              └───────────────┘
```

---

## 7. 一句话总结

> **相机给物体位置，标定给相机位置，相乘得物体在基座下的位置，再乘以「夹爪应在物体什么相对位置」的常量，就是 MoveIt 的目标位姿。**
