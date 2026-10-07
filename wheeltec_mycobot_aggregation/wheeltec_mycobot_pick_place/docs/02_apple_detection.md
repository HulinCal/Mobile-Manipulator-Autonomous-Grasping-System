# 2. 苹果识别节点 (apple_detection_node)

## 功能

使用 **YOLO26**（端侧部署方案）从摄像头 RGB 图像中检测苹果，
结合深度图像计算苹果中心的 3D 位姿，并通过 TF 变换到 `base_link` 坐标系发布。

## YOLO26 端侧部署方案

本节点默认使用 **yolo26_onnx** 后端：

```
训练侧 (conda env: yolov26, ~/YOLO/yolov26 源码)
  yolo26n.pt ── export(format=onnx, imgsz=640) ──→ src/models/yolo26n.onnx

推理侧 (ROS 节点, 系统 Python 3.12)
  yolo26n.onnx ── onnxruntime (CPU/CUDA 自动选择) ──→ 检测框
```

选择 ONNX 的原因：
1. **跨环境**：yolov26 训练环境是 Python 3.11 + torch cu13（conda），ROS Jazzy
   是 Python 3.12，torch ABI 不兼容；ONNX + onnxruntime 与环境解耦
2. **端侧友好**：YOLO26 是 NMS-Free 端到端模型，ONNX 输出 `(1, 300, 6)`
   即最终检测结果，无需 NMS 后处理，推理链路简单、延迟低
3. **可替换**：边缘设备（JetBox/工控机）只需 `onnxruntime`（或
   `onnxruntime-gpu`），无需完整 torch

ONNX 输出布局：每行 `[x1, y1, x2, y2, score, class_id]`（letterbox 640 空间），
节点内部完成 letterbox 逆变换映射回原图像素。

### 重新导出模型（如需更新）

```bash
# 使用 yolov26 conda 环境导出
PYTHONPATH= /home/hl/miniconda3/envs/yolov26/bin/python -c "
from ultralytics import YOLO
model = YOLO('/home/hl/YOLO/yolo26n.pt')
model.export(format='onnx', imgsz=640, opset=17, simplify=True)
"
cp /home/hl/YOLO/yolo26n.onnx ~/wheeltec/wheeltec_ws/src/models/yolo26n.onnx
```

### 备选后端

`model_type: ultralytics` 可直接加载 `.pt`（需在 ROS Python 中安装
ultralytics，不走端侧方案，仅作调试用）。

## 数据流

```
/camera/color/image_raw ──┐
                           ├── YOLO26 ONNX → 苹果像素框 (u, v)
/camera/depth/image_raw ──┘
                           └── 深度 → 3D点 (x, y, z) in camera_frame
                                        ↓ TF 变换
                              /pick_place/apple_pose (base_link)
```

深度来源（自动）：优先点云 `/camera/depth/color/points`（当前实现用深度图+内参
针孔投影，点云话题已订阅备用）。

## 话题与服务

| 类型 | 名称 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 图像（真实） |
| 订阅 | `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图（真实） |
| 订阅 | `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| 发布 | `/pick_place/apple_pose` | `geometry_msgs/PoseStamped` | 苹果 3D 位姿 |
| 发布 | `/pick_place/apple_detected` | `std_msgs/Bool` | 是否检测到苹果 |
| 发布 | `/pick_place/detection_image` | `sensor_msgs/Image` | 带检测框的调试图 |
| 服务 | `/pick_place/detect_apple` | `std_srvs/Trigger` | 执行一次检测并返回结果 |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sim_mode` | `true` | 模拟模式 |
| `model_type` | `yolo26_onnx` | 推理后端：`yolo26_onnx` / `ultralytics` |
| `model_path` | `.../models/yolo26n.onnx` | 模型文件路径 |
| `target_class` | `apple` | 类别名（调试图标签） |
| `target_class_id` | `47` | COCO apple 类别 id（-1 表示不过滤类别） |
| `input_size` | `640` | 模型输入尺寸 |
| `confidence_threshold` | `0.4` | 置信度阈值 |
| `target_frame` | `base_link` | 输出位姿的目标坐标系 |
| `sim_apple_pose` | `[0.5, 0.0, 0.25, 0, 0, 0]` | 模拟苹果位姿 (x y z r p y) |

## 模拟测试方法

```bash
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py sim_mode:=true

# 查看模拟发布的苹果位姿
ros2 topic echo /pick_place/apple_pose --once

# 主动触发一次检测
ros2 service call /pick_place/detect_apple std_srvs/srv/Trigger
```

## 真实使用方法

```bash
# 真实模式自动订阅 Astra 相机话题并加载 ONNX 模型
ros2 launch wheeltec_mycobot_pick_place pick_place.launch.py sim_mode:=false

# 查看检测结果
ros2 topic echo /pick_place/apple_detected
ros2 topic echo /pick_place/apple_pose
ros2 topic echo /pick_place/detection_image   # 需 image_view 等工具查看
```

> YOLO26 ONNX 模型已就位：`wheeltec_ws/src/models/yolo26n.onnx`（COCO 预训练，
> 含 apple 类别 id=47）。需安装依赖：`pip install onnxruntime opencv-python`。
> 自定义苹果数据集微调训练请参考 `~/YOLO/yolov26/使用说明.md`。
