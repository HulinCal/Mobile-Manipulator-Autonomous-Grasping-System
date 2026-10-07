#!/usr/bin/env python3
"""Apple detection node (YOLOv8 + depth -> 3D pose).

Subscribes to an RGB image and a depth image / point cloud, runs YOLOv8
to detect "apple", and computes the 3D position of the apple centre in
the camera frame, then transforms it into ``base_link`` (or ``map``).

Outputs
-------
* ``/pick_place/apple_pose`` (geometry_msgs/PoseStamped) - apple pose in
  ``base_link`` frame (position = apple centre, orientation = identity).
* ``/pick_place/detection_image`` (sensor_msgs/Image) - optional debug
  image with bounding box drawn.
* ``/pick_place/apple_detected`` (std_msgs/Bool) - True when an apple is
  currently detected.

Services
--------
* ``/pick_place/detect_apple`` (std_srvs/Trigger) - run a single detection
  cycle and return the latest apple pose.

Modes
-----
* ``sim_mode:=true`` : no camera required. Publishes the configured
  ``sim_apple_pose`` (in base_link) at a fixed rate so the rest of the
  pipeline can be tested without hardware.
* ``sim_mode:=false``: subscribes to the real camera topics.

Depth source priority
---------------------
1. If a point cloud topic is available, use it (most accurate).
2. Otherwise use depth image + camera_info (pinhole projection).
"""

import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_srvs.srv import Trigger

from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped

# Optional inference backends (handled gracefully for sim-only usage)
# 1. YOLO26 ONNX (recommended, edge deployment): onnxruntime + pre-exported
#    yolo26n.onnx (NMS-free end2end, exported from ~/YOLO/yolov26 source).
# 2. ultralytics fallback: load .pt directly via ultralytics package.
try:
    import onnxruntime as ort
    _HAS_ORT = True
except ImportError:
    _HAS_ORT = False

try:
    from ultralytics import YOLO
    _HAS_YOLO = True
except ImportError:
    _HAS_YOLO = False

try:
    import cv2
except ImportError:
    cv2 = None


# COCO class id for "apple" (both YOLOv8 and YOLO26 COCO pretrained)
COCO_APPLE_CLASS_ID = 47


def _quat_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class AppleDetectionNode(Node):

    def __init__(self):
        super().__init__('apple_detection_node')

        # ---- parameters --------------------------------------------------
        self.declare_parameter('sim_mode', True)
        # 'yolo26_onnx' (edge deployment, recommended) or 'ultralytics' (.pt)
        self.declare_parameter('model_type', 'yolo26_onnx')
        self.declare_parameter('model_path',
                               '/home/admi/wheeltec_mycobot_ws/src/models/yolo26n.onnx')
        self.declare_parameter('target_class', 'apple')
        self.declare_parameter('target_class_id', COCO_APPLE_CLASS_ID)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('depth_info_topic', '/camera/color/camera_info')
        self.declare_parameter('pointcloud_topic', '/camera/depth/color/points')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('sim_apple_pose',
                               [0.5, 0.0, 0.25, 0.0, 0.0, 0.0])
        self.declare_parameter('sim_publish_rate', 2.0)
        # 真机检测: 深度在苹果区域会多秒级闪烁, 服务在此时间窗内持续重试
        self.declare_parameter('detect_timeout', 10.0)

        self.sim_mode = self.get_parameter('sim_mode').value
        self.model_type = self.get_parameter('model_type').value
        self.model_path = self.get_parameter('model_path').value
        self.target_class = self.get_parameter('target_class').value
        self.target_class_id = int(self.get_parameter('target_class_id').value)
        self.input_size = int(self.get_parameter('input_size').value)
        self.conf_thresh = float(self.get_parameter('confidence_threshold').value)
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_info_topic = self.get_parameter('depth_info_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.publish_debug = self.get_parameter('publish_debug_image').value
        sim_pose = self.get_parameter('sim_apple_pose').value
        self.sim_apple_pose = list(sim_pose) if sim_pose else \
            [0.5, 0.0, 0.25, 0.0, 0.0, 0.0]
        sim_rate = float(self.get_parameter('sim_publish_rate').value)
        self.detect_timeout = float(self.get_parameter('detect_timeout').value)

        # ---- state -------------------------------------------------------
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_pointcloud = None
        self.latest_apple_pose = None
        self.detected = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- publishers --------------------------------------------------
        # TRANSIENT_LOCAL (latched): coordinator 在 detect 服务返回后才
        # 临时订阅该话题, 必须用 latched 才能拿到发布过的最新位姿。
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/pick_place/apple_pose', pose_qos)
        self.detected_pub = self.create_publisher(Bool, '/pick_place/apple_detected', 10)
        self.status_pub = self.create_publisher(String, '/pick_place/status', 10)
        self.debug_pub = self.create_publisher(Image, '/pick_place/detection_image', 10)

        # ---- subscribers -------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        # 传感器订阅放在独立可重入回调组: 服务回调在重试等待期间需要
        # 持续接收新图像帧 (配合 MultiThreadedExecutor)。
        self.sensor_cbg = rclpy.callback_groups.ReentrantCallbackGroup()
        if not self.sim_mode:
            self.create_subscription(
                Image, self.image_topic, self._on_color, qos,
                callback_group=self.sensor_cbg)
            self.create_subscription(
                Image, self.depth_topic, self._on_depth, qos,
                callback_group=self.sensor_cbg)
            self.create_subscription(
                CameraInfo, self.depth_info_topic, self._on_camera_info, 10,
                callback_group=self.sensor_cbg)
            # pointcloud optional
            try:
                self.create_subscription(
                    PointCloud2, self.pointcloud_topic,
                    self._on_pointcloud, qos,
                    callback_group=self.sensor_cbg)
            except Exception:
                pass

        # ---- services ----------------------------------------------------
        self.detect_srv = self.create_service(
            Trigger, '/pick_place/detect_apple', self._on_detect)

        # ---- YOLO model --------------------------------------------------
        self.ort_session = None   # for yolo26_onnx backend
        self.model = None         # for ultralytics backend
        self.model_ready = False
        if not self.sim_mode:
            if not os.path.exists(self.model_path):
                self.get_logger().error(
                    f'Model file not found: {self.model_path}')
            elif self.model_type == 'yolo26_onnx':
                if not _HAS_ORT:
                    self.get_logger().error(
                        'onnxruntime not installed. Run: '
                        'pip install onnxruntime')
                else:
                    try:
                        providers = ['CPUExecutionProvider']
                        # Use GPU if available (edge devices with GPU)
                        avail = ort.get_available_providers()
                        if 'CUDAExecutionProvider' in avail:
                            providers.insert(0, 'CUDAExecutionProvider')
                        self.ort_session = ort.InferenceSession(
                            self.model_path, providers=providers)
                        self.model_ready = True
                        self.get_logger().info(
                            f'YOLO26 ONNX model loaded: {self.model_path} '
                            f'(providers={providers})')
                    except Exception as e:
                        self.get_logger().error(f'Failed to load ONNX: {e}')
            else:  # ultralytics backend
                if not _HAS_YOLO:
                    self.get_logger().error(
                        'ultralytics not installed. Run: pip install ultralytics')
                else:
                    try:
                        self.model = YOLO(self.model_path)
                        self.model_ready = True
                        self.get_logger().info(
                            f'ultralytics model loaded: {self.model_path}')
                    except Exception as e:
                        self.get_logger().error(
                            f'Failed to load ultralytics model: {e}')

        # ---- sim / real timer -------------------------------------------
        if self.sim_mode:
            self.sim_timer = self.create_timer(1.0 / max(sim_rate, 0.1),
                                               self._sim_publish)
            self.get_logger().info(
                f'apple_detection_node SIM mode | '
                f'publishing apple at {self.sim_apple_pose} in {self.target_frame}')
        else:
            self.get_logger().info(
                f'apple_detection_node REAL mode | waiting for camera on '
                f'{self.image_topic}')

    # ==================================================================
    # SIM mode
    # ==================================================================
    def _sim_publish(self):
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.sim_apple_pose[0])
        pose.pose.position.y = float(self.sim_apple_pose[1])
        pose.pose.position.z = float(self.sim_apple_pose[2])
        yaw = float(self.sim_apple_pose[5]) if len(self.sim_apple_pose) > 5 else 0.0
        pose.pose.orientation = _quat_from_yaw(yaw)
        self.latest_apple_pose = pose
        self.detected = True
        self.pose_pub.publish(pose)
        d = Bool(); d.data = True
        self.detected_pub.publish(d)

    # ==================================================================
    # REAL mode callbacks
    # ==================================================================
    def _on_color(self, msg: Image):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge color error: {e}')

    def _on_depth(self, msg: Image):
        try:
            # depth image is typically 16UC1 (mm) or 32FC1 (m)
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge depth error: {e}')

    def _on_camera_info(self, msg: CameraInfo):
        self.latest_camera_info = msg

    def _on_pointcloud(self, msg: PointCloud2):
        # Store raw; actual extraction is done on demand in _detect_once
        self.latest_pointcloud = msg

    # ==================================================================
    # Detection logic
    # ==================================================================
    def _detect_once(self):
        """Run one detection cycle.

        Returns (best_xyxy|None in image pixels, debug_image|None).
        """
        if self.sim_mode:
            return None, None

        if self.latest_color is None or not self.model_ready:
            return None, None

        if self.model_type == 'yolo26_onnx':
            return self._infer_yolo26_onnx(self.latest_color)
        return self._infer_ultralytics(self.latest_color)

    # ------------------------------------------------------------------
    def _infer_yolo26_onnx(self, img_bgr):
        """YOLO26 ONNX inference (NMS-free end2end).

        ONNX output: (1, 300, 6) rows of [x1, y1, x2, y2, score, class_id]
        in letterboxed input space -> un-letterbox to original image.
        """
        if cv2 is None:
            self.get_logger().error('opencv not available for preprocessing')
            return None, None

        s = self.input_size
        h, w = img_bgr.shape[:2]
        r = min(s / h, s / w)
        nh, nw = int(round(h * r)), int(round(w * r))
        resized = cv2.resize(img_bgr, (nw, nh))
        pad_w, pad_h = s - nw, s - nh
        padded = cv2.copyMakeBorder(
            resized, pad_h // 2, pad_h - pad_h // 2,
            pad_w // 2, pad_w - pad_w // 2,
            cv2.BORDER_CONSTANT, value=(114, 114, 114))
        blob = cv2.dnn.blobFromImage(
            padded, 1.0 / 255.0, (s, s), swapRB=True)

        inp_name = self.ort_session.get_inputs()[0].name
        out = self.ort_session.run(None, {inp_name: blob})[0][0]  # (300, 6)

        # Filter by score and target class
        scores = out[:, 4]
        classes = out[:, 5].astype(int)
        mask = scores >= self.conf_thresh
        if self.target_class_id >= 0:
            mask &= classes == self.target_class_id
        dets = out[mask]

        debug_img = None
        if self.publish_debug:
            debug_img = img_bgr.copy()
            if cv2 is not None and len(dets) > 0:
                for row in dets[:5]:
                    x1, y1, x2, y2 = self._unletterbox_box(row[:4], r,
                                                           pad_w // 2, pad_h // 2)
                    cv2.rectangle(debug_img, (int(x1), int(y1)),
                                  (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(debug_img,
                                f'{self.target_class} {row[4]:.2f}',
                                (int(x1), max(int(y1) - 6, 0)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0), 2)

        if len(dets) == 0:
            return None, debug_img

        best = dets[dets[:, 4].argmax()]
        x1, y1, x2, y2 = self._unletterbox_box(best[:4], r,
                                               pad_w // 2, pad_h // 2)
        return (float(x1), float(y1), float(x2), float(y2)), debug_img

    def _unletterbox_box(self, box, r, pad_x, pad_y):
        """Map letterboxed 640-space box back to original image pixels."""
        x1 = (box[0] - pad_x) / r
        y1 = (box[1] - pad_y) / r
        x2 = (box[2] - pad_x) / r
        y2 = (box[3] - pad_y) / r
        return x1, y1, x2, y2

    # ------------------------------------------------------------------
    def _infer_ultralytics(self, img_bgr):
        """Fallback: ultralytics .pt inference. Returns (xyxy|None, debug)."""
        results = self.model(img_bgr, verbose=False)[0]
        best_xyxy = None
        best_conf = 0.0
        for box in results.boxes:
            cls_id = int(box.cls.item())
            conf = float(box.conf.item())
            cls_name = self.model.names.get(cls_id, '')
            if cls_name.lower() == self.target_class.lower() and \
                    conf >= self.conf_thresh and conf > best_conf:
                best_xyxy = box.xyxy.cpu().numpy()[0]
                best_conf = conf

        debug_img = None
        if self.publish_debug:
            try:
                debug_img = results.plot()
            except Exception:
                debug_img = img_bgr.copy()
                if best_xyxy is not None and cv2 is not None:
                    p = best_xyxy.astype(int)
                    cv2.rectangle(debug_img, (p[0], p[1]), (p[2], p[3]),
                                  (0, 255, 0), 2)

        if best_xyxy is None:
            return None, debug_img
        return tuple(float(v) for v in best_xyxy), debug_img

    # ------------------------------------------------------------------
    def _box_to_pose(self, xyxy):
        """Convert pixel box to 3D pose in target frame. Returns PoseStamped."""
        cx = float((xyxy[0] + xyxy[2]) / 2.0)
        cy = float((xyxy[1] + xyxy[3]) / 2.0)

        # ---- get 3D point from depth -----------------------------------
        # 首选: 在整个检测框内取有效深度的主导聚类 (苹果表面深度常呈斑块状,
        # 中心点甚至 ±15px 都可能是空洞)。失败再退回中心邻域采样。
        point_cam = self._depth_box_to_3d(xyxy)
        if point_cam is None:
            bw = xyxy[2] - xyxy[0]
            bh = xyxy[3] - xyxy[1]
            radius = int(max(3, min(15, round(min(bw, bh) / 6.0))))
            point_cam = self._depth_to_3d(cx, cy, radius)
        if point_cam is None:
            self.get_logger().warn('Apple detected but no depth available')
            return None

        # ---- transform to target frame ---------------------------------
        pose_cam = PoseStamped()
        pose_cam.header.frame_id = self.camera_frame
        pose_cam.header.stamp = self.get_clock().now().to_msg()
        pose_cam.pose.position = Point(x=point_cam[0], y=point_cam[1], z=point_cam[2])
        pose_cam.pose.orientation.w = 1.0

        pose_target = self._transform_pose(pose_cam, self.target_frame)
        if pose_target is None:
            self.get_logger().warn(
                f'Could not transform apple pose to {self.target_frame}')
            return None

        return pose_target

    def _depth_box_to_3d(self, xyxy, margin_ratio=0.06, min_ratio=0.03):
        """Estimate the object 3D point from valid depths inside (and around)
        the detection box.

        The box is first shrunk by ``margin_ratio`` to avoid edge background.
        If not enough valid pixels are found (e.g. the object surface does
        not reflect the structured-light IR, like a red apple), the box is
        progressively expanded (1.5x, 2.0x, 3.0x) to sample surrounding
        pixels at a similar depth. The dominant 50 mm depth cluster's median
        is returned. Returns (x, y, z) in the camera frame, or None.
        """
        if self.latest_depth is None or self.latest_camera_info is None:
            return None
        K = self.latest_camera_info.k
        fx, fy = K[0], K[4]
        cx0, cy0 = K[2], K[5]
        if not all(np.isfinite([fx, fy, cx0, cy0])) or fx == 0 or fy == 0:
            return None

        h, w = self.latest_depth.shape[:2]
        x1, y1, x2, y2 = xyxy
        bw, bh = x2 - x1, y2 - y1

        # Try shrunk box first, then progressively expand.
        # factor 1.0 -> shrink by margin_ratio; factors >1 -> expand outward.
        for factor in (1.0, 1.5, 2.0, 3.0):
            if factor == 1.0:
                sx1 = int(np.clip(x1 + bw * margin_ratio, 0, w - 1))
                sx2 = int(np.clip(x2 - bw * margin_ratio, 0, w))
                sy1 = int(np.clip(y1 + bh * margin_ratio, 0, h - 1))
                sy2 = int(np.clip(y2 - bh * margin_ratio, 0, h))
            else:
                pad = (factor - 1.0) / 2.0
                sx1 = int(np.clip(x1 - bw * pad, 0, w - 1))
                sx2 = int(np.clip(x2 + bw * pad, 0, w))
                sy1 = int(np.clip(y1 - bh * pad, 0, h - 1))
                sy2 = int(np.clip(y2 + bh * pad, 0, h))
            if sx2 <= sx1 or sy2 <= sy1:
                continue

            region = self.latest_depth[sy1:sy2, sx1:sx2].astype(np.float64)
            if np.issubdtype(self.latest_depth.dtype, np.integer):
                region /= 1000.0
            vv, uu = np.where((region > 0.0) & (region < 10.0))
            area = (sx2 - sx1) * (sy2 - sy1)
            if len(vv) < max(10, int(area * min_ratio)):
                continue
            zs = region[vv, uu]

            # 50 mm bins; 苹果红色表面深度常完全空洞, 框内有效像素多为
            # 苹果边缘/桌面或背景。选"最近的显著 bin" (苹果是前景, 比背景
            # 近), count 过滤孤立飞点。用主导 bin 会取到背景深度 (偏远)。
            lo = np.floor(zs.min() / 0.05) * 0.05
            hist, edges = np.histogram(zs, bins=np.arange(lo, zs.max() + 0.1, 0.05))
            sig = hist >= max(5, int(area * 0.0005))
            if not sig.any():
                sig = hist == hist.max()
            idx = int(np.where(sig)[0][0])   # bins 按 z 升序, 第一个即最近
            in_bin = (zs >= edges[idx]) & (zs < edges[idx + 1])
            if in_bin.sum() < 10:
                in_bin = np.ones(len(zs), dtype=bool)

            z = float(np.median(zs[in_bin]))
            # 水平坐标仍用检测框中心 (苹果本身位置), 深度用周围像素估计
            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0
            x = (u - cx0) * z / fx
            y = (v - cy0) * z / fy
            return (x, y, z)

        return None

    def _depth_to_3d(self, u, v, search_radius=5):
        """Convert pixel (u,v) to 3D point in camera frame using depth image.

        If the exact pixel has no valid return, valid depths in a
        +/-search_radius window are used and their median is taken.
        """
        if self.latest_depth is None or self.latest_camera_info is None:
            return None
        K = self.latest_camera_info.k
        fx, fy = K[0], K[4]
        cx0, cy0 = K[2], K[5]
        # 拦截 NaN/0 内参 (NaN != 0, 必须用 isfinite 检查)
        if not all(np.isfinite([fx, fy, cx0, cy0])) or fx == 0 or fy == 0:
            return None

        h, w = self.latest_depth.shape[:2]
        u_i = int(np.clip(u, 0, w - 1))
        v_i = int(np.clip(v, 0, h - 1))
        r = max(0, int(search_radius))
        patch = self.latest_depth[
            max(v_i - r, 0):min(v_i + r + 1, h),
            max(u_i - r, 0):min(u_i + r + 1, w)].astype(np.float64)
        # Handle both mm (uint16) and m (float) depth encodings
        if np.issubdtype(self.latest_depth.dtype, np.integer):
            patch /= 1000.0
        valid = np.isfinite(patch) & (patch > 0.0) & (patch < 10.0)
        if not valid.any():
            return None
        z = float(np.median(patch[valid]))
        x = (u - cx0) * z / fx
        y = (v - cy0) * z / fy
        return (x, y, z)

    def _transform_pose(self, pose_in: PoseStamped, target_frame: str):
        if pose_in.header.frame_id == target_frame:
            return pose_in
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, pose_in.header.frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            return do_transform_pose_stamped(pose_in, transform)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    # ==================================================================
    # Service
    # ==================================================================
    def _on_detect(self, request, response):
        """Run one detection cycle: pixel box -> 3D pose -> publish."""
        response.success = False
        response.message = 'No apple detected'

        if self.sim_mode:
            # In sim mode the latest apple pose is published periodically
            if self.latest_apple_pose is not None:
                response.success = True
                p = self.latest_apple_pose.pose.position
                response.message = (
                    f'[SIM] apple at x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} '
                    f'in {self.latest_apple_pose.header.frame_id}')
            return response

        # 苹果区域的深度会呈多秒级闪烁(有效窗约 47% 帧, 无效窗 0%),
        # 单次检测经常失败 -> 在 detect_timeout 秒内持续重试等待有效帧。
        # 兜底: 任何意外异常只返回失败响应, 绝不让异常穿透 executor
        # 导致整个检测节点进程退出。
        import time
        try:
            deadline = time.time() + self.detect_timeout
            attempts = 0
            pose = None
            while time.time() < deadline:
                attempts += 1
                xyxy, debug_img = self._detect_once()
                if debug_img is not None and self.publish_debug:
                    try:
                        self.debug_pub.publish(
                            self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
                    except Exception:
                        pass
                if xyxy is not None:
                    pose = self._box_to_pose(xyxy)
                    if pose is not None:
                        break
                # 等待期间传感器回调组持续接收新帧(MultiThreadedExecutor)
                time.sleep(0.4)

            if pose is not None:
                self.latest_apple_pose = pose
                self.detected = True
                self.pose_pub.publish(pose)
                d = Bool(); d.data = True
                self.detected_pub.publish(d)
                response.success = True
                p = pose.pose.position
                response.message = (
                    f'apple at x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} '
                    f'in {pose.header.frame_id} (after {attempts} attempts)')
                return response

            self.detected = False
            d = Bool(); d.data = False
            self.detected_pub.publish(d)
            response.message = \
                f'No apple detected after {attempts} attempts'
        except Exception as e:
            self.get_logger().error(f'detect cycle failed: {e}')
            response.success = False
            response.message = f'detect cycle failed: {e}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AppleDetectionNode()
    # 多线程执行器: 服务回调重试等待时, 传感器回调仍能接收新帧
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
