#!/usr/bin/env python3
"""
读取手眼标定结果，发布 base_link → camera_link 的静态 TF。

为什么需要这个节点？
- 标定结果是 base_link → camera_color_optical_frame
- astra_camera 发布 camera_link → camera_color_frame → camera_color_optical_frame
- 如果直接发布标定结果，camera_color_optical_frame 会有两个父节点（冲突）
- 解决方案：把标定结果转换到 base_link → camera_link，让 astra 的内部 TF 树挂接到 base_link

变换计算：
  T_base_optical   = 标定结果 (base_link → camera_color_optical_frame)
  T_link_optical   = astra_camera 内部固定变换 (camera_link → camera_color_optical_frame)
                    = identity translation, quaternion(-0.5, 0.5, -0.5, 0.5)
  T_base_link      = T_base_optical × inverse(T_link_optical)

:author hl
:date September 2026
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import tf2_ros
import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R

# astra_camera 内部固定变换: camera_link → camera_color_optical_frame
# 通过 camera_link → camera_color_frame (identity) → camera_color_optical_frame
# 旋转 quaternion (xyzw) = (-0.5, 0.5, -0.5, 0.5)
T_LINK_OPTICAL_QUAT = np.array([-0.5, 0.5, -0.5, 0.5])
T_LINK_OPTICAL_TRANS = np.array([0.0, 0.0, 0.0])

# 默认 optical → link 的固定变换（用于反向计算）
def link_optical_matrix():
    T = np.eye(4)
    T[:3, :3] = R.from_quat(T_LINK_OPTICAL_QUAT).as_matrix()
    T[:3, 3] = T_LINK_OPTICAL_TRANS
    return T


class CameraLinkTfPublisher(Node):
    """读取手眼标定，发布 base_link → camera_link 静态 TF"""

    def __init__(self):
        super().__init__('camera_link_tf_publisher')

        self.declare_parameter('calibration_name', 'mycobot280_calibration')
        name = self.get_parameter('calibration_name').get_parameter_value().string_value

        self.get_logger().info(f'Loading calibration: {name}')

        # 动态导入 easy_handeye2 (避免硬依赖路径)
        try:
            from easy_handeye2.handeye_calibration import load_calibration
        except ImportError:
            self.get_logger().fatal(
                '无法导入 easy_handeye2，请确认已 source easy_handeye2_ws/install/setup.bash')
            raise

        calib = load_calibration(name)
        params = calib.parameters

        if params.calibration_type != 'eye_on_base':
            self.get_logger().fatal(
                f'此节点仅支持 eye_on_base 标定，当前是 {params.calibration_type}')
            raise RuntimeError('Unsupported calibration type')

        # 标定结果: T_base_optical
        t = calib.transform.translation
        q = calib.transform.rotation
        T_base_optical = np.eye(4)
        T_base_optical[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T_base_optical[:3, 3] = [t.x, t.y, t.z]

        # 计算 T_base_link = T_base_optical × inverse(T_link_optical)
        T_link_optical = link_optical_matrix()
        T_base_link = T_base_optical @ np.linalg.inv(T_link_optical)

        out_t = T_base_link[:3, 3]
        out_q = R.from_matrix(T_base_link[:3, :3]).as_quat()  # xyzw

        self.get_logger().info(
            f'Publishing static TF: base_link → camera_link\n'
            f'  translation: ({out_t[0]:.6f}, {out_t[1]:.6f}, {out_t[2]:.6f})\n'
            f'  rotation xyzw: ({out_q[0]:.6f}, {out_q[1]:.6f}, {out_q[2]:.6f}, {out_q[3]:.6f})')

        # 发布静态 TF
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = params.robot_base_frame       # base_link
        msg.child_frame_id = 'camera_link'                  # 固定为 camera_link
        msg.transform.translation.x = float(out_t[0])
        msg.transform.translation.y = float(out_t[1])
        msg.transform.translation.z = float(out_t[2])
        msg.transform.rotation.x = float(out_q[0])
        msg.transform.rotation.y = float(out_q[1])
        msg.transform.rotation.z = float(out_q[2])
        msg.transform.rotation.w = float(out_q[3])
        self.broadcaster.sendTransform(msg)

        self.get_logger().info('Static TF published.')


def main(args=None):
    rclpy.init(args=args)
    node = CameraLinkTfPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
