#!/usr/bin/env python3
"""
自动发布 5 个边界点到 /clicked_point，用于 RRT 自主探索建图。

5 个点顺序（global_rrt.cpp / local_rrt.cpp 约定）：
  点 1 (P1): 矩形左上角
  点 2 (P2): 矩形右上角  
  点 3 (P3): 矩形右下角  → P1 和 P3 决定矩形尺寸和中心
  点 4 (P4): 矩形左下角
  点 5 (P5): RRT 树根节点位置（通常是小车当前位置）
  注：P1/P2/P3/P4 顺序可以是顺/逆时针，但 P1+P3 必须是对角顶点

用法：
  # 先确保 RRT 探索节点（global_rrt / local_rrt）已启动
  ros2 run wheeltec_robot_rrt boundary_publisher.py

也可以自定义坐标（以 map 坐标系为准，单位 m）：
  python3 boundary_publisher.py --xmin -3.0 --xmax 3.0 --ymin -2.0 --ymax 2.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import time
import argparse
import sys


class BoundaryPublisher(Node):
    def __init__(self):
        super().__init__('boundary_points_publisher_node')
        # publisher + reliable QoS 保证 global_rrt/local_rrt 能收到
        self.pub = self.create_publisher(PointStamped, '/clicked_point', 10)

    def wait_for_subscribers(self, expected=2, timeout_sec=10.0):
        """等待至少 expected 个订阅者（global_rrt + local_rrt）就绪"""
        start = time.time()
        while time.time() - start < timeout_sec:
            n = self.pub.get_subscription_count()
            if n >= expected:
                self.get_logger().info(f'已连接 {n} 个订阅者')
                return True
            # 同时处理 callbacks（让 DDS discovery 跑起来）
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().warn(
            f'等待订阅者超时，当前只有 {self.pub.get_subscription_count()} 个')
        return False

    def publish_points(self, points):
        """依次发布 5 个点，每个点之间留 0.3s 确保不丢"""
        for i, (x, y) in enumerate(points, 1):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.point.x = x
            msg.point.y = y
            msg.point.z = 0.0
            self.pub.publish(msg)
            self.get_logger().info(f'发布点 {i}/5: ({x:.2f}, {y:.2f})')
            # spin_once 让消息真正被序列化发送
            rclpy.spin_once(self, timeout_sec=0.3)


def main():
    parser = argparse.ArgumentParser(
        description='发布 RRT 探索边界 5 点到 /clicked_point')
    parser.add_argument('--xmin', type=float, default=-5.0,
                        help='矩形左边界 x (默认 -5.0)')
    parser.add_argument('--xmax', type=float, default=5.0,
                        help='矩形右边界 x (默认  5.0)')
    parser.add_argument('--ymin', type=float, default=-5.0,
                        help='矩形下边界 y (默认 -5.0)')
    parser.add_argument('--ymax', type=float, default=5.0,
                        help='矩形上边界 y (默认  5.0)')
    parser.add_argument('--root_x', type=float, default=0.0,
                        help='第5个点(RRT树根) x (默认 0.0)')
    parser.add_argument('--root_y', type=float, default=0.0,
                        help='第5个点(RRT树根) y (默认 0.0)')
    args = parser.parse_args()

    # 5 个边界点 (P1=左上, P2=右上, P3=右下, P4=左下, P5=树根)
    # global_rrt.cpp 用 P1 和 P3 算矩形尺寸
    points = [
        (args.xmin, args.ymax),   # P1 左上
        (args.xmax, args.ymax),   # P2 右上
        (args.xmax, args.ymin),   # P3 右下
        (args.xmin, args.ymin),   # P4 左下
        (args.root_x, args.root_y),  # P5 树根(小车当前位置)
    ]

    rclpy.init(args=sys.argv)
    node = BoundaryPublisher()

    try:
        # 等订阅者连上（global_rrt + local_rrt 各一个）
        node.wait_for_subscribers(expected=2, timeout_sec=10.0)
        time.sleep(0.5)  # 额外保险：让 DDS 握手彻底完成
        # 发布 5 个点
        node.publish_points(points)
        # 给订阅者回调处理时间
        time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
