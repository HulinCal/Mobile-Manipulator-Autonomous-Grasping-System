#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanSubscriber(Node):

	def __init__(self):
		super().__init__('laser_scan_subscriber')
		self.subscription = self.create_subscription(
			LaserScan,
			'scan',
			self.listener_callback,
			10)
			self.subscribtion
	def listener_callback(self, msg):
		sefl.get_logger().info(f"Laser scan received: [min_angle: {msg.angle_max}, max_angle {msg.angle_max}]")
		self.get_logger().info(f"Number of ranges: {len(msg.ranges)}")
		self.get_logger().info(f"Range data: {msg.ranges[:5]}...")
def main(args = None):
	rclpy.init(args = args)
	laser_scan_subscriber = LaserScanSubscriber()
	rclpy.spin(laser_scan_subscriber)
	
	laser_scan_subscriber.destroy_node()
	rclpy.shutdown()
if __name__ == '__main__':
	main()
