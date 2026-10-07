import socketio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys

MAX_LINEAR_SPEED = 0.3  # m/s
MAX_ANGULAR_SPEED = 0.3  # rad/s

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_socket')
        
        # Create ROS 2 publisher for cmd_vel (movement)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create an instance of the Socket.IO client
        self.sio = socketio.Client()

        # Register event handlers for socket communication
        self.sio.on('drive_forward', self.forward)
        self.sio.on('drive_backward', self.backward)
        self.sio.on('drive_left', self.left)
        self.sio.on('drive_right', self.right)
        self.sio.on('drive_stop', self.stop)

        # Connect to the Socket.IO server (replace with your server URL)
        self.sio.connect('http://192.168.0.170:80', retry=True)

    def forward(self):
        self.get_logger().info("Drive forward")
        self.move_robot(MAX_LINEAR_SPEED, 0.0, 0.0, 0.0)  # Move forward with max linear speed

    def backward(self):
        self.get_logger().info("Drive backward")
        self.move_robot(-MAX_LINEAR_SPEED, 0.0, 0.0, 0.0)  # Move backward with max linear speed

    def left(self):
        self.get_logger().info("Drive left")
        self.move_robot(0.0, 0.0, 0.0, MAX_ANGULAR_SPEED)  # Turn left with max angular speed

    def right(self):
        self.get_logger().info("Drive right")
        self.move_robot(0.0, 0.0, 0.0, -MAX_ANGULAR_SPEED)  # Turn right with max angular speed

    def stop(self):
        self.get_logger().info("Stop")
        self.move_robot(0.0, 0.0, 0.0, 0.0)  # Stop the robot

    def move_robot(self, x, y, z, th):
        """Publish movement command to cmd_vel topic with speed limits."""
        twist = Twist()
        
        # Limit linear and angular velocities to max speeds
        twist.linear.x = max(-MAX_LINEAR_SPEED, min(x, MAX_LINEAR_SPEED))
        twist.linear.y = max(-MAX_LINEAR_SPEED, min(y, MAX_LINEAR_SPEED))
        twist.linear.z = max(-MAX_LINEAR_SPEED, min(z, MAX_LINEAR_SPEED))
        
        twist.angular.z = max(-MAX_ANGULAR_SPEED, min(th, MAX_ANGULAR_SPEED))

        # Publish the twist message
        self.publisher.publish(twist)


def main(args=None):
    if args is None:
        args = sys.argv

    # Initialize ROS 2
    rclpy.init(args=args)

    # Create and start the node
    node = TeleopNode()

    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.sio.disconnect()  # Disconnect the Socket.IO client
        rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()
