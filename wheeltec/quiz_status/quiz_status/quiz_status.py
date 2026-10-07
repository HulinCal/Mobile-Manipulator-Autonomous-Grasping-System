import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socketio
import time


class ImposterNode(Node):
    def __init__(self):
        super().__init__('Quiz_status')
        
        # Create ROS 2 publisher
        self.publisher_ = self.create_publisher(String, 'quiz_status', 10)
        
        # Initialize socket.io client
        self.sio = socketio.Client()

        # Register event handlers
        self.sio.on('connect', self.on_connect)
        self.sio.on('disconnect', self.on_disconnect)
        self.sio.on('quiz_finished', self.on_quiz_finished)
        self.sio.on('quiz_inactive', self.on_quiz_inactive)
        self.sio.on('drive_to_quiz_location', self.on_drive_to_quiz_location)

        # Connect to the web server
        self.sio.connect('http://192.168.0.170:80', retry=True)

    def publish_message(self, message):
        """Publishes a message to the 'quiz' topic"""
        msg = String()
        msg.data = message
        print(message)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
    
    # Event Handlers
    def on_connect(self):
        self.get_logger().info('Connected to server')
        self.publish_message("Connected to web app")

    def on_disconnect(self):
        self.get_logger().info('Disconnected from server')
        self.publish_message("Disconnected from web app")

    def on_quiz_finished(self):
        self.get_logger().info("Quiz finished")
        self.publish_message("quiz_finished")

    def on_quiz_inactive(self):
        self.get_logger().info("Quiz inactive")
        self.publish_message("quiz_inactive")

    def on_drive_to_quiz_location(self):
        self.get_logger().info("Drive to quiz location")
        self.publish_message("drive_to_quiz_location")
 





def main():
    rclpy.init()
    imposter_node = ImposterNode()
    
    
    try:
        rclpy.spin(imposter_node)  # Keep ROS 2 node running
        
    except KeyboardInterrupt:
        pass
    finally:
        imposter_node.sio.disconnect()
        
        imposter_node.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()
