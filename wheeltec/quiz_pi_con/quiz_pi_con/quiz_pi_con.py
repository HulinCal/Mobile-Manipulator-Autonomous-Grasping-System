import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socketio
import time




class PunisherNode(Node):
    def __init__(self):
        super().__init__('quiz_pi_con')
        
        # Create ROS 2 subscriber
        self.subscription = self.create_subscription(
            String,
            'rpitopic',
            self.rpi_callback,
            10
        )
        
        # Initialize socket.io client
        self.sio = socketio.Client()
        self.sio.connect('http://192.168.0.170:80', retry=True)

    def rpi_callback(self, msg):
        """Callback function for messages received from the Raspberry Pi topic"""
        
        self.get_logger().info(f'Received from RPi: {msg.data}')
        
        if msg.data == "re":
            print("robot is exploring")
            self.sio.emit("robot-explore")
        elif msg.data == "pr":
            print("Path has been created, robot will be driving")
            self.sio.emit("robot-go-to-visitors")
            time.sleep(5)
            self.sio.emit("robot-arrived-at-visitors")



def main():
    rclpy.init()
    
    punisher_node = PunisherNode()
    
    try:
          # Keep ROS 2 node running
        rclpy.spin(punisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        
        punisher_node.sio.disconnect()
       
        punisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
