import socket
import hashlib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PROJECTOR_IP = "192.168.1.20"  # Replace with your projector's IP
PJLINK_PORT = 4352
PJLINK_PASSWORD = ""  # Replace with your projector's password, if required

class QuizNode(Node):
    def __init__(self):
        super().__init__('Beamer')
        
        # Create ROS 2 subscriber
        self.subscription = self.create_subscription(
            String,
            'quiz',
            self.quiz_callback,  # Ensure this matches the method name
            10
        )
        

    def send_command(self, command):  # Add self parameter
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((PROJECTOR_IP, PJLINK_PORT))
        
                # Receive initial response from the projector
                initial_response = s.recv(1024).decode()
        
                if "PJLINK 1" in initial_response:  # Authentication required
                    try:
                        challenge = initial_response.split(" ")[1].strip()
                        hashed_password = hashlib.md5((challenge + PJLINK_PASSWORD).encode()).hexdigest()
                        command = f"%1POWR {hashed_password} {command[-2:]}\r"
                    except IndexError:
                        print("Error: Invalid authentication response format.")
                        return
        
                s.sendall(command.encode('utf-8'))
                response = s.recv(1024).decode()
        
                if "ERR" in response:
                    print("Error received from projector:", response)
                else:
                    print("Response:", response)
        except Exception as e:
            print(f"An error occurred while sending command: {e}")

    def quiz_callback(self, msg):
        """Callback function for messages received from the Raspberry Pi topic"""
        
        self.get_logger().info(f'Received from Quiz: {msg.data}')
        
        if msg.data == "quiz_finished":
            self.send_command("%1POWR 0\r")  # Use self to call the method
        elif msg.data == "drive_to_quiz_location":
            self.send_command("%1POWR 1\r")
            

def main():
    rclpy.init()
    
    quiz_node = QuizNode()
    
    try:
        # Keep ROS 2 node running
        rclpy.spin(quiz_node)
    except KeyboardInterrupt:
        pass
    finally:
        quiz_node.sio.disconnect()
        quiz_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
