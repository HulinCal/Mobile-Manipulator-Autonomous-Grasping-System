# Import all necessary libraries
import os
import sys
import select
import rclpy
import time
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

# Import different libraries when using different operating systems
if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# Constants
DEFAULT_SPEED = 0.2  # m/s
DEFAULT_TURN = 0.3   # rad/s

# Function to print current velocity
def print_vels(speed, turn):
    print('Currently:\tspeed {speed}\tturn {turn}')

# Functions to move the robot
# Function to drive forwards
def drive_forward(pub, speed):
    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = 0.0
    pub.publish(twist)

# Function to drive backwards
def drive_backward(pub, speed):
    twist = Twist()
    twist.linear.x = -speed
    twist.angular.z = 0.0
    pub.publish(twist)

# Function to turn left
def turn_left(pub, turn):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = turn
    pub.publish(twist)

# Function to turn right
def turn_right(pub, turn):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = -turn
    pub.publish(twist)

# Function to stop 
def stop(pub):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

# Main where we will test all the functionality
def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('wheeltec_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    # Speed and angle parameters
    speed = DEFAULT_SPEED
    turn = DEFAULT_TURN

    # Try Except statement to catch potential errors 
    try:
        while True:
            # Here we will test the functionality of this program we derived from the shipped examples with the robot
            # We will test the functions for: driving, turning and stopping so we can use it in our main program more easily 
            # Drive forward for 1 second
            drive_forward(pub, speed)
            time.sleep(1)
            stop(pub)
            # Turn left for half a second
            turn_left(pub, turn)
            time.sleep(0.5)
            stop(pub)
            # Turn right for half a second (we should end up at the starting position again)
            turn_right(pub, turn)
            time.sleep(0.5)
            stop(pub)

    except Exception as e:
        print(e)

    finally:
        stop(pub)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
