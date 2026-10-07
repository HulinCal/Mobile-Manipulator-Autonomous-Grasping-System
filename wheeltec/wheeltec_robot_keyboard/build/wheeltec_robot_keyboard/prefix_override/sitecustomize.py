import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_keyboard/install/wheeltec_robot_keyboard'
