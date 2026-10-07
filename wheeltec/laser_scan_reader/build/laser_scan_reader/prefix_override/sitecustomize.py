import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wheeltec/wheeltec_ros2/src/laser_scan_reader/install/laser_scan_reader'
