import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/Roboty/Projekt/ros2_ws/src/robot_interface/install/robot_interface'
