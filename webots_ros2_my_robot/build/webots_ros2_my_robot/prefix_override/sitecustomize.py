import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/robin/Documents/WEBOTS_SHIT/webots_ws/src/webots_ros2/webots_ros2_my_robot/install/webots_ros2_my_robot'
