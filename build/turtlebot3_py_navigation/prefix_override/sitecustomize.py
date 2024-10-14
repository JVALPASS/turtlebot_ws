import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/app/TURTLEBOT_WS/install/turtlebot3_py_navigation'
