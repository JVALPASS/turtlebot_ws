import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/app/prova/turtlebot_ws/install/turtlebot3_py_navigation'
