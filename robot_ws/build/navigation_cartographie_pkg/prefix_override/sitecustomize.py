import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/koumba/robot_ws/install/navigation_cartographie_pkg'
