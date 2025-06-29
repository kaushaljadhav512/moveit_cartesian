import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kaushal/kaushal/moveit_ws/install/launch_param_builder'
