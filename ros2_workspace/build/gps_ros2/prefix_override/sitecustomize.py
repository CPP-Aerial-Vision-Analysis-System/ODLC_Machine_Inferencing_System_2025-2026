import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/ODLC_Machine_Inferencing_System_2025-2026/ros2_workspace/install/gps_ros2'
