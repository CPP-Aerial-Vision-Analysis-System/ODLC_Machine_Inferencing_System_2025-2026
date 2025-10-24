# build -> source -> run video_cam node

cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select video_cam && source install/setup.bash && ros2 run video_cam image_pub

# od for taken photos

cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws &&
colcon build --packages-select video_cam &&
source install/setup.bash &&
ros2 run video_cam object_detection