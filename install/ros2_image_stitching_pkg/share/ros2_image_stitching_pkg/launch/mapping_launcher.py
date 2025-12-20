from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 🟢 Mock camera node — publishes test images
        Node(
            package='ros2_image_stitching_pkg',
            executable='mock_camera_node',
            name='mock_camera_node',
            output='screen',
            parameters=[{
                # Adjust this path to your actual test image folder
                'image_dir': '/workspace/ros2_ws/src/ros2_image_stitching_pkg/test_images'
            }]
        ),

        # 🟣 Incremental stitcher node — subscribes to camera/image_raw
        Node(
            package='ros2_image_stitching_pkg',
            executable='incremental_stitcher',
            name='incremental_stitcher',
            output='screen',
            parameters=[{
                'use_sift': True,
                'downscale_factor': 1.0,
                'ratio_test': 0.8,
                'min_matches': 10,
                'blend_method': 'multiband',
                'max_frames': 54,
                'save_dir': '/workspace/ros2_ws/panoramas'
            }]
        ),
    ])
