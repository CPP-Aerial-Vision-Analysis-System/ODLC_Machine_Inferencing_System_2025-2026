from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file for SAHI Object Detection Node
    
    This launch file starts the SAHI-based object detection node with
    configurable parameters optimized for small object detection in aerial imagery.
    """
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolo11s.pt',
        description='Path to YOLO model file'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.15',
        description='Confidence threshold for detections (lower = more sensitive)'
    )
    
    slice_height_arg = DeclareLaunchArgument(
        'slice_height',
        default_value='512',
        description='Height of each slice for SAHI (smaller = better for small objects)'
    )
    
    slice_width_arg = DeclareLaunchArgument(
        'slice_width',
        default_value='512',
        description='Width of each slice for SAHI (smaller = better for small objects)'
    )
    
    overlap_height_ratio_arg = DeclareLaunchArgument(
        'overlap_height_ratio',
        default_value='0.3',
        description='Vertical overlap ratio between slices (0.3 = 30%)'
    )
    
    overlap_width_ratio_arg = DeclareLaunchArgument(
        'overlap_width_ratio',
        default_value='0.3',
        description='Horizontal overlap ratio between slices (0.3 = 30%)'
    )
    
    check_interval_arg = DeclareLaunchArgument(
        'check_interval',
        default_value='2.0',
        description='Interval in seconds to check for new images'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='auto',
        description='Device to use for inference (auto, cpu, cuda:0, mps)'
    )
    
    # Create the SAHI object detection node
    sahi_detection_node = Node(
        package='video_cam',
        executable='object_detection_sahi',
        name='sahi_object_detection_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'slice_height': LaunchConfiguration('slice_height'),
            'slice_width': LaunchConfiguration('slice_width'),
            'overlap_height_ratio': LaunchConfiguration('overlap_height_ratio'),
            'overlap_width_ratio': LaunchConfiguration('overlap_width_ratio'),
            'check_interval': LaunchConfiguration('check_interval'),
            'device': LaunchConfiguration('device'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        model_path_arg,
        confidence_threshold_arg,
        slice_height_arg,
        slice_width_arg,
        overlap_height_ratio_arg,
        overlap_width_ratio_arg,
        check_interval_arg,
        device_arg,
        sahi_detection_node,
    ])
