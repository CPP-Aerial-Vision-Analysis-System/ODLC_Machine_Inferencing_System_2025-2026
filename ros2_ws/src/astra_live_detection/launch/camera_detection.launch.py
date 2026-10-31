from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'model_path': ('', str), 'device': ('cpu', str),
        'confidence_threshold': ('0.3', float), 'enabled': ('true', bool),
        'slice_height': ('640', int), 'slice_width': ('640', int),
        'overlap': ('0.2', float), 'max_pending_age_sec': ('2.0', float),
        'rtsp_url': ('rtsp://192.168.144.25:8554/main.264', str),
        'width': ('1280', int), 'height': ('720', int), 'fps': ('10.0', float),
        'image_topic': ('zr10/image_raw', str),
        'snapshot_directory': ('stream_snapshots', str),
        'start_stream': ('true', bool),
        'start_camera_control': ('false', bool),
        'camera_ip': ('192.168.144.25', str), 'dry_run': ('true', bool),
    }
    detector = ['model_path', 'device', 'confidence_threshold', 'enabled',
                'slice_height', 'slice_width', 'overlap', 'max_pending_age_sec']
    stream = ['rtsp_url', 'width', 'height', 'fps', 'image_topic', 'snapshot_directory']
    declarations = [DeclareLaunchArgument(key, default_value=value[0])
                    for key, value in defaults.items()]
    return LaunchDescription(declarations + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('zr10_stream') + '/launch/stream.launch.py'),
            condition=IfCondition(LaunchConfiguration('start_stream')),
            launch_arguments={key: LaunchConfiguration(key) for key in stream}.items()),
        Node(package='astra_live_detection', executable='live_detection', output='screen',
             parameters=[{key: ParameterValue(LaunchConfiguration(key), value_type=defaults[key][1])
                          for key in detector}],
             remappings=[('zr10/image_raw', LaunchConfiguration('image_topic'))]),
        Node(package='zr10_ros2', executable='zr10_node', output='screen',
             condition=IfCondition(LaunchConfiguration('start_camera_control')),
             parameters=[{'camera_ip': ParameterValue(LaunchConfiguration('camera_ip'), value_type=str),
                          'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}]),
    ])
