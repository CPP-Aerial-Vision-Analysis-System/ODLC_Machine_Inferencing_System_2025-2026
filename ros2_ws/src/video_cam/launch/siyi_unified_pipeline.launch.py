
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    camera_ip = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.144.25',
        description='IP address of SIYI camera'
    )

    resolution = DeclareLaunchArgument(
        'resolution',
        default_value='4K',
        description='Photo resolution (4K, 2.7K, or 1080P)'
    )

    min_altitude_agl = DeclareLaunchArgument(
        'min_altitude_agl',
        default_value='-13.716',
        description='Altitude threshold above which the camera is enabled'
    )

    rotate_180 = DeclareLaunchArgument(
        'rotate_180',
        default_value='true',
        description='Rotate captured images 180 degrees (set to false if camera is mounted right-side-up)'
    )

    # Unified pipeline node
    pipeline_node = Node(
        package='video_cam',
        executable='siyi',
        name='siyi',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_real_camera': True,
            'camera_ip': LaunchConfiguration('camera_ip'),
            'resolution': LaunchConfiguration('resolution'),
            'min_altitude_agl': LaunchConfiguration('min_altitude_agl'),
            'rotate_180': LaunchConfiguration('rotate_180'),
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
        ]
    )

    return LaunchDescription([
        camera_ip,
        resolution,
        min_altitude_agl,
        rotate_180,
        LogInfo(msg='Starting SIYI Unified Pipeline...'),
        pipeline_node,
        LogInfo(msg='SIYI Unified Pipeline launched successfully'),
    ])
