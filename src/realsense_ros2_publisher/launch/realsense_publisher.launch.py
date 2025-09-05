from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_ros2_publisher',
            executable='realsense_publisher',
            name='realsense_publisher',
            parameters=[{
                'ip_address': '192.168.1.189',
                'use_network': True,
                'color_width': 640,
                'color_height': 480,
                'depth_width': 640,
                'depth_height': 480,
                'framerate': 30
            }],
            output='screen',
            emulate_tty=True
        ),
        # Static transform publisher for RTAB-Map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_color_optical_frame']
        )
    ])
