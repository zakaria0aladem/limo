from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mocap_localization',
            executable='mocap_map_odom',
            name='mocap_map_odom',
            output='screen',
            parameters=[{
                'mocap_topic': '/vrpn_mocap/Limo/pose',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_rate': 30.0,
                'reg_x': 0.0,
                'reg_y': 0.0,
                'reg_yaw': 0.0,
                'jump_threshold': 0.0,
            }],
        ),
    ])
