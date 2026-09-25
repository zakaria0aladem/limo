from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Tune without editing the file, e.g.:
    #   ros2 launch mocap_localization mocap_localization.launch.py \
    #       reg_yaw:=0.05 jump_threshold:=0.3
    args = {
        'mocap_topic': '/vrpn_mocap/Limo/pose',
        # registration offset (map <- world); 0 = map identical to world
        'reg_x': '0.0',
        'reg_y': '0.0',
        'reg_yaw': '0.0',
        # reject mocap flips/dropouts larger than this many meters; 0 = off
        'jump_threshold': '0.0',
        'publish_rate': '30.0',
    }
    declare_args = [DeclareLaunchArgument(k, default_value=v)
                    for k, v in args.items()]

    def f(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    return LaunchDescription(declare_args + [
        Node(
            package='mocap_localization',
            executable='mocap_map_odom',
            name='mocap_map_odom',
            output='screen',
            parameters=[{
                'mocap_topic': LaunchConfiguration('mocap_topic'),
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_rate': f('publish_rate'),
                'reg_x': f('reg_x'),
                'reg_y': f('reg_y'),
                'reg_yaw': f('reg_yaw'),
                'jump_threshold': f('jump_threshold'),
            }],
        ),
    ])
