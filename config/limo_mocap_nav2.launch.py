#!/usr/bin/env python3
# ============================================================================
# limo_mocap_nav2.launch.py
# ----------------------------------------------------------------------------
# ONE command to bring up the whole laptop-side stack for mocap-localized Nav2:
#   1. map_server         — serves the Cartographer map (/map)
#   2. lifecycle_manager  — auto-activates map_server (no manual lifecycle step)
#   3. navigation_launch  — Nav2 planner/controller/costmaps/BT  (NO AMCL)
#   4. mocap_map_odom     — your OptiTrack localizer (publishes map->odom)
#
# Deliberately does NOT start AMCL (the mocap node owns map->odom) and does NOT
# start vrpn_mocap or the robot bringup — start those separately first:
#   robot (SSH):  ros2 launch limo_bringup limo_start.launch.py
#   container:    ros2 launch vrpn_mocap client.launch.yaml server:=<MOTIVE_IP> port:=3883
#
# RUN (inside the container, workspace sourced so mocap_localization is found):
#   source /opt/ros/foxy/setup.bash
#   source /root/ros2_ws/install/setup.bash
#   ros2 launch /root/maps/limo_mocap_nav2.launch.py
#
# Tune without editing the file, e.g.:
#   ros2 launch /root/maps/limo_mocap_nav2.launch.py reg_yaw:=0.05 jump_threshold:=0.3
# ============================================================================
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ---- Launch arguments (override from CLI) ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    reg_x = LaunchConfiguration('reg_x')
    reg_y = LaunchConfiguration('reg_y')
    reg_yaw = LaunchConfiguration('reg_yaw')
    jump_threshold = LaunchConfiguration('jump_threshold')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value='/root/maps/mapMTR5.yaml'),
        DeclareLaunchArgument('params_file', default_value='/root/maps/nav2.yaml'),
        # registration offset (map <- world); 0 = map identical to world
        DeclareLaunchArgument('reg_x', default_value='0.0'),
        DeclareLaunchArgument('reg_y', default_value='0.0'),
        DeclareLaunchArgument('reg_yaw', default_value='0.0'),
        # reject mocap flips/dropouts larger than this many meters; 0 = off
        DeclareLaunchArgument('jump_threshold', default_value='0.0'),
    ]

    # ---- 1. Map server (lifecycle node) ----
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    # ---- 2. Lifecycle manager that auto-activates map_server ----
    # (replaces the manual `ros2 run nav2_util lifecycle_bringup map_server`)
    # Separate from Nav2's own manager so the two don't clash.
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ---- 4. Mocap localizer (publishes map->odom; replaces AMCL) ----
    mocap_node = Node(
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
            'reg_x': ParameterValue(reg_x, value_type=float),
            'reg_y': ParameterValue(reg_y, value_type=float),
            'reg_yaw': ParameterValue(reg_yaw, value_type=float),
            'jump_threshold': ParameterValue(jump_threshold, value_type=float),
        }],
    )

    # ---- 3. Nav2 navigation (NO AMCL) ----
    # Started a few seconds later so /map is already latched when the global
    # costmap comes up (avoids "waiting for map" startup warnings).
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )
    navigation_delayed = TimerAction(period=3.0, actions=[navigation])

    return LaunchDescription(
        declare_args + [map_server, map_lifecycle, mocap_node, navigation_delayed]
    )
