#!/usr/bin/env python3
"""GPS + IMU + wheel-odometry fusion for corobo."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('corobo')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_local = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_local', output='screen',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/filtered')]
    )

    ekf_global = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_global', output='screen',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform', output='screen',
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data',           'imu/data'),
            ('gps/fix',            'gps/fix'),
            ('odometry/filtered',  'odometry/global'),
            ('odometry/gps',       'odometry/gps'),
            ('gps/filtered',       'gps/filtered'),
        ]
    )

    latlon_odom = Node(
        package='corobo', executable='latlon_odom.py',
        name='latlon_odom', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ekf_local,
        ekf_global,
        navsat,
        latlon_odom,
    ])
