#!/usr/bin/env python3
"""Real-robot bringup. Same graph as the sim, different sources."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('corobo')
    launch_dir = os.path.join(pkg, 'launch')
    xacro_file = os.path.join(pkg, 'urdf', 'corobo.urdf.xacro')

    port = LaunchConfiguration('port')
    gps_port = LaunchConfiguration('gps_port')
    lidar_port = LaunchConfiguration('lidar_port')
    sim = {'use_sim_time': False}

    robot_description = ParameterValue(Command(['xacro ', xacro_file]),
                                       value_type=str)

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': robot_description}, sim])

    jsp = Node(package='joint_state_publisher', executable='joint_state_publisher',
               parameters=[sim])

    base = Node(package='corobo', executable='serial_base.py',
                name='serial_base', output='screen',
                parameters=[{'port': port,
                             'wheel_radius': 0.10,
                             'wheel_separation': 0.40,
                             'ticks_per_rev': 1320,
                             'publish_tf': False}, sim])

    gps = Node(package='nmea_navsat_driver', executable='nmea_serial_driver',
               name='gps_driver', output='screen',
               parameters=[{'port': gps_port, 'baud': 38400,
                            'frame_id': 'gps_link'}, sim],
               remappings=[('fix', 'gps/fix')])

    imu = Node(package='bno055', executable='bno055',
               name='imu_driver', output='screen',
               parameters=[{'ros_topic_prefix': '', 'frame_id': 'imu_link',
                            'connection_type': 'i2c'}, sim],
               remappings=[('imu', 'imu/data')])

    lidar = Node(package='rplidar_ros', executable='rplidar_composition',
                 name='rplidar', output='screen',
                 parameters=[{'serial_port': lidar_port,
                              'serial_baudrate': 115200,
                              'frame_id': 'lidar_link',
                              'angle_compensate': True,
                              'scan_mode': 'Standard'}, sim])

    loc = TimerAction(period=5.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items())])

    nav = TimerAction(period=10.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items())])

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        rsp, jsp, base, gps, imu, lidar, loc, nav,
    ])
