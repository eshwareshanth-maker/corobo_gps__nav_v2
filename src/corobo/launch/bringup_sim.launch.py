#!/usr/bin/env python3
"""Everything at once: Gazebo + fusion + Nav2 + RViz."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('corobo')
    launch_dir = os.path.join(pkg, 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time')
    args = {'use_sim_time': use_sim_time}.items()

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sim.launch.py')),
        launch_arguments=args)

    loc = TimerAction(period=6.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'localization.launch.py')),
            launch_arguments=args)])

    nav = TimerAction(period=12.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments=args)])

    rviz = TimerAction(period=8.0, actions=[
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'corobo.rviz')],
             parameters=[{'use_sim_time': use_sim_time}])])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        sim, loc, rviz, nav,
    ])
