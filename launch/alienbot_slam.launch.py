#!/usr/bin/env python3
"""Launch robot_state_publisher with URDF and SLAM toolbox."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve URDF path relative to this launch file
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    repo_dir = os.path.dirname(launch_dir)
    urdf_path = os.path.join(repo_dir, 'description', 'urdf', 'robot_v4.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    slam_config = os.path.join(
        '/', 'opt', 'ros', 'humble', 'share',
        'slam_toolbox', 'config', 'mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        # Publish URDF → TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # SLAM — builds map from /scan + /tf
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_config,
                {
                    'use_sim_time': False,
                    'base_frame': 'base_link',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                    'scan_topic': '/scan',
                }
            ],
            output='screen',
        ),
    ])
