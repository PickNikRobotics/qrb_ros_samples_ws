# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear


import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_share_amr_carto = FindPackageShare(package='cartographer_ros').find('cartographer_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')

    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution,
                   '-publish_period_sec', publish_period_sec,
                   '-enable_standard_gridmap',
                #    '-include_unfrozen_submaps',
                #    '-include_frozen_submaps',
                   ]
    )

    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    return ld