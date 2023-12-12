#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name='ompl_control'
    share_dir = get_package_share_directory(package_name)

    ompl2ros = Node(
        package='ompl_control',  
        executable='ompl2ros',  
        output='screen',
    )

    # uav_sdf = os.path.join(share_dir, 'models', 'uav.sdf')
    # spawn_uav = Node(package='gazebo_ros', executable='spawn_entity.py', 
    #             name='spawn_uav',
    #             arguments=['-entity', 'my_uav', '-file', uav_sdf],
    #             output='screen')

    # launch_dir = os.path.join(share_dir, 'launch')
    # rviz_config_path = os.path.join(share_dir, 'config', 'amp.rviz')
    # start_rviz2_cmd = ExecuteProcess(
    #     cmd=['rviz2', '--display-config', rviz_config_path],
    #     cwd=[launch_dir],
    #     output='screen')

    return LaunchDescription([
        ompl2ros,
        # spawn_uav,
        # start_rviz2_cmd
    ])
