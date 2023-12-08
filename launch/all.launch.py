#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ompl2ros = Node(
        package='ompl_control',  
        executable='ompl2ros',  
        output='screen',
    )

    return LaunchDescription([
        ompl2ros
    ])
