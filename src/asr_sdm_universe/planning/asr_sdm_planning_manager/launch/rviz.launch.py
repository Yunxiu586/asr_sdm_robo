#!/usr/bin/env python3
"""ROS 2 launch file to start RViz with the Fast-Planner trajectory config."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("asr_sdm_planning_manager"), "config", "traj.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rvizvisualisation",
        output="log",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([rviz_node])
