#!/usr/bin/env python3
"""
ROS2 Launch file for asr_sdm_imu_hiwonder_10axis IMU node.
Subscribes to raw IMU data and publishes both raw and filtered IMU topics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu_topic_in_arg = DeclareLaunchArgument(
        'imu_topic_in',
        default_value='/sensing/imu/imu_raw',
        description='Input IMU topic (from hardware/driver)'
    )

    imu_topic_out_raw_arg = DeclareLaunchArgument(
        'imu_topic_out_raw',
        default_value='/sensing/imu/imu_raw',
        description='Output IMU topic for raw data'
    )

    imu_topic_out_filtered_arg = DeclareLaunchArgument(
        'imu_topic_out_filtered',
        default_value='/sensing/imu/imu_filtered',
        description='Output IMU topic for filtered data'
    )

    imu_node = Node(
        package='svo_ros',
        executable='imu',
        name='asr_sdm_imu_hiwonder_10axis',
        output='screen',
        parameters=[{
            'imu_topic_in': LaunchConfiguration('imu_topic_in'),
            'imu_topic_out_raw': LaunchConfiguration('imu_topic_out_raw'),
            'imu_topic_out_filtered': LaunchConfiguration('imu_topic_out_filtered'),
        }],
    )

    return LaunchDescription([
        imu_topic_in_arg,
        imu_topic_out_raw_arg,
        imu_topic_out_filtered_arg,
        imu_node,
    ])
