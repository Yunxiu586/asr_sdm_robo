#!/usr/bin/env python3
"""
ROS2 Launch file for asr_sdm_camera_realsense_d405 camera node.
Subscribes to raw camera topics and republishes to standardized sensing namespace.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_in_arg = DeclareLaunchArgument(
        'image_topic_in',
        default_value='/camera/image_raw',
        description='Input camera image topic (from hardware driver or bag)'
    )

    info_in_arg = DeclareLaunchArgument(
        'camera_info_topic_in',
        default_value='/camera/camera_info',
        description='Input camera info topic'
    )

    image_out_arg = DeclareLaunchArgument(
        'image_topic_out',
        default_value='/sensing/camera/realsense/color/image_raw',
        description='Output camera image topic'
    )

    info_out_arg = DeclareLaunchArgument(
        'camera_info_topic_out',
        default_value='/sensing/camera/realsense/color/camera_info',
        description='Output camera info topic'
    )

    camera_node = Node(
        package='svo_ros',
        executable='camera',
        name='asr_sdm_camera_realsense_d405',
        output='screen',
        parameters=[{
            'image_topic_in': LaunchConfiguration('image_topic_in'),
            'camera_info_topic_in': LaunchConfiguration('camera_info_topic_in'),
            'image_topic_out': LaunchConfiguration('image_topic_out'),
            'camera_info_topic_out': LaunchConfiguration('camera_info_topic_out'),
        }],
    )

    return LaunchDescription([
        image_in_arg,
        info_in_arg,
        image_out_arg,
        info_out_arg,
        camera_node,
    ])
