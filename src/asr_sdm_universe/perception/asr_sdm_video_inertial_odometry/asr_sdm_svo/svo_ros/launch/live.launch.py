#!/usr/bin/env python3
"""
ROS2 Launch file for SVO Visual Odometry - Live camera configuration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # Get the package share directory
    svo_ros_dir = get_package_share_directory('svo_ros')

    # Load camera + VO parameters from my_camera.yaml
    camera_yaml_path = os.path.join(svo_ros_dir, 'param', 'my_camera.yaml')
    with open(camera_yaml_path, 'r') as f:
        node_parameters = yaml.safe_load(f)

    # Combine with launch args
    node_parameters = {
        # Camera topic to subscribe to (override via launch argument)
        'cam_topic': LaunchConfiguration('cam_topic'),
        **node_parameters,
    }

    # Declare launch arguments
    cam_topic_arg = DeclareLaunchArgument(
        'cam_topic',
        default_value='/sensing/camera/realsense/color/image_raw',
        description='Camera image topic (from asr_sdm_camera_realsense_d405 node)'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/sensing/imu/imu_filtered',
        description='IMU topic (from asr_sdm_imu_hiwonder_10axis node)'
    )

    image_topic_in_arg = DeclareLaunchArgument(
        'image_topic_in',
        default_value='/camera/image_raw',
        description='Input camera image topic (from hardware driver or bag)'
    )

    camera_info_topic_in_arg = DeclareLaunchArgument(
        'camera_info_topic_in',
        default_value='/camera/camera_info',
        description='Input camera info topic'
    )

    imu_topic_in_arg = DeclareLaunchArgument(
        'imu_topic_in',
        default_value='/imu/data',
        description='Input IMU topic'
    )

    camera_node = Node(
        package='svo_ros',
        executable='camera',
        name='asr_sdm_camera_realsense_d405',
        output='screen',
        parameters=[{
            'image_topic_in': LaunchConfiguration('image_topic_in'),
            'camera_info_topic_in': LaunchConfiguration('camera_info_topic_in'),
            'image_topic_out': '/sensing/camera/realsense/color/image_raw',
            'camera_info_topic_out': '/sensing/camera/realsense/color/camera_info',
        }],
    )

    imu_node = Node(
        package='svo_ros',
        executable='imu',
        name='asr_sdm_imu_hiwonder_10axis',
        output='screen',
        parameters=[{
            'imu_topic_in': LaunchConfiguration('imu_topic_in'),
            'imu_topic_out_raw': '/sensing/imu/imu_raw',
            'imu_topic_out_filtered': '/sensing/imu/imu_filtered',
        }],
    )

    svo_node = Node(
        package='svo_ros',
        executable='vo',
        name='asr_sdm_video_inertial_odometry',
        output='screen',
        parameters=[node_parameters],
    )

    return LaunchDescription([
        image_topic_in_arg,
        camera_info_topic_in_arg,
        imu_topic_in_arg,
        cam_topic_arg,
        imu_topic_arg,
        camera_node,
        imu_node,
        svo_node,
    ])


