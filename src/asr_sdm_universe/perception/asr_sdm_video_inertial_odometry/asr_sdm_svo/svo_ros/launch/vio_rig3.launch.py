#!/usr/bin/env python3
"""
ROS2 Launch file for SVO VIO (Visual-Inertial Odometry) - Rig3 configuration.

Mirrors rpg_svo_pro_open/svo_ros/launch/euroc_vio_mono.launch which loads
the full VIO pipeline with IMU-prior SparseImgAlign + PoseOptimizer + optional
Local Bundle Adjustment (g2o).

Differences from test_rig3.launch.py (VO mode):
  - Loads vio_rig3.yaml instead of my_camera.yaml + imu_rig3.yaml
  - Enables pose_optim_prior_lambda_rot by default (0.5)
  - Enables local bundle adjustment (loba_num_iter=3)
  - Enables automatic re-initialization on tracking failure
  - Includes bag-playback static TF (no dynamic cam->world needed)

Usage:
  # Basic VIO (recommended starting point)
  ros2 launch svo_ros vio_rig3.launch.py

  # VIO with RViz
  ros2 launch svo_ros vio_rig3.launch.py rviz:=true

  # VIO with custom IMU prior weights
  ros2 launch svo_ros vio_rig3.launch.py \
    img_align_prior_lambda_rot:=0.5 \
    pose_optim_prior_lambda_rot:=0.5

  # Pure VO (no IMU) - same as test_rig3.launch.py
  ros2 launch svo_ros vio_rig3.launch.py use_imu:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    svo_ros_dir = get_package_share_directory('svo_ros')

    # VIO parameter file (includes camera intrinsics, IMU params, VIO weights)
    vio_yaml_path = os.path.join(svo_ros_dir, 'param', 'vio_rig3.yaml')
    imu_calib_yaml_path = os.path.join(svo_ros_dir, 'param', 'calib', 'rig3_mono.yaml')

    # Declare all launch arguments
    cam_topic_arg = DeclareLaunchArgument(
        'cam_topic',
        default_value='/sensing/camera/realsense/color/image_raw',
        description='Camera image topic')

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/sensing/imu/imu_filtered',
        description='IMU topic (typically filtered output from imu node)')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for visualization')

    rviz_sw_arg = DeclareLaunchArgument(
        'rviz_sw',
        default_value='true',
        description='Use software rendering (LIBGL_ALWAYS_SOFTWARE=1) for RViz')

    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='true',
        description='Enable IMU VIO (SparseImgAlign + PoseOptimizer prior)')

    # VIO-specific: IMU prior lambda for SparseImgAlign (frontend)
    img_align_prior_lambda_rot_arg = DeclareLaunchArgument(
        'img_align_prior_lambda_rot',
        default_value='0.5',
        description='IMU rotation prior weight in SparseImgAlign (0.5=rpg/vio_mono.yaml)')

    img_align_prior_lambda_trans_arg = DeclareLaunchArgument(
        'img_align_prior_lambda_trans',
        default_value='0.0',
        description='IMU translation prior weight in SparseImgAlign (0=pure visual for mono)')

    # VIO-specific: IMU prior lambda for PoseOptimizer (backend BA stage)
    # Set > 0 to constrain rotation toward IMU in Gauss-Newton optimization
    pose_optim_prior_lambda_rot_arg = DeclareLaunchArgument(
        'pose_optim_prior_lambda_rot',
        default_value='0.5',
        description='IMU rotation prior weight in PoseOptimizer (0=disabled, 0.5=recommended)')

    pose_optim_prior_lambda_trans_arg = DeclareLaunchArgument(
        'pose_optim_prior_lambda_trans',
        default_value='0.0',
        description='IMU translation prior weight in PoseOptimizer (0=pure visual)')

    # VIO-specific: zero-motion detection
    zero_motion_accel_std_thresh_arg = DeclareLaunchArgument(
        'zero_motion_accel_std_thresh',
        default_value='0.05',
        description='Zero-motion accel std thresh (m/s^2); below=stationary')

    # VIO-specific: IMU preprocessing mode
    imu_preprocessing_mode_arg = DeclareLaunchArgument(
        'imu_preprocessing_mode',
        default_value='0',
        description='IMU preprocessing: 0=none, 1=collect+calibrate, 2=use calibration')

    # Camera republisher (subscribes bag topics, republishes to standardized sensing namespace)
    camera_node = Node(
        package='svo_ros',
        executable='camera',
        name='asr_sdm_camera_realsense_d405',
        output='screen',
        parameters=[{
            'image_topic_in': '/camera/image_raw',
            'camera_info_topic_in': '/camera/camera_info',
            'image_topic_out': '/sensing/camera/realsense/color/image_raw',
            'camera_info_topic_out': '/sensing/camera/realsense/color/camera_info',
        }],
    )

    # IMU republisher (subscribes bag IMU, republishes raw + filtered)
    imu_node = Node(
        package='svo_ros',
        executable='imu',
        name='asr_sdm_imu_hiwonder_10axis',
        output='screen',
        parameters=[{
            'imu_topic_in': '/imu/data',
            'imu_topic_out_raw': '/sensing/imu/imu_raw',
            'imu_topic_out_filtered': '/sensing/imu/imu_filtered',
        }],
    )

    # SVO VIO node
    # Loads vio_rig3.yaml which includes:
    #   - Camera intrinsics (Pinhole, 752x480, D405 params)
    #   - IMU calibration params
    #   - VIO algorithm params (loba_num_iter=3, pose_optim_prior_lambda=0.5, etc.)
    svo_node = Node(
        package='svo_ros',
        executable='vo',
        name='asr_sdm_video_inertial_odometry',
        output='screen',
        parameters=[
            vio_yaml_path,
            {
                # Topic remapping
                'cam_topic': LaunchConfiguration('cam_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),

                # NOTE: Do NOT enable use_sim_time for bags with absolute timestamps.
                # The bag uses real Unix timestamps (March 2013). Enabling --clock
                # would use simulated time, breaking IMU-camera synchronization.

                # Initial camera orientation (downward-looking rig)
                'init_rx': 3.14,
                'init_ry': 0.0,
                'init_rz': 0.0,

                # VIO: IMU enable
                'use_imu': LaunchConfiguration('use_imu'),

                # VIO: IMU calibration file (EuRoC format with T_cam_imu)
                'imu_calib_file': imu_calib_yaml_path,

                # VIO: SparseImgAlign prior weights (frontend)
                'img_align_prior_lambda_rot': LaunchConfiguration('img_align_prior_lambda_rot'),
                'img_align_prior_lambda_trans': LaunchConfiguration('img_align_prior_lambda_trans'),

                # VIO: PoseOptimizer prior weights (backend BA stage)
                'pose_optim_prior_lambda_rot': LaunchConfiguration('pose_optim_prior_lambda_rot'),
                'pose_optim_prior_lambda_trans': LaunchConfiguration('pose_optim_prior_lambda_trans'),

                # VIO: zero-motion detection
                'zero_motion_accel_std_thresh': LaunchConfiguration('zero_motion_accel_std_thresh'),

                # VIO: IMU preprocessing
                'imu_preprocessing_mode': LaunchConfiguration('imu_preprocessing_mode'),

                # Visualization
                'enable_visualization': True,
                'publish_markers': True,
                'publish_dense_input': False,
                'publish_every_nth_img': 1,
                'publish_every_nth_dense_input': 4,
                'publish_map_every_frame': False,
            },
        ],
    )

    # RViz2 config path
    rviz_config_path = os.path.join(svo_ros_dir, 'rviz_config.rviz')

    # Software rendering env vars (for systems without GPU or VM)
    env_libgl = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))
    env_gallium = SetEnvironmentVariable(
        name='GALLIUM_DRIVER',
        value='llvmpipe',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))
    env_gl_version = SetEnvironmentVariable(
        name='MESA_GL_VERSION_OVERRIDE',
        value='3.0',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))
    env_qt_gl = SetEnvironmentVariable(
        name='QT_XCB_FORCE_SOFTWARE_OPENGL',
        value='1',
        condition=IfCondition(LaunchConfiguration('rviz_sw')))

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # Static TF: world -> camera
    # Matches the euroc_vio_mono.launch convention (world at first camera position).
    # For downward-looking cameras: initial pose is identity, world frame = camera init.
    world_to_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_cam_tf',
        arguments=[
            '0', '0', '0',   # x y z
            '0', '0', '0',  # roll pitch yaw
            'world',        # parent frame
            'camera',       # child frame (matches bag /camera/* topics)
        ],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        cam_topic_arg,
        imu_topic_arg,
        rviz_arg,
        rviz_sw_arg,
        use_imu_arg,
        img_align_prior_lambda_rot_arg,
        img_align_prior_lambda_trans_arg,
        pose_optim_prior_lambda_rot_arg,
        pose_optim_prior_lambda_trans_arg,
        zero_motion_accel_std_thresh_arg,
        imu_preprocessing_mode_arg,

        # Environment (RViz software rendering)
        env_libgl,
        env_gallium,
        env_gl_version,
        env_qt_gl,

        # Nodes: camera -> IMU -> VIO -> RViz
        camera_node,
        imu_node,
        svo_node,
        world_to_cam_tf,
        rviz_node,
    ])
