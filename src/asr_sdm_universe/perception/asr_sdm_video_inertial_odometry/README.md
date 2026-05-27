# Installation

## Dependency
```sh
sudo apt-get -y install ros-$ROS_DISTRO-sophus libgoogle-glog-dev
```

## Build
```sh
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --parallel-workers 8
```

> **Note:** SVO is a compute-intensive visual odometry algorithm. Always use `Release` build for performance. Debug builds will run significantly slower.

# Run Instructions

### Run using ROS2 Jazzy



### Visual Odometry demo (rosbag + RViz2)
This section shows how to run the SVO visual odometry node with the included rosbag and visualize results in RViz2.

Prerequisites
`rm -rf build install log`
`source /opt/ros/$ROS_DISTRO/setup.bash`
- Workspace has been built: `colcon build --packages-up-to svo_ros --cmake-args -DCMAKE_BUILD_TYPE=Release`
- Source the workspace: `source install/setup.bash`
- The demo bag exists at: `datasheet/airground_rig_s3_ros2/airground_rig_s3_ros2.db3`
- RViz2 config file: `src/asr_sdm_universe/perception/asr_sdm_video_inertial_odometry/asr_sdm_svo/svo_ros/rviz_config.rviz`

Recommended: use three terminals and source the workspace in each
```sh
source install/setup.bash
```

Terminal 1 — start all nodes (camera + IMU + VIO) in one launch
```sh
# Camera node: subscribes bag /camera/image_raw, publishes /sensing/camera/realsense/color/*
ros2 launch svo_ros camera.launch.py

# IMU node: subscribes bag /imu/data, publishes raw + filtered
ros2 launch svo_ros imu.launch.py imu_topic_in:=/imu/data

# VIO node: subscribes camera + IMU filtered
ros2 launch svo_ros test_rig3.launch.py cam_topic:=/sensing/camera/realsense/color/image_raw imu_topic:=/sensing/imu/imu_filtered fast_type:=12

# Or use the combined launch (includes camera + IMU + VIO + RViz)
ros2 launch svo_ros test_rig3.launch.py rviz:=true fast_type:=12

# 仅纯视觉（不使用 IMU）
ros2 launch svo_ros test_rig3.launch.py use_imu:=false fast_type:=12

# VIO 方案（IMU prior λ）
# 方案A（推荐）：仅 SparseImgAlign 用 IMU prior
ros2 launch svo_ros test_rig3.launch.py use_imu:=true \
  img_align_prior_lambda_rot:=0.5 pose_optim_prior_lambda_rot:=0.0 fast_type:=12

# 方案B：SparseImgAlign + PoseOptimizer 全部用 IMU prior
ros2 launch svo_ros test_rig3.launch.py use_imu:=true \
  img_align_prior_lambda_rot:=0.5 pose_optim_prior_lambda_rot:=0.5 fast_type:=12

# EuRoC
ros2 launch svo_ros test_euroc.launch.py
ros2 launch svo_ros vio_euroc.launch.py
```

Terminal 2 — play rosbag (topics match bag defaults)
```sh
ros2 bag play datasheet/airground_rig_s3_ros2
ros2 bag play ~/svo/asr_sdm_ws/datasheet/MH_01_easy_ros2 --clock --rate 1.0
```

Terminal 3 — start RViz2 with the provided config and use simulated time
```sh
rviz2 -d src/asr_sdm_universe/perception/asr_sdm_video_inertial_odometry/asr_sdm_svo/svo_ros/rviz_config.rviz 
```

What you should see
- Nodes: `/asr_sdm_camera_realsense_d405`, `/asr_sdm_imu_hiwonder_10axis`, `/asr_sdm_video_inertial_odometry`, `/rosbag2_player`
- Topics: `/camera/image_raw`, `/camera/camera_info`, `/imu/data`, `/sensing/camera/realsense/color/image_raw`, `/sensing/camera/realsense/color/camera_info`, `/sensing/imu/imu_raw`, `/sensing/imu/imu_filtered`, `/localization/video_inertial_odom/image`, `/localization/video_inertial_odom/keyframes`, `/localization/video_inertial_odom/points`, `/localization/video_inertial_odom/pose`, `/localization/video_inertial_odom/trajectory`, `/tf`, `/clock`

Quick checks
```sh
# list nodes
ros2 node list

# list topics
ros2 topic list | sort

# Camera node info
ros2 node info /asr_sdm_camera_realsense_d405

# IMU node info (publishers)
ros2 node info /asr_sdm_imu_hiwonder_10axis

# check camera publishing
ros2 topic hz /sensing/camera/realsense/color/image_raw

# check IMU publishing
ros2 topic hz /sensing/imu/imu_raw
ros2 topic hz /sensing/imu/imu_filtered

# SVO node info (publishers/subscribers)
ros2 node info /asr_sdm_video_inertial_odometry

# check VIO publishing rates
ros2 topic hz /localization/video_inertial_odom/image
ros2 topic hz /localization/video_inertial_odom/pose
```

Stop processes
```sh
# stop camera node
pkill -f "ros2 launch svo_ros camera.launch.py"

# stop IMU node
pkill -f "ros2 launch svo_ros imu.launch.py"

# stop VIO launch
pkill -f "ros2 launch svo_ros test_rig3.launch.py"

# stop rosbag player
pkill -f "ros2 bag play"

# stop RViz2
pkill -f "rviz2 -d .*rviz_config.rviz"
```

Notes
- The camera node (`asr_sdm_camera_realsense_d405`) subscribes to bag topics (`/camera/image_raw`, `/camera/camera_info`) and republishes to the standardized sensing namespace (`/sensing/camera/realsense/color/*`). VIO node and downstream modules subscribe to the standardized topics.
- The IMU node (`asr_sdm_imu_hiwonder_10axis`) subscribes to `/imu/data` and republishes to `/sensing/imu/imu_raw` (raw pass-through) and `/sensing/imu/imu_filtered` (filtered output, ready for VIO integration).
- The VIO node subscribes to `/sensing/camera/realsense/color/image_raw` and `/sensing/imu/imu_filtered`. Since the combined launch starts both camera and IMU nodes automatically, the data pipeline is wired up correctly with no manual remapping needed.
- If you prefer a single combined launch that starts everything together, use `test_rig3.launch.py` directly (it includes camera + IMU + VIO + RViz).
- The rosbag can be played with `--clock` for simulated time. RViz2 is started with `use_sim_time:=true` so the timeline follows the bag.
