#!/usr/bin/env python3
"""ROS 2 launch file for the Fast-Planner topological replanning demo.

Port of topo_replan.launch / topo_algorithm.xml. Starts planning_manager_node (in
topological replanning mode) and traj_server. The external ROS 1 simulator stack
is not launched here; provide your own odometry / sensor sources.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_size_x = 40.0
    map_size_y = 20.0
    map_size_z = 5.0

    cx = 321.04638671875
    cy = 243.44969177246094
    fx = 387.229248046875
    fy = 387.229248046875

    max_vel = 3.0
    max_acc = 2.0

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic", default_value="/state_ukf/odom",
        description="Topic of your odometry such as VIO or LIO")
    odom_topic = LaunchConfiguration("odom_topic")

    planner_params = {
        "fsm.flight_type": 1,
        "fsm.thresh_replan": 0.5,
        "fsm.thresh_no_replan": 2.0,
        "fsm.waypoint_num": 2,
        "fsm.waypoint0_x": 19.0,
        "fsm.waypoint0_y": 0.0,
        "fsm.waypoint0_z": 1.0,
        "fsm.waypoint1_x": -19.0,
        "fsm.waypoint1_y": 0.0,
        "fsm.waypoint1_z": 1.0,
        "fsm.waypoint2_x": 0.0,
        "fsm.waypoint2_y": 19.0,
        "fsm.waypoint2_z": 1.0,
        "fsm.act_map": False,

        "sdf_map.resolution": 0.1,
        "sdf_map.map_size_x": map_size_x,
        "sdf_map.map_size_y": map_size_y,
        "sdf_map.map_size_z": map_size_z,
        "sdf_map.local_update_range_x": 5.5,
        "sdf_map.local_update_range_y": 5.5,
        "sdf_map.local_update_range_z": 4.5,
        "sdf_map.obstacles_inflation": 0.099,
        "sdf_map.local_bound_inflate": 0.5,
        "sdf_map.local_map_margin": 50,
        "sdf_map.ground_height": -1.0,
        "sdf_map.cx": cx,
        "sdf_map.cy": cy,
        "sdf_map.fx": fx,
        "sdf_map.fy": fy,
        "sdf_map.use_depth_filter": True,
        "sdf_map.depth_filter_tolerance": 0.15,
        "sdf_map.depth_filter_maxdist": 4.5,
        "sdf_map.depth_filter_mindist": 0.2,
        "sdf_map.depth_filter_margin": 2,
        "sdf_map.k_depth_scaling_factor": 1000.0,
        "sdf_map.skip_pixel": 3,
        "sdf_map.p_hit": 0.65,
        "sdf_map.p_miss": 0.35,
        "sdf_map.p_min": 0.12,
        "sdf_map.p_max": 0.90,
        "sdf_map.p_occ": 0.80,
        "sdf_map.min_ray_length": 0.5,
        "sdf_map.max_ray_length": 4.5,
        "sdf_map.esdf_slice_height": 0.3,
        "sdf_map.visualization_truncate_height": 2.5,
        "sdf_map.virtual_ceil_height": 3.0,
        "sdf_map.show_occ_time": False,
        "sdf_map.show_esdf_time": False,
        "sdf_map.pose_type": 1,
        "sdf_map.frame_id": "world",

        "manager.max_vel": max_vel,
        "manager.max_acc": max_acc,
        "manager.max_jerk": 4.0,
        "manager.dynamic_environment": 0,
        "manager.local_segment_length": 7.0,
        "manager.clearance_threshold": 0.2,
        "manager.control_points_distance": 0.3,
        "manager.use_geometric_path": False,
        "manager.use_topo_path": True,
        "manager.use_optimization": True,

        "topo_prm.sample_inflate_x": 1.0,
        "topo_prm.sample_inflate_y": 3.5,
        "topo_prm.sample_inflate_z": 1.0,
        "topo_prm.clearance": 0.3,
        "topo_prm.max_sample_time": 0.005,
        "topo_prm.max_sample_num": 2000,
        "topo_prm.max_raw_path": 300,
        "topo_prm.max_raw_path2": 25,
        "topo_prm.short_cut_num": 1,
        "topo_prm.reserve_num": 6,
        "topo_prm.ratio_to_short": 5.5,
        "topo_prm.parallel_shortcut": True,

        "optimization.lambda1": 10.0,
        "optimization.lambda2": 5.0,
        "optimization.lambda3": 0.0,
        "optimization.lambda4": 0.001,
        "optimization.lambda5": 1.5,
        "optimization.lambda6": 10.0,
        "optimization.lambda7": 20.0,
        "optimization.dist0": 0.4,
        "optimization.dist1": 0.0,
        "optimization.max_vel": max_vel,
        "optimization.max_acc": max_acc,
        "optimization.algorithm1": 15,
        "optimization.algorithm2": 11,
        "optimization.max_iteration_num1": 2,
        "optimization.max_iteration_num2": 300,
        "optimization.max_iteration_time1": 0.0001,
        "optimization.max_iteration_time2": 0.005,
        "optimization.order": 3,

        "bspline.limit_vel": max_vel,
        "bspline.limit_acc": max_acc,
        "bspline.limit_ratio": 1.1,

        "heading_planner.yaw_diff": 5 * 3.1415926 / 180.0,
        "heading_planner.half_vert_num": 3,
        "heading_planner.lambda1": 2.0,
        "heading_planner.lambda2": 1.0,
        "heading_planner.max_yaw_rate": 60 * 3.1415926 / 180.0,
        "heading_planner.w": 10.0,
    }

    planning_manager_node = Node(
        package="asr_sdm_planning_manager",
        executable="planning_manager_node",
        name="planning_manager_node",
        output="screen",
        parameters=[planner_params],
        remappings=[
            ("/odom_world", odom_topic),
            ("/sdf_map/odom", odom_topic),
            ("/sdf_map/cloud", "/pcl_render_node/cloud"),
            ("/sdf_map/pose", "/pcl_render_node/camera_pose"),
            ("/sdf_map/depth", "/pcl_render_node/depth"),
        ],
    )

    traj_server = Node(
        package="asr_sdm_planning_manager",
        executable="traj_server",
        name="traj_server",
        output="screen",
        parameters=[{"traj_server.time_forward": 1.5}],
        remappings=[
            ("/position_cmd", "planning/pos_cmd"),
            ("/odom_world", odom_topic),
        ],
    )

    return LaunchDescription([
        odom_topic_arg,
        planning_manager_node,
        traj_server,
    ])
