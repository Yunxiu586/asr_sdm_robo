# asr_sdm_guidance_planner

ROS 2 Jazzy package for fixed-map 3D trajectory planning.

The package is intentionally independent from `asr_sdm_esdf_map`. It reads the fixed inflated occupancy cloud from

```sh
/esdf_map/occupancy_inflate
```

then builds a planning-side 3D voxel map and ESDF. This avoids changing the ESDF mapping package and keeps the planner backend replaceable.

## Pipeline

```text
/esdf_map/occupancy_inflate
        ↓
planning-side VoxelEsdfMap
        ↓
3D A*
        ↓
L-BFGS path smoothing with ESDF obstacle cost
        ↓
/planning/astar_path
/planning/trajectory
/planning/astar_path_marker
/planning/trajectory_marker
```

## RViz interaction

Use RViz `Publish Point` twice:

```text
first click  -> start
second click -> goal and plan
third click  -> new start
fourth click -> new goal and replan
```

Alternative topic interfaces:

```text
/planning/start   geometry_msgs/msg/PointStamped
/planning/goal    geometry_msgs/msg/PointStamped
/initialpose      geometry_msgs/msg/PoseWithCovarianceStamped
/goal_pose        geometry_msgs/msg/PoseStamped
```

For `2D Pose Estimate` and `2D Goal Pose`, the z value is set by

```yaml
selection.default_planning_z: 0.5
```

## Build

Put both packages under your workspace `src` directory:

```sh
asr_sdm_lbfgs_solver
asr_sdm_guidance_planner
```

Then build:

```sh
cd /home/yunxiu/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to asr_sdm_guidance_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --symlink-install
source install/setup.bash
```

## Run with the fixed ESDF map

First run the ESDF node with the final fixed map enabled:

```yaml
esdf_map.final_map_enable: true
```

Play the bag once, without `--loop`:

```sh
ros2 bag play /home/yunxiu/vo_esdf_inputs --rate 1.0
```

Wait until the ESDF node reports that the final fixed map is ready. Then start the planner:

```sh
ros2 launch asr_sdm_guidance_planner astar_lbfgs_planner_launch.py
```

In RViz2, set `Fixed Frame = world`, then add:

```text
/esdf_map/occupied_map_3d        Marker
/planning/astar_path_marker      Marker
/planning/trajectory_marker      Marker
/planning/start_goal_marker      Marker
/planning/astar_path             Path
/planning/trajectory             Path
```

