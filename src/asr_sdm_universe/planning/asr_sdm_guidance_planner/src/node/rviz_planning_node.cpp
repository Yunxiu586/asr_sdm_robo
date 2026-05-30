#include <asr_sdm_guidance_planner/node/rviz_planning_node.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/color_rgba.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace asr_sdm_guidance_planner
{

RvizPlanningNode::RvizPlanningNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("astar_lbfgs_trajectory_planner", options)
{
  loadParameters();
  map_.reset(map_options_);

  astar_planner_.setOptions(Astar3dOptions{
    this->get_parameter("astar.heuristic_weight").as_double(),
    this->get_parameter("astar.extra_clearance").as_double(),
    static_cast<int>(this->get_parameter("astar.max_expansions").as_int()),
    static_cast<int>(this->get_parameter("astar.nearest_free_search_radius").as_int()),
    this->get_parameter("astar.allow_diagonal").as_bool()});

  optimizer_.setOptions(LbfgsPathOptimizerOptions{
    this->get_parameter("optimizer.enabled").as_bool(),
    static_cast<int>(this->get_parameter("optimizer.max_iterations").as_int()),
    static_cast<int>(this->get_parameter("optimizer.max_control_points").as_int()),
    this->get_parameter("optimizer.epsilon").as_double(),
    this->get_parameter("optimizer.smooth_weight").as_double(),
    this->get_parameter("optimizer.length_weight").as_double(),
    this->get_parameter("optimizer.obstacle_weight").as_double(),
    this->get_parameter("optimizer.guide_weight").as_double(),
    this->get_parameter("optimizer.safe_distance").as_double(),
    this->get_parameter("optimizer.validity_check_step").as_double(),
    this->get_parameter("optimizer.extra_clearance").as_double()});

  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    map_topic_, rclcpp::SensorDataQoS(),
    std::bind(&RvizPlanningNode::mapCallback, this, std::placeholders::_1));

  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    this->get_parameter("topics.clicked_point").as_string(), 10,
    std::bind(&RvizPlanningNode::clickedPointCallback, this, std::placeholders::_1));

  start_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    this->get_parameter("topics.start_point").as_string(), 10,
    std::bind(&RvizPlanningNode::startPointCallback, this, std::placeholders::_1));

  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    this->get_parameter("topics.goal_point").as_string(), 10,
    std::bind(&RvizPlanningNode::goalPointCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    this->get_parameter("topics.initialpose").as_string(), 10,
    std::bind(&RvizPlanningNode::initialPoseCallback, this, std::placeholders::_1));

  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    this->get_parameter("topics.goal_pose").as_string(), 10,
    std::bind(&RvizPlanningNode::goalPoseCallback, this, std::placeholders::_1));

  raw_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/astar_path", 10);
  trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/trajectory", 10);
  raw_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/astar_path_marker", 10);
  trajectory_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("/planning/trajectory_marker", 10);
  start_goal_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("/planning/start_goal_marker", 10);
  status_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planning/status_text", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "3D A* + L-BFGS planner started. Map topic: %s. Use RViz Publish Point twice: start then goal.",
    map_topic_.c_str());
}

void RvizPlanningNode::loadParameters()
{
  this->declare_parameter("frame_id", "world");
  this->declare_parameter("topics.map", "/esdf_map/occupancy_inflate");
  this->declare_parameter("topics.clicked_point", "/clicked_point");
  this->declare_parameter("topics.start_point", "/planning/start");
  this->declare_parameter("topics.goal_point", "/planning/goal");
  this->declare_parameter("topics.initialpose", "/initialpose");
  this->declare_parameter("topics.goal_pose", "/goal_pose");

  this->declare_parameter("map.resolution", 0.15);
  this->declare_parameter("map.origin_x", -10.0);
  this->declare_parameter("map.origin_y", -10.0);
  this->declare_parameter("map.origin_z", -1.0);
  this->declare_parameter("map.size_x", 20.0);
  this->declare_parameter("map.size_y", 20.0);
  this->declare_parameter("map.size_z", 5.0);
  this->declare_parameter("map.unknown_as_occupied", false);
  this->declare_parameter("map.clear_before_integrate", true);
  this->declare_parameter("map.auto_compute_esdf_on_map", true);
  this->declare_parameter("map.skip_repeated_map_with_same_point_count", true);

  this->declare_parameter("selection.clicked_point_use_msg_z", true);
  this->declare_parameter("selection.default_planning_z", 0.5);
  this->declare_parameter("selection.use_optimized_only_if_safe", true);

  this->declare_parameter("astar.heuristic_weight", 1.0);
  this->declare_parameter("astar.extra_clearance", 0.0);
  this->declare_parameter("astar.max_expansions", 300000);
  this->declare_parameter("astar.nearest_free_search_radius", 10);
  this->declare_parameter("astar.allow_diagonal", true);

  this->declare_parameter("optimizer.enabled", true);
  this->declare_parameter("optimizer.max_iterations", 120);
  this->declare_parameter("optimizer.max_control_points", 80);
  this->declare_parameter("optimizer.epsilon", 1.0e-4);
  this->declare_parameter("optimizer.smooth_weight", 1.0);
  this->declare_parameter("optimizer.length_weight", 0.05);
  this->declare_parameter("optimizer.obstacle_weight", 12.0);
  this->declare_parameter("optimizer.guide_weight", 0.08);
  this->declare_parameter("optimizer.safe_distance", 0.20);
  this->declare_parameter("optimizer.validity_check_step", 0.05);
  this->declare_parameter("optimizer.extra_clearance", 0.0);

  frame_id_ = this->get_parameter("frame_id").as_string();
  map_topic_ = this->get_parameter("topics.map").as_string();
  clicked_point_use_msg_z_ = this->get_parameter("selection.clicked_point_use_msg_z").as_bool();
  default_planning_z_ = this->get_parameter("selection.default_planning_z").as_double();
  auto_compute_esdf_on_map_ = this->get_parameter("map.auto_compute_esdf_on_map").as_bool();
  use_optimized_only_if_safe_ = this->get_parameter("selection.use_optimized_only_if_safe").as_bool();
  skip_repeated_map_with_same_point_count_ =
    this->get_parameter("map.skip_repeated_map_with_same_point_count").as_bool();

  map_options_.resolution = this->get_parameter("map.resolution").as_double();
  map_options_.origin = Eigen::Vector3d(
    this->get_parameter("map.origin_x").as_double(),
    this->get_parameter("map.origin_y").as_double(),
    this->get_parameter("map.origin_z").as_double());
  map_options_.size = Eigen::Vector3d(
    this->get_parameter("map.size_x").as_double(),
    this->get_parameter("map.size_y").as_double(),
    this->get_parameter("map.size_z").as_double());
  map_options_.unknown_as_occupied = this->get_parameter("map.unknown_as_occupied").as_bool();
  map_options_.clear_before_integrate = this->get_parameter("map.clear_before_integrate").as_bool();
}

void RvizPlanningNode::mapCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  if (
    skip_repeated_map_with_same_point_count_ && map_.isReady() &&
    static_cast<int>(cloud.points.size()) == last_map_point_count_)
  {
    return;
  }

  last_map_point_count_ = static_cast<int>(cloud.points.size());
  map_.integrateOccupiedCloud(cloud);

  if (auto_compute_esdf_on_map_) {
    map_.computeEsdf();
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Map updated: input_points=%d, occupied_voxels=%d, voxels=%d, esdf=%s",
    map_.inputPointCount(), map_.occupiedCount(), map_.voxelCount(), map_.hasEsdf() ? "ready" : "not_ready");
}

void RvizPlanningNode::clickedPointCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
{
  Eigen::Vector3d point = pointMsgToEigen(msg->point);
  if (!clicked_point_use_msg_z_) {
    point.z() = default_planning_z_;
  }

  if (!waiting_for_goal_click_) {
    setStart(point);
    waiting_for_goal_click_ = true;
    publishStatusText("Start selected. Click goal point.");
    RCLCPP_INFO(this->get_logger(), "Start selected: [%.3f, %.3f, %.3f]", point.x(), point.y(), point.z());
    return;
  }

  waiting_for_goal_click_ = false;
  setGoalAndPlan(point);
}

void RvizPlanningNode::startPointCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
{
  Eigen::Vector3d point = pointMsgToEigen(msg->point);
  setStart(point);
  waiting_for_goal_click_ = true;
}

void RvizPlanningNode::goalPointCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
{
  Eigen::Vector3d point = pointMsgToEigen(msg->point);
  setGoalAndPlan(point);
}

void RvizPlanningNode::initialPoseCallback(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  Eigen::Vector3d point = poseMsgToEigen(msg->pose.pose);
  point.z() = default_planning_z_;
  setStart(point);
  waiting_for_goal_click_ = true;
}

void RvizPlanningNode::goalPoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  Eigen::Vector3d point = poseMsgToEigen(msg->pose);
  point.z() = default_planning_z_;
  setGoalAndPlan(point);
}

void RvizPlanningNode::setStart(const Eigen::Vector3d & start)
{
  start_ = start;
  have_start_ = true;
  publishStartGoalMarker();
}

void RvizPlanningNode::setGoalAndPlan(const Eigen::Vector3d & goal)
{
  goal_ = goal;
  have_goal_ = true;
  publishStartGoalMarker();

  if (!have_start_) {
    RCLCPP_WARN(this->get_logger(), "Goal selected but start is not set. Select start first.");
    publishStatusText("Goal selected, but start is not set.");
    return;
  }

  runPlanning();
}

void RvizPlanningNode::runPlanning()
{
  if (!map_.isReady()) {
    RCLCPP_WARN(this->get_logger(), "Map is not ready. Wait for /esdf_map/occupancy_inflate first.");
    publishStatusText("Map is not ready.");
    return;
  }

  const auto cycle_start_time = std::chrono::steady_clock::now();
  double esdf_time_ms = 0.0;
  double astar_time_ms = 0.0;
  double optimizer_time_ms = 0.0;

  if (!map_.hasEsdf() && auto_compute_esdf_on_map_) {
    const auto esdf_start_time = std::chrono::steady_clock::now();
    map_.computeEsdf();
    const auto esdf_end_time = std::chrono::steady_clock::now();
    esdf_time_ms = std::chrono::duration<double, std::milli>(esdf_end_time - esdf_start_time).count();
  }

  RCLCPP_INFO(
    this->get_logger(), "Planning from [%.2f %.2f %.2f] to [%.2f %.2f %.2f]",
    start_.x(), start_.y(), start_.z(), goal_.x(), goal_.y(), goal_.z());

  const auto astar_start_time = std::chrono::steady_clock::now();
  PlanResult plan = astar_planner_.plan(map_, start_, goal_);
  const auto astar_end_time = std::chrono::steady_clock::now();
  astar_time_ms = std::chrono::duration<double, std::milli>(astar_end_time - astar_start_time).count();

  if (!plan.success) {
    const auto cycle_end_time = std::chrono::steady_clock::now();
    const double total_time_ms =
      std::chrono::duration<double, std::milli>(cycle_end_time - cycle_start_time).count();

    std::ostringstream timing;
    timing << "Guidance planner cycle time: total=" << total_time_ms << " ms"
           << ", esdf=" << esdf_time_ms << " ms"
           << ", astar=" << astar_time_ms << " ms"
           << ", optimizer=0 ms";

    RCLCPP_WARN(this->get_logger(), "%s", plan.message.c_str());
    RCLCPP_INFO(this->get_logger(), "%s", timing.str().c_str());
    publishStatusText(plan.message + " | " + timing.str());
    return;
  }

  publishPath(plan.raw_path, raw_path_pub_);
  publishLineMarker(plan.raw_path, raw_marker_pub_, 0, 0.05, 0.1f, 0.4f, 1.0f, 0.95f);

  const auto optimizer_start_time = std::chrono::steady_clock::now();
  OptimizerResult opt = optimizer_.optimize(plan.raw_path, map_);
  const auto optimizer_end_time = std::chrono::steady_clock::now();
  optimizer_time_ms =
    std::chrono::duration<double, std::milli>(optimizer_end_time - optimizer_start_time).count();

  const auto cycle_end_time = std::chrono::steady_clock::now();
  const double total_time_ms =
    std::chrono::duration<double, std::milli>(cycle_end_time - cycle_start_time).count();

  std::vector<Eigen::Vector3d> final_path = opt.path;
  bool use_raw_fallback = false;
  if (!opt.success || (use_optimized_only_if_safe_ && !opt.path_safe)) {
    use_raw_fallback = true;
    final_path = plan.raw_path;
  }

  publishPath(final_path, trajectory_pub_);
  publishLineMarker(final_path, trajectory_marker_pub_, 1, 0.08, 1.0f, 0.15f, 0.05f, 1.0f);

  std::ostringstream timing;
  timing << "Guidance planner cycle time: total=" << total_time_ms << " ms"
         << ", esdf=" << esdf_time_ms << " ms"
         << ", astar=" << astar_time_ms << " ms"
         << ", optimizer=" << optimizer_time_ms << " ms";

  std::ostringstream status;
  status << plan.message << " | " << opt.message << " | " << timing.str();
  if (use_raw_fallback) {
    status << " | fallback: raw A* path published as trajectory";
  }

  publishStatusText(status.str());
  RCLCPP_INFO(this->get_logger(), "%s", status.str().c_str());
}

Eigen::Vector3d RvizPlanningNode::pointMsgToEigen(const geometry_msgs::msg::Point & point) const
{
  Eigen::Vector3d out(point.x, point.y, point.z);
  if (!clicked_point_use_msg_z_) {
    out.z() = default_planning_z_;
  }
  return out;
}

Eigen::Vector3d RvizPlanningNode::poseMsgToEigen(const geometry_msgs::msg::Pose & pose) const
{
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

void RvizPlanningNode::publishPath(
  const std::vector<Eigen::Vector3d> & path,
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & pub) const
{
  nav_msgs::msg::Path msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = this->now();
  msg.poses.reserve(path.size());

  for (const auto & point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }

  pub->publish(msg);
}

void RvizPlanningNode::publishLineMarker(
  const std::vector<Eigen::Vector3d> & path,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub,
  int id,
  double width,
  float r,
  float g,
  float b,
  float a) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "planning_trajectory";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.points.reserve(path.size());

  for (const auto & point : path) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    marker.points.push_back(p);
  }

  pub->publish(marker);
}

void RvizPlanningNode::publishStartGoalMarker() const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "planning_start_goal";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  if (have_start_) {
    geometry_msgs::msg::Point p;
    p.x = start_.x();
    p.y = start_.y();
    p.z = start_.z();
    marker.points.push_back(p);

    std_msgs::msg::ColorRGBA c;
    c.r = 0.0f;
    c.g = 1.0f;
    c.b = 0.0f;
    c.a = 1.0f;
    marker.colors.push_back(c);
  }

  if (have_goal_) {
    geometry_msgs::msg::Point p;
    p.x = goal_.x();
    p.y = goal_.y();
    p.z = goal_.z();
    marker.points.push_back(p);

    std_msgs::msg::ColorRGBA c;
    c.r = 1.0f;
    c.g = 0.0f;
    c.b = 0.0f;
    c.a = 1.0f;
    marker.colors.push_back(c);
  }

  start_goal_marker_pub_->publish(marker);
}

void RvizPlanningNode::publishStatusText(const std::string & text) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = this->now();
  marker.ns = "planning_status";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = start_.x();
  marker.pose.position.y = start_.y();
  marker.pose.position.z = start_.z() + 0.8;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.25;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.text = text;
  status_marker_pub_->publish(marker);
}

}  // namespace asr_sdm_guidance_planner
