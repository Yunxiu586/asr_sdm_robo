#include <bspline/bspline_path_generator.h>

#include <Eigen/Core>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace bspline_test
{

struct VirtualObstacle
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.0;
};

class BsplineTestNode final : public rclcpp::Node
{
public:
  explicit BsplineTestNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("bspline_test_node", options)
  {
    loadParameters();

    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.reliable();
    latched_qos.transient_local();

    candidate_paths_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      input_candidates_marker_topic_, latched_qos,
      std::bind(&BsplineTestNode::candidatePathsCallback, this, std::placeholders::_1));

    if (enable_virtual_obstacle_check_) {
      virtual_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        virtual_obstacles_marker_topic_, latched_qos,
        std::bind(&BsplineTestNode::virtualObstaclesCallback, this, std::placeholders::_1));
    }

    bspline_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(bspline_path_topic_, latched_qos);
    bspline_candidate_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      bspline_candidate_marker_topic_, latched_qos);
    ctrl_points_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      ctrl_points_topic_, latched_qos);
    ctrl_points_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      ctrl_points_marker_topic_, latched_qos);
    collision_samples_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      collision_samples_marker_topic_, latched_qos);

    RCLCPP_INFO(
      this->get_logger(),
      "Bspline test node started. Input topo candidates: %s, candidate B-splines: %s, ctrl points data: %s, "
      "ctrl point markers: %s.",
      input_candidates_marker_topic_.c_str(), bspline_candidate_marker_topic_.c_str(),
      ctrl_points_topic_.c_str(), ctrl_points_marker_topic_.c_str());
  }

private:
  void loadParameters()
  {
    this->declare_parameter("frame_id", "world");
    this->declare_parameter("topics.input_candidates_marker", "/planning/topo_candidate_paths");
    this->declare_parameter("topics.bspline_path", "/planning/bspline_path");
    this->declare_parameter("topics.bspline_candidate_paths_marker", "/planning/bspline_candidate_paths");
    this->declare_parameter("topics.ctrl_points", "/planning/bspline_ctrl_points");
    this->declare_parameter("topics.ctrl_points_marker", "/planning/bspline_ctrl_points_marker");
    this->declare_parameter("topics.virtual_obstacles_marker", "/planning/virtual_obstacles");
    this->declare_parameter("topics.collision_samples_marker", "/planning/bspline_collision_samples");

    this->declare_parameter("bspline.order", 3);
    this->declare_parameter("bspline.ts", 0.20);
    this->declare_parameter("bspline.sample_dt", 0.02);
    this->declare_parameter("bspline.min_input_point_spacing", 1.0e-3);
    this->declare_parameter("bspline.max_reallocation_iter", 8);
    this->declare_parameter("bspline.fallback_to_input_on_failure", true);

    this->declare_parameter("physical_limits.max_vel", 2.0);
    this->declare_parameter("physical_limits.max_acc", 3.0);

    this->declare_parameter("collision.enable_virtual_obstacle_check", true);
    this->declare_parameter("collision.clearance", 0.03);
    this->declare_parameter("collision.default_virtual_obstacle_radius", 0.35);
    this->declare_parameter("collision.fallback_to_input_on_collision", true);

    frame_id_ = this->get_parameter("frame_id").as_string();
    input_candidates_marker_topic_ = this->get_parameter("topics.input_candidates_marker").as_string();
    bspline_path_topic_ = this->get_parameter("topics.bspline_path").as_string();
    bspline_candidate_marker_topic_ = this->get_parameter("topics.bspline_candidate_paths_marker").as_string();
    ctrl_points_topic_ = this->get_parameter("topics.ctrl_points").as_string();
    ctrl_points_marker_topic_ = this->get_parameter("topics.ctrl_points_marker").as_string();
    virtual_obstacles_marker_topic_ = this->get_parameter("topics.virtual_obstacles_marker").as_string();
    collision_samples_marker_topic_ = this->get_parameter("topics.collision_samples_marker").as_string();

    bspline::BsplinePathGeneratorOptions generator_options;
    generator_options.order = static_cast<int>(this->get_parameter("bspline.order").as_int());
    generator_options.ts = this->get_parameter("bspline.ts").as_double();
    generator_options.sample_dt = this->get_parameter("bspline.sample_dt").as_double();
    generator_options.min_input_point_spacing = this->get_parameter(
      "bspline.min_input_point_spacing").as_double();
    generator_options.max_reallocation_iter = static_cast<int>(
      this->get_parameter("bspline.max_reallocation_iter").as_int());
    generator_options.fallback_to_input_on_failure = this->get_parameter(
      "bspline.fallback_to_input_on_failure").as_bool();
    generator_options.max_vel = this->get_parameter("physical_limits.max_vel").as_double();
    generator_options.max_acc = this->get_parameter("physical_limits.max_acc").as_double();
    generator_.setOptions(generator_options);

    enable_virtual_obstacle_check_ = this->get_parameter(
      "collision.enable_virtual_obstacle_check").as_bool();
    collision_clearance_ = this->get_parameter("collision.clearance").as_double();
    default_virtual_obstacle_radius_ = this->get_parameter(
      "collision.default_virtual_obstacle_radius").as_double();
    fallback_to_input_on_collision_ = this->get_parameter(
      "collision.fallback_to_input_on_collision").as_bool();
  }

  void candidatePathsCallback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
  {
    latest_candidate_paths_.clear();
    for (const auto & marker : msg->markers) {
      if (marker.action != visualization_msgs::msg::Marker::ADD ||
        marker.type != visualization_msgs::msg::Marker::LINE_STRIP ||
        marker.points.size() < 2U)
      {
        continue;
      }

      if (!marker.header.frame_id.empty()) {
        frame_id_ = marker.header.frame_id;
      }

      std::vector<Eigen::Vector3d> path;
      path.reserve(marker.points.size());
      for (const auto & p : marker.points) {
        path.push_back(Eigen::Vector3d(p.x, p.y, p.z));
      }
      if (path.size() >= 2U) {
        latest_candidate_paths_.push_back(path);
      }
    }

    have_candidate_paths_ = !latest_candidate_paths_.empty();
    if (!have_candidate_paths_) {
      RCLCPP_WARN(this->get_logger(), "Received topo candidate MarkerArray, but no valid candidate path was found.");
      publishDeleteMarkers();
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Received %zu topo candidate paths. Generating one B-spline per candidate.",
      latest_candidate_paths_.size());
    computeAndPublish();
  }

  void virtualObstaclesCallback(visualization_msgs::msg::Marker::ConstSharedPtr msg)
  {
    if (msg->action == visualization_msgs::msg::Marker::DELETE ||
      msg->action == visualization_msgs::msg::Marker::DELETEALL)
    {
      virtual_obstacles_.clear();
      if (have_candidate_paths_) {
        computeAndPublish();
      }
      return;
    }

    if (msg->type != visualization_msgs::msg::Marker::SPHERE_LIST) {
      return;
    }

    if (!msg->header.frame_id.empty()) {
      frame_id_ = msg->header.frame_id;
    }

    const double scale_radius = 0.5 * std::max({msg->scale.x, msg->scale.y, msg->scale.z});
    const double radius = scale_radius > 1.0e-6 ? scale_radius : default_virtual_obstacle_radius_;

    virtual_obstacles_.clear();
    virtual_obstacles_.reserve(msg->points.size());
    for (const auto & p : msg->points) {
      VirtualObstacle obstacle;
      obstacle.center = Eigen::Vector3d(p.x, p.y, p.z);
      obstacle.radius = radius;
      virtual_obstacles_.push_back(obstacle);
    }

    RCLCPP_INFO(
      this->get_logger(), "Updated virtual obstacle cache from marker: %zu obstacles, radius=%.3f.",
      virtual_obstacles_.size(), radius);

    if (have_candidate_paths_) {
      computeAndPublish();
    }
  }

  void computeAndPublish()
  {
    if (!have_candidate_paths_) {
      return;
    }

    std::vector<bspline::BsplinePathResult> results;
    results.reserve(latest_candidate_paths_.size());
    std::vector<Eigen::Vector3d> all_collision_samples;

    for (std::size_t i = 0; i < latest_candidate_paths_.size(); ++i) {
      bspline::BsplinePathResult result = generator_.generate(latest_candidate_paths_[i]);
      if (!result.success) {
        RCLCPP_WARN(
          this->get_logger(), "Candidate %zu B-spline generation failed: %s.", i, result.message.c_str());
        continue;
      }

      std::vector<Eigen::Vector3d> collision_samples;
      const bool collision_free = checkCollision(result.samples, collision_samples);
      if (!collision_free) {
        all_collision_samples.insert(
          all_collision_samples.end(), collision_samples.begin(), collision_samples.end());
        RCLCPP_WARN(
          this->get_logger(), "Candidate %zu B-spline collides with virtual obstacles: %zu samples.",
          i, collision_samples.size());
        if (fallback_to_input_on_collision_) {
          result.samples = result.input_path;
          result.message = "collision fallback to topo candidate";
        }
      }

      results.push_back(std::move(result));
    }

    if (results.empty()) {
      publishDeleteMarkers();
      RCLCPP_WARN(this->get_logger(), "No valid B-spline candidate was generated.");
      return;
    }

    publishPath(results.front().samples);
    publishCandidateMarkers(results);
    publishControlPointData(results);
    publishControlPointMarkers(results);
    publishCollisionSamples(all_collision_samples);

    RCLCPP_INFO(
      this->get_logger(),
      "Published %zu B-spline candidates. First candidate: input=%zu, ctrl=%ld, samples=%zu, duration=%.3f s, "
      "realloc_iter=%d, feasible=%s, mean/max vel=%.3f/%.3f, mean/max acc=%.3f/%.3f.",
      results.size(), results.front().input_path.size(), results.front().control_points.rows(),
      results.front().samples.size(), results.front().duration, results.front().reallocation_iter,
      results.front().feasible ? "true" : "false", results.front().mean_vel, results.front().max_vel,
      results.front().mean_acc, results.front().max_acc);
  }

  bool checkCollision(
    const std::vector<Eigen::Vector3d> & samples,
    std::vector<Eigen::Vector3d> & collision_samples) const
  {
    collision_samples.clear();
    if (!enable_virtual_obstacle_check_ || virtual_obstacles_.empty()) {
      return true;
    }

    for (const auto & sample : samples) {
      for (const auto & obstacle : virtual_obstacles_) {
        const double min_dist = obstacle.radius + collision_clearance_;
        if ((sample - obstacle.center).norm() <= min_dist) {
          collision_samples.push_back(sample);
          break;
        }
      }
    }

    return collision_samples.empty();
  }

  void publishPath(const std::vector<Eigen::Vector3d> & path) const
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
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

    bspline_path_pub_->publish(msg);
  }

  void publishCandidateMarkers(const std::vector<bspline::BsplinePathResult> & results) const
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = this->now();

    for (int id = 0; id < last_bspline_candidate_marker_count_; ++id) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id_;
      marker.ns = "bspline_candidate_paths";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(marker);
    }

    int id = 0;
    for (const auto & result : results) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id_;
      marker.ns = "bspline_candidate_paths";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.055;
      marker.color.r = 0.0f;
      marker.color.g = 0.75f;
      marker.color.b = 1.0f;
      marker.color.a = 0.90f;
      marker.points.reserve(result.samples.size());

      for (const auto & point : result.samples) {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        marker.points.push_back(p);
      }
      array.markers.push_back(marker);
    }

    last_bspline_candidate_marker_count_ = id;
    bspline_candidate_marker_pub_->publish(array);
  }

  void publishControlPointData(const std::vector<bspline::BsplinePathResult> & results) const
  {
    std_msgs::msg::Float64MultiArray msg;

    // Flat ragged schema for optimizer test nodes:
    // [version, candidate_num,
    //   order, ts, rows, cols, duration, feasible, q00, q01, q02, q10, ...,
    //   order, ts, rows, cols, duration, feasible, ...]
    // Each candidate control point block can be converted directly to Eigen::MatrixXd(rows, 3).
    msg.data.reserve(2U);
    msg.data.push_back(1.0);  // schema version
    msg.data.push_back(0.0);  // valid candidate count, filled after packing

    std::size_t valid_candidate_count = 0U;
    for (const auto & result : results) {
      const Eigen::MatrixXd & ctrl_pts = result.control_points;
      if (ctrl_pts.rows() <= 0 || ctrl_pts.cols() < 3) {
        continue;
      }

      ++valid_candidate_count;
      msg.data.push_back(static_cast<double>(generator_.options().order));
      msg.data.push_back(generator_.options().ts);
      msg.data.push_back(static_cast<double>(ctrl_pts.rows()));
      msg.data.push_back(3.0);
      msg.data.push_back(result.duration);
      msg.data.push_back(result.feasible ? 1.0 : 0.0);

      for (int i = 0; i < ctrl_pts.rows(); ++i) {
        msg.data.push_back(ctrl_pts(i, 0));
        msg.data.push_back(ctrl_pts(i, 1));
        msg.data.push_back(ctrl_pts(i, 2));
      }
    }

    msg.data[1] = static_cast<double>(valid_candidate_count);

    std_msgs::msg::MultiArrayDimension dim;
    dim.label = "flat_bspline_ctrl_points";
    dim.size = static_cast<unsigned int>(msg.data.size());
    dim.stride = static_cast<unsigned int>(msg.data.size());
    msg.layout.dim.push_back(dim);
    msg.layout.data_offset = 0U;

    ctrl_points_pub_->publish(msg);
  }

  void publishControlPointMarkers(const std::vector<bspline::BsplinePathResult> & results) const
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = this->now();

    for (int id = 0; id < last_ctrl_marker_count_; ++id) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id_;
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      marker.ns = (id % 2 == 0) ? "bspline_ctrl_points" : "bspline_ctrl_polygon";
      array.markers.push_back(marker);
    }

    int id = 0;
    for (const auto & result : results) {
      const Eigen::MatrixXd & ctrl_pts = result.control_points;
      if (ctrl_pts.rows() <= 0 || ctrl_pts.cols() < 3) {
        continue;
      }

      visualization_msgs::msg::Marker spheres;
      spheres.header.stamp = stamp;
      spheres.header.frame_id = frame_id_;
      spheres.ns = "bspline_ctrl_points";
      spheres.id = id++;
      spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      spheres.action = visualization_msgs::msg::Marker::ADD;
      spheres.pose.orientation.w = 1.0;
      spheres.scale.x = 0.10;
      spheres.scale.y = 0.10;
      spheres.scale.z = 0.10;
      spheres.color.r = 0.95f;
      spheres.color.g = 0.35f;
      spheres.color.b = 1.0f;
      spheres.color.a = 0.80f;
      spheres.points.reserve(static_cast<std::size_t>(ctrl_pts.rows()));

      visualization_msgs::msg::Marker line;
      line.header.stamp = stamp;
      line.header.frame_id = frame_id_;
      line.ns = "bspline_ctrl_polygon";
      line.id = id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.020;
      line.color.r = 0.95f;
      line.color.g = 0.35f;
      line.color.b = 1.0f;
      line.color.a = 0.45f;
      line.points.reserve(static_cast<std::size_t>(ctrl_pts.rows()));

      for (int i = 0; i < ctrl_pts.rows(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = ctrl_pts(i, 0);
        p.y = ctrl_pts(i, 1);
        p.z = ctrl_pts(i, 2);
        spheres.points.push_back(p);
        line.points.push_back(p);
      }

      array.markers.push_back(spheres);
      array.markers.push_back(line);
    }

    last_ctrl_marker_count_ = id;
    ctrl_points_marker_pub_->publish(array);
  }

  void publishCollisionSamples(const std::vector<Eigen::Vector3d> & collision_samples) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = frame_id_;
    marker.ns = "bspline_collision_samples";
    marker.id = 0;

    if (collision_samples.empty()) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      collision_samples_marker_pub_->publish(marker);
      return;
    }

    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.16;
    marker.scale.y = 0.16;
    marker.scale.z = 0.16;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.points.reserve(collision_samples.size());

    for (const auto & sample : collision_samples) {
      geometry_msgs::msg::Point p;
      p.x = sample.x();
      p.y = sample.y();
      p.z = sample.z();
      marker.points.push_back(p);
    }

    collision_samples_marker_pub_->publish(marker);
  }

  void publishDeleteMarkers() const
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = this->now();

    for (int id = 0; id < last_bspline_candidate_marker_count_; ++id) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id_;
      marker.ns = "bspline_candidate_paths";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(marker);
    }
    last_bspline_candidate_marker_count_ = 0;
    bspline_candidate_marker_pub_->publish(array);

    publishControlPointData({});
    publishControlPointMarkers({});
    publishCollisionSamples({});
  }

  std::string frame_id_ = "world";
  std::string input_candidates_marker_topic_ = "/planning/topo_candidate_paths";
  std::string bspline_path_topic_ = "/planning/bspline_path";
  std::string bspline_candidate_marker_topic_ = "/planning/bspline_candidate_paths";
  std::string ctrl_points_topic_ = "/planning/bspline_ctrl_points";
  std::string ctrl_points_marker_topic_ = "/planning/bspline_ctrl_points_marker";
  std::string virtual_obstacles_marker_topic_ = "/planning/virtual_obstacles";
  std::string collision_samples_marker_topic_ = "/planning/bspline_collision_samples";

  bspline::BsplinePathGenerator generator_;

  bool enable_virtual_obstacle_check_ = true;
  double collision_clearance_ = 0.03;
  double default_virtual_obstacle_radius_ = 0.35;
  bool fallback_to_input_on_collision_ = true;

  bool have_candidate_paths_ = false;
  std::vector<std::vector<Eigen::Vector3d>> latest_candidate_paths_;
  std::vector<VirtualObstacle> virtual_obstacles_;
  mutable int last_bspline_candidate_marker_count_ = 0;
  mutable int last_ctrl_marker_count_ = 0;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_paths_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr virtual_obstacles_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bspline_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bspline_candidate_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ctrl_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ctrl_points_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_samples_marker_pub_;
};

}  // namespace bspline_test

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bspline_test::BsplineTestNode>());
  rclcpp::shutdown();
  return 0;
}
