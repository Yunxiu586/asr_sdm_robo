#ifndef ASR_SDM_GUIDANCE_PLANNER_NODE_RVIZ_PLANNING_NODE_HPP_
#define ASR_SDM_GUIDANCE_PLANNER_NODE_RVIZ_PLANNING_NODE_HPP_

#include <asr_sdm_guidance_planner/map/voxel_esdf_map.hpp>
#include <asr_sdm_guidance_planner/optimizer/lbfgs_path_optimizer.hpp>
#include <asr_sdm_guidance_planner/planner/astar_3d_planner.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>
#include <vector>

namespace asr_sdm_guidance_planner
{

class RvizPlanningNode final : public rclcpp::Node
{
public:
  explicit RvizPlanningNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadParameters();
  void mapCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void clickedPointCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
  void startPointCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
  void goalPointCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
  void initialPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void goalPoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  void setStart(const Eigen::Vector3d & start);
  void setGoalAndPlan(const Eigen::Vector3d & goal);
  void runPlanning();

  Eigen::Vector3d pointMsgToEigen(const geometry_msgs::msg::Point & point) const;
  Eigen::Vector3d poseMsgToEigen(const geometry_msgs::msg::Pose & pose) const;

  void publishPath(
    const std::vector<Eigen::Vector3d> & path,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr & pub) const;
  void publishLineMarker(
    const std::vector<Eigen::Vector3d> & path,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub,
    int id,
    double width,
    float r,
    float g,
    float b,
    float a) const;
  void publishStartGoalMarker() const;
  void publishStatusText(const std::string & text) const;

  std::string frame_id_ = "world";
  std::string map_topic_ = "/esdf_map/occupancy_inflate";
  bool clicked_point_use_msg_z_ = true;
  double default_planning_z_ = 0.5;
  bool auto_compute_esdf_on_map_ = true;
  bool use_optimized_only_if_safe_ = true;
  bool skip_repeated_map_with_same_point_count_ = true;
  int last_map_point_count_ = -1;

  VoxelEsdfMap map_;
  VoxelMapOptions map_options_;
  Astar3dPlanner astar_planner_;
  LbfgsPathOptimizer optimizer_;

  bool have_start_ = false;
  bool have_goal_ = false;
  bool waiting_for_goal_click_ = false;
  Eigen::Vector3d start_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr raw_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr status_marker_pub_;
};

}  // namespace asr_sdm_guidance_planner

#endif  // ASR_SDM_GUIDANCE_PLANNER_NODE_RVIZ_PLANNING_NODE_HPP_
