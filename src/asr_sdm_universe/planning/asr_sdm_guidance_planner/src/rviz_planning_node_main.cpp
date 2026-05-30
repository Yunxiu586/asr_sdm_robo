#include <asr_sdm_guidance_planner/node/rviz_planning_node.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<asr_sdm_guidance_planner::RvizPlanningNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
