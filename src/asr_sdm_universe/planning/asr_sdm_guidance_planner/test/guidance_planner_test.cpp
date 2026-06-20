#include <rclcpp/rclcpp.hpp>
#include <rviz_planning_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<asr_sdm_guidance_planner::RvizPlanningNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
