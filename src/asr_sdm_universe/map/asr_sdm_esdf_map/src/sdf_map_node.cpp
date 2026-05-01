#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_esdf_map/sdf_map.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("sdf_map");
  SDFMap sdf_map;
  sdf_map.initMap(node);

  RCLCPP_INFO(node->get_logger(), "sdf_map node started");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
