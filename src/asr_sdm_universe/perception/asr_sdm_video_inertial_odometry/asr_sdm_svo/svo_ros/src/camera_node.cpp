#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

namespace asr_sdm_camera
{

class CameraNode : public rclcpp::Node
{
public:
  CameraNode()
  : Node("asr_sdm_camera_realsense_d405")
  {
    this->declare_parameter<std::string>("camera_info_topic_in", "/camera/camera_info");
    this->declare_parameter<std::string>("image_topic_in", "/camera/image_raw");
    this->declare_parameter<std::string>("camera_info_topic_out", "/sensing/camera/realsense/color/camera_info");
    this->declare_parameter<std::string>("image_topic_out", "/sensing/camera/realsense/color/image_raw");

    std::string image_in, info_in, image_out, info_out;
    this->get_parameter("image_topic_in", image_in);
    this->get_parameter("camera_info_topic_in", info_in);
    this->get_parameter("image_topic_out", image_out);
    this->get_parameter("camera_info_topic_out", info_out);

    RCLCPP_INFO(this->get_logger(), "Camera input:   image=%s, info=%s", image_in.c_str(), info_in.c_str());
    RCLCPP_INFO(this->get_logger(), "Camera output:  image=%s, info=%s", image_out.c_str(), info_out.c_str());

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_in, rclcpp::SensorDataQoS(),
        [this, image_out](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
          if (pub_image_ && pub_image_->get_subscription_count() > 0) {
            pub_image_->publish(*msg);
          }
        });

    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        info_in, rclcpp::SensorDataQoS(),
        [this, info_out](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          if (pub_info_ && pub_info_->get_subscription_count() > 0) {
            pub_info_->publish(*msg);
          }
        });

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(image_out, rclcpp::SensorDataQoS());
    pub_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(info_out, rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "asr_sdm_camera_realsense_d405 node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
};

}  // namespace asr_sdm_camera

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<asr_sdm_camera::CameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
