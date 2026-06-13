#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>

namespace asr_sdm_imu
{

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("asr_sdm_imu_hiwonder_10axis")
  {
    this->declare_parameter<std::string>("imu_topic_in", "/sensing/imu/imu_raw");
    this->declare_parameter<std::string>("imu_topic_out_raw", "/sensing/imu/imu_raw");
    this->declare_parameter<std::string>("imu_topic_out_filtered", "/sensing/imu/imu_filtered");

    std::string imu_topic_in;
    std::string imu_topic_out_raw;
    std::string imu_topic_out_filtered;
    this->get_parameter("imu_topic_in", imu_topic_in);
    this->get_parameter("imu_topic_out_raw", imu_topic_out_raw);
    this->get_parameter("imu_topic_out_filtered", imu_topic_out_filtered);

    RCLCPP_INFO(this->get_logger(), "IMU input topic: %s", imu_topic_in.c_str());
    RCLCPP_INFO(this->get_logger(), "IMU output (raw): %s", imu_topic_out_raw.c_str());
    RCLCPP_INFO(this->get_logger(), "IMU output (filtered): %s", imu_topic_out_filtered.c_str());

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_in, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          if (pub_raw_ && pub_raw_->get_subscription_count() > 0) {
            pub_raw_->publish(*msg);
          }
          if (pub_filtered_ && pub_filtered_->get_subscription_count() > 0) {
            sensor_msgs::msg::Imu filtered_msg = *msg;
            // TODO: apply IMU filter (bias correction, noise filtering, etc.)
            pub_filtered_->publish(filtered_msg);
          }
        });

    pub_raw_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_out_raw, rclcpp::QoS(100).reliable());
    pub_filtered_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_out_filtered, rclcpp::QoS(100).reliable());

    RCLCPP_INFO(this->get_logger(), "asr_sdm_imu_hiwonder_10axis node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_filtered_;
};

}  // namespace asr_sdm_imu

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<asr_sdm_imu::ImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
