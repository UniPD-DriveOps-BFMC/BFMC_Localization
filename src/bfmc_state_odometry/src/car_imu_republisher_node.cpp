#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class CarImuRepublisherNode : public rclcpp::Node
{
public:
  CarImuRepublisherNode()
  : Node("car_imu_republisher_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/automobile/imu/data");
    output_topic_ = declare_parameter<std::string>("output_topic", "/car/imu/data");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "imu_link");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_,
      50,
      std::bind(&CarImuRepublisherNode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Car IMU republisher started");
    RCLCPP_INFO(get_logger(), "input_topic : %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "frame_id    : %s", output_frame_id_.c_str());
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out = *msg;
    out.header.stamp = now();
    out.header.frame_id = output_frame_id_;
    // Covariance is passed through as-is from the IMU publisher:
    //   orientation     [0.02, 0.02, 0.5]  — roll/pitch trusted, yaw uncertain
    //   angular_velocity [0.01, 0.01, 0.01]
    //   linear_accel     [0.1,  0.1,  0.1]
    imu_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarImuRepublisherNode>());
  rclcpp::shutdown();
  return 0;
}
