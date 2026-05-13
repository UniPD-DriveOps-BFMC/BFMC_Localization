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
    input_topic_ = declare_parameter<std::string>("input_topic", "/automobile/IMU");
    output_topic_ = declare_parameter<std::string>("output_topic", "/car/imu/data");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "car_imu_link");

    orientation_variance_ = declare_parameter<double>("orientation_variance", 999999.0);
    angular_velocity_variance_ = declare_parameter<double>("angular_velocity_variance", 0.02);
    linear_acceleration_variance_ = declare_parameter<double>("linear_acceleration_variance", 0.5);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_,
      50,
      std::bind(&CarImuRepublisherNode::imuCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Car IMU republisher started");
    RCLCPP_INFO(get_logger(), "input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "output_frame_id: %s", output_frame_id_.c_str());
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out = *msg;

    out.header.stamp = now();
    out.header.frame_id = output_frame_id_;

    // First safe configuration:
    // Do not trust absolute orientation unless the IMU has a proper orientation filter.
    out.orientation_covariance[0] = orientation_variance_;
    out.orientation_covariance[4] = orientation_variance_;
    out.orientation_covariance[8] = orientation_variance_;

    // Trust angular velocity moderately, especially yaw-rate z.
    out.angular_velocity_covariance[0] = angular_velocity_variance_;
    out.angular_velocity_covariance[4] = angular_velocity_variance_;
    out.angular_velocity_covariance[8] = angular_velocity_variance_;

    // Trust acceleration weakly at first.
    out.linear_acceleration_covariance[0] = linear_acceleration_variance_;
    out.linear_acceleration_covariance[4] = linear_acceleration_variance_;
    out.linear_acceleration_covariance[8] = linear_acceleration_variance_;

    imu_pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;

  double orientation_variance_{999999.0};
  double angular_velocity_variance_{0.02};
  double linear_acceleration_variance_{0.5};

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
