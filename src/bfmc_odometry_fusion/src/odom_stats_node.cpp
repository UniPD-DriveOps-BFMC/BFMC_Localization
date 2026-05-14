#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

class OdomStatsNode : public rclcpp::Node
{
public:
  OdomStatsNode()
  : Node("odom_stats_node")
  {
    local_odom_topic_ = declare_parameter<std::string>("local_odom_topic", "/odometry/local");
    distance_topic_ = declare_parameter<std::string>("distance_topic", "/odom_distance");
    velocity_topic_ = declare_parameter<std::string>("velocity_topic", "/odom_velocity");

    distance_pub_ = create_publisher<std_msgs::msg::Float32>(distance_topic_, 10);
    velocity_pub_ = create_publisher<std_msgs::msg::Float32>(velocity_topic_, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_,
      10,
      std::bind(&OdomStatsNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Odom stats node started");
    RCLCPP_INFO(get_logger(), "Input: %s", local_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Distance: %s", distance_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Velocity: %s", velocity_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    if (has_prev_) {
      const double dx = x - prev_x_;
      const double dy = y - prev_y_;
      total_distance_ += std::sqrt(dx * dx + dy * dy);
    }

    prev_x_ = x;
    prev_y_ = y;
    has_prev_ = true;

    std_msgs::msg::Float32 dist_msg;
    dist_msg.data = static_cast<float>(total_distance_);
    distance_pub_->publish(dist_msg);

    std_msgs::msg::Float32 vel_msg;
    vel_msg.data = static_cast<float>(msg->twist.twist.linear.x);
    velocity_pub_->publish(vel_msg);
  }

  std::string local_odom_topic_;
  std::string distance_topic_;
  std::string velocity_topic_;

  bool has_prev_{false};
  double prev_x_{0.0};
  double prev_y_{0.0};
  double total_distance_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomStatsNode>());
  rclcpp::shutdown();
  return 0;
}
