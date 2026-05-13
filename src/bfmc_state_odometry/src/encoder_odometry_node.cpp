#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class EncoderOdometryNode : public rclcpp::Node
{
public:
  EncoderOdometryNode()
  : Node("encoder_odometry_node")
  {
    speed_topic_ = declare_parameter<std::string>("speed_topic", "/automobile/encoder/speed");
    distance_topic_ = declare_parameter<std::string>("distance_topic", "/automobile/encoder/distance");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/encoder_odom");

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    use_distance_ = declare_parameter<bool>("use_distance", false);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);

    velocity_variance_ = declare_parameter<double>("velocity_variance", 0.02);
    large_variance_ = declare_parameter<double>("large_variance", 999999.0);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      speed_topic_,
      10,
      std::bind(&EncoderOdometryNode::speedCallback, this, std::placeholders::_1));

    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      distance_topic_,
      10,
      std::bind(&EncoderOdometryNode::distanceCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EncoderOdometryNode::timerCallback, this));

    last_time_ = now();

    RCLCPP_INFO(get_logger(), "Encoder odometry node started");
    RCLCPP_INFO(get_logger(), "speed_topic: %s", speed_topic_.c_str());
    RCLCPP_INFO(get_logger(), "distance_topic: %s", distance_topic_.c_str());
    RCLCPP_INFO(get_logger(), "odom_topic: %s", odom_topic_.c_str());
  }

private:
  static geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
  }

  void speedCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    speed_mps_ = static_cast<double>(msg->data);
    got_speed_ = true;
  }

  void distanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    const double distance_m = static_cast<double>(msg->data);

    if (!got_distance_) {
      previous_distance_m_ = distance_m;
      got_distance_ = true;
      return;
    }

    delta_distance_m_ += distance_m - previous_distance_m_;
    previous_distance_m_ = distance_m;
  }

  void fillCovariance(nav_msgs::msg::Odometry & odom)
  {
    odom.pose.covariance.fill(0.0);
    odom.twist.covariance.fill(0.0);

    // Do not trust encoder-only pose in the EKF.
    odom.pose.covariance[0] = large_variance_;    // x
    odom.pose.covariance[7] = large_variance_;    // y
    odom.pose.covariance[14] = large_variance_;   // z
    odom.pose.covariance[21] = large_variance_;   // roll
    odom.pose.covariance[28] = large_variance_;   // pitch
    odom.pose.covariance[35] = large_variance_;   // yaw

    // Trust mainly forward velocity.
    odom.twist.covariance[0] = velocity_variance_; // vx
    odom.twist.covariance[7] = large_variance_;    // vy
    odom.twist.covariance[14] = large_variance_;   // vz
    odom.twist.covariance[21] = large_variance_;   // wx
    odom.twist.covariance[28] = large_variance_;   // wy
    odom.twist.covariance[35] = large_variance_;   // wz
  }

  void timerCallback()
  {
    const rclcpp::Time current_time = now();
    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0) {
      return;
    }

    double ds = 0.0;

    if (use_distance_) {
      ds = delta_distance_m_;
      delta_distance_m_ = 0.0;
      speed_mps_ = ds / dt;
    } else {
      ds = speed_mps_ * dt;
    }

    // First version: encoder node does not estimate heading.
    // EKF will combine this forward velocity with IMU yaw-rate and visual odom.
    x_m_ += ds * std::cos(yaw_rad_);
    y_m_ += ds * std::sin(yaw_rad_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_m_;
    odom.pose.pose.position.y = y_m_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = yawToQuaternion(yaw_rad_);

    odom.twist.twist.linear.x = speed_mps_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    fillCovariance(odom);
    odom_pub_->publish(odom);
  }

  std::string speed_topic_;
  std::string distance_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  bool use_distance_{false};
  bool got_speed_{false};
  bool got_distance_{false};

  double publish_rate_hz_{50.0};
  double velocity_variance_{0.02};
  double large_variance_{999999.0};

  double speed_mps_{0.0};
  double previous_distance_m_{0.0};
  double delta_distance_m_{0.0};

  double x_m_{0.0};
  double y_m_{0.0};
  double yaw_rad_{0.0};

  rclcpp::Time last_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
