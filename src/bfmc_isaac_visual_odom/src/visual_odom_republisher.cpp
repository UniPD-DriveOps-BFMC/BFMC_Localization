#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class VisualOdomRepublisher : public rclcpp::Node
{
public:
  VisualOdomRepublisher()
  : Node("visual_odom_republisher")
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/visual_slam/tracking/odometry");

    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/visual_odom");

    planar_output_topic_ = this->declare_parameter<std::string>(
      "planar_output_topic", "/visual_odom_planar");

    output_frame_id_ = this->declare_parameter<std::string>(
      "output_frame_id", "visual_odom");

    output_child_frame_id_ = this->declare_parameter<std::string>(
      "output_child_frame_id", "base_link");

    publish_planar_ = this->declare_parameter<bool>(
      "publish_planar", true);

    flatten_planar_ = this->declare_parameter<bool>(
      "flatten_planar", true);

    position_variance_xy_ = this->declare_parameter<double>(
      "position_variance_xy", 0.04);

    position_variance_z_ = this->declare_parameter<double>(
      "position_variance_z", 999.0);

    roll_pitch_variance_ = this->declare_parameter<double>(
      "roll_pitch_variance", 999.0);

    yaw_variance_ = this->declare_parameter<double>(
      "yaw_variance", 0.08);

    linear_velocity_variance_xy_ = this->declare_parameter<double>(
      "linear_velocity_variance_xy", 0.10);

    angular_velocity_variance_yaw_ = this->declare_parameter<double>(
      "angular_velocity_variance_yaw", 0.10);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      output_topic_, 10);

    planar_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      planar_output_topic_, 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic_,
      10,
      std::bind(&VisualOdomRepublisher::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Input Isaac odom: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output odom: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output planar odom: %s", planar_output_topic_.c_str());
  }

private:
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    const double half_yaw = 0.5 * yaw;

    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(half_yaw);
    q.w = std::cos(half_yaw);

    return q;
  }

  void standardizeFrames(nav_msgs::msg::Odometry & msg)
  {
    if (!output_frame_id_.empty()) {
      msg.header.frame_id = output_frame_id_;
    }

    if (!output_child_frame_id_.empty()) {
      msg.child_frame_id = output_child_frame_id_;
    }
  }

  void injectCovariance(nav_msgs::msg::Odometry & msg)
  {
    for (double & value : msg.pose.covariance) {
      value = 0.0;
    }

    for (double & value : msg.twist.covariance) {
      value = 0.0;
    }

    // Pose covariance order:
    // x, y, z, roll, pitch, yaw.
    msg.pose.covariance[0] = position_variance_xy_;
    msg.pose.covariance[7] = position_variance_xy_;
    msg.pose.covariance[14] = position_variance_z_;
    msg.pose.covariance[21] = roll_pitch_variance_;
    msg.pose.covariance[28] = roll_pitch_variance_;
    msg.pose.covariance[35] = yaw_variance_;

    // Twist covariance order:
    // vx, vy, vz, vroll, vpitch, vyaw.
    msg.twist.covariance[0] = linear_velocity_variance_xy_;
    msg.twist.covariance[7] = linear_velocity_variance_xy_;
    msg.twist.covariance[14] = 999.0;
    msg.twist.covariance[21] = 999.0;
    msg.twist.covariance[28] = 999.0;
    msg.twist.covariance[35] = angular_velocity_variance_yaw_;
  }

  nav_msgs::msg::Odometry makePlanarOdometry(const nav_msgs::msg::Odometry & input)
  {
    nav_msgs::msg::Odometry planar = input;

    if (flatten_planar_) {
      const double yaw = yawFromQuaternion(planar.pose.pose.orientation);

      planar.pose.pose.position.z = 0.0;
      planar.pose.pose.orientation = quaternionFromYaw(yaw);

      planar.twist.twist.linear.z = 0.0;
      planar.twist.twist.angular.x = 0.0;
      planar.twist.twist.angular.y = 0.0;
    }

    injectCovariance(planar);

    return planar;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry output = *msg;

    standardizeFrames(output);
    injectCovariance(output);

    odom_pub_->publish(output);

    if (publish_planar_) {
      nav_msgs::msg::Odometry planar = makePlanarOdometry(output);
      planar_pub_->publish(planar);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string planar_output_topic_;
  std::string output_frame_id_;
  std::string output_child_frame_id_;

  bool publish_planar_;
  bool flatten_planar_;

  double position_variance_xy_;
  double position_variance_z_;
  double roll_pitch_variance_;
  double yaw_variance_;
  double linear_velocity_variance_xy_;
  double angular_velocity_variance_yaw_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr planar_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<VisualOdomRepublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}