#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

class GpsTagToBaseNode : public rclcpp::Node
{
public:
  GpsTagToBaseNode()
  : Node("gps_tag_to_base_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    tag_pose_topic_ = declare_parameter<std::string>(
      "tag_pose_topic", "/automobile/gps/tag_pose");

    local_odom_topic_ = declare_parameter<std::string>(
      "local_odom_topic", "/odometry/local");

    base_pose_topic_ = declare_parameter<std::string>(
      "base_pose_topic", "/automobile/gps/base_pose");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tag_frame_ = declare_parameter<std::string>("tag_frame", "gps_tag_link");

    use_yaw_from_local_odom_ = declare_parameter<bool>(
      "use_yaw_from_local_odom", true);

    force_xy_variance_ = declare_parameter<bool>(
      "force_xy_variance", true);

    position_stddev_m_ = declare_parameter<double>(
      "position_stddev_m", 0.15);

    base_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        base_pose_topic_, 10);

    tag_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        tag_pose_topic_,
        10,
        std::bind(&GpsTagToBaseNode::tagPoseCallback, this, std::placeholders::_1));

    local_odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
        local_odom_topic_,
        10,
        std::bind(&GpsTagToBaseNode::localOdomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "GPS tag-to-base node started");
    RCLCPP_INFO(get_logger(), "Input tag pose: %s", tag_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Input local odom: %s", local_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output base pose: %s", base_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Using TF offset: %s -> %s", base_frame_.c_str(), tag_frame_.c_str());
  }

private:
  static geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
  }

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
  {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
  }

  void localOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_yaw_rad_ = quaternionToYaw(msg->pose.pose.orientation);
    has_local_yaw_ = true;
  }

  bool getTagOffsetFromTf(double & offset_x, double & offset_y, double & offset_z)
  {
    try {
      // This gives the position of gps_tag_link expressed in base_link.
      const geometry_msgs::msg::TransformStamped tf =
        tf_buffer_.lookupTransform(
          base_frame_,
          tag_frame_,
          tf2::TimePointZero);

      offset_x = tf.transform.translation.x;
      offset_y = tf.transform.translation.y;
      offset_z = tf.transform.translation.z;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Cannot get TF %s -> %s: %s",
        base_frame_.c_str(),
        tag_frame_.c_str(),
        ex.what());
      return false;
    }
  }

  void tagPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    double tag_offset_x = 0.0;
    double tag_offset_y = 0.0;
    double tag_offset_z = 0.0;

    if (!getTagOffsetFromTf(tag_offset_x, tag_offset_y, tag_offset_z)) {
      return;
    }

    double yaw = quaternionToYaw(msg->pose.pose.orientation);

    if (use_yaw_from_local_odom_ && has_local_yaw_) {
      yaw = latest_yaw_rad_;
    }

    const double tag_x_map = msg->pose.pose.position.x;
    const double tag_y_map = msg->pose.pose.position.y;

    // p_tag_map = p_base_map + R(yaw) * p_tag_base
    // p_base_map = p_tag_map - R(yaw) * p_tag_base
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    const double offset_x_map = c * tag_offset_x - s * tag_offset_y;
    const double offset_y_map = s * tag_offset_x + c * tag_offset_y;

    const double base_x_map = tag_x_map - offset_x_map;
    const double base_y_map = tag_y_map - offset_y_map;

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = map_frame_;

    out.pose.pose.position.x = base_x_map;
    out.pose.pose.position.y = base_y_map;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = yawToQuaternion(yaw);

    out.pose.covariance = msg->pose.covariance;

    if (force_xy_variance_) {
      const double pos_var = position_stddev_m_ * position_stddev_m_;
      out.pose.covariance[0] = pos_var;       // x
      out.pose.covariance[7] = pos_var;       // y
      out.pose.covariance[14] = 999.0;        // z
      out.pose.covariance[21] = 999.0;        // roll
      out.pose.covariance[28] = 999.0;        // pitch
      out.pose.covariance[35] = 999999.0;     // yaw
    }

    base_pose_pub_->publish(out);
  }

  std::string tag_pose_topic_;
  std::string local_odom_topic_;
  std::string base_pose_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string tag_frame_;

  bool use_yaw_from_local_odom_{true};
  bool force_xy_variance_{true};
  double position_stddev_m_{0.15};

  bool has_local_yaw_{false};
  double latest_yaw_rad_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr base_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tag_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsTagToBaseNode>());
  rclcpp::shutdown();
  return 0;
}
