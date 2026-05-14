#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "bfmc_gps_position/msg/gps_tag_pose.hpp"

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
    tag_pose_topic_ = declare_parameter<std::string>("tag_pose_topic", "/automobile/gps/tag_pose");
    local_odom_topic_ = declare_parameter<std::string>("local_odom_topic", "/odometry/local");
    base_pose_topic_ = declare_parameter<std::string>("base_pose_topic", "/automobile/gps/base_pose");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    tag_frame_ = declare_parameter<std::string>("tag_frame", "gps_tag_link");

    // At quality=100 this is the stddev in metres (max error radius = 15 cm).
    base_stddev_m_ = declare_parameter<double>("base_stddev_m", 0.15);
    max_stddev_m_ = declare_parameter<double>("max_stddev_m", 2.0);
    min_quality_ = declare_parameter<int>("min_quality", 5);

    // GPS messages arrive ~1 s after the measurement was taken.
    // This value is subtracted from the header stamp so that robot_localization
    // associates the measurement with the correct time in its history buffer.
    measurement_delay_s_ = declare_parameter<double>("measurement_delay_s", 1.0);

    base_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(base_pose_topic_, 10);

    tag_pose_sub_ =
      create_subscription<bfmc_gps_position::msg::GpsTagPose>(
        tag_pose_topic_, 10,
        std::bind(&GpsTagToBaseNode::tagPoseCallback, this, std::placeholders::_1));

    local_odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
        local_odom_topic_, 10,
        std::bind(&GpsTagToBaseNode::localOdomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "GPS tag-to-base node started");
    RCLCPP_INFO(get_logger(), "Input tag pose : %s", tag_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Input local odom: %s", local_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output base pose: %s", base_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Stddev at quality=100: %.3f m", base_stddev_m_);
    RCLCPP_INFO(get_logger(), "Delay compensation: %.2f s", measurement_delay_s_);
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
      const geometry_msgs::msg::TransformStamped tf =
        tf_buffer_.lookupTransform(base_frame_, tag_frame_, tf2::TimePointZero);
      offset_x = tf.transform.translation.x;
      offset_y = tf.transform.translation.y;
      offset_z = tf.transform.translation.z;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot get TF %s -> %s: %s",
        base_frame_.c_str(), tag_frame_.c_str(), ex.what());
      return false;
    }
  }

  // Linear scaling: stddev = base_stddev * (100 / quality)
  //   quality=100  →  stddev = base_stddev_m_   (e.g. 0.15 m)
  //   quality=50   →  stddev = 2 * base_stddev_m_ (e.g. 0.30 m)
  //   quality=25   →  stddev = 4 * base_stddev_m_ (e.g. 0.60 m)
  double computeVariance(const int quality) const
  {
    const double stddev =
      std::min(base_stddev_m_ * (100.0 / static_cast<double>(quality)), max_stddev_m_);
    return stddev * stddev;
  }

  void tagPoseCallback(const bfmc_gps_position::msg::GpsTagPose::SharedPtr msg)
  {
    if (msg->quality < min_quality_) {
      RCLCPP_DEBUG(get_logger(), "GPS quality too low (%d < %d), skipping", msg->quality, min_quality_);
      return;
    }

    if (!has_local_yaw_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No local yaw yet, skipping GPS");
      return;
    }

    double tag_offset_x = 0.0, tag_offset_y = 0.0, tag_offset_z = 0.0;
    if (!getTagOffsetFromTf(tag_offset_x, tag_offset_y, tag_offset_z)) {
      return;
    }

    // Rotate tag offset by current yaw and subtract to get base_link position.
    const double yaw = latest_yaw_rad_;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    const double base_x = static_cast<double>(msg->x) - (c * tag_offset_x - s * tag_offset_y);
    const double base_y = static_cast<double>(msg->y) - (s * tag_offset_x + c * tag_offset_y);

    const double pos_var = computeVariance(msg->quality);

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    // Subtract the known transmission delay so robot_localization can match
    // this measurement to the correct time in its EKF history.
    out.header.stamp = rclcpp::Time(msg->header.stamp) -
      rclcpp::Duration::from_seconds(measurement_delay_s_);
    out.header.frame_id = map_frame_;

    out.pose.pose.position.x = base_x;
    out.pose.pose.position.y = base_y;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = yawToQuaternion(yaw);

    // GPS gives x, y position only — yaw covariance is very large.
    out.pose.covariance[0]  = pos_var;     // x
    out.pose.covariance[7]  = pos_var;     // y
    out.pose.covariance[14] = 999.0;       // z
    out.pose.covariance[21] = 999.0;       // roll
    out.pose.covariance[28] = 999.0;       // pitch
    out.pose.covariance[35] = 999999.0;    // yaw (not from GPS)

    base_pose_pub_->publish(out);

    RCLCPP_DEBUG(
      get_logger(), "GPS pose published: x=%.3f y=%.3f quality=%d stddev=%.3f m",
      base_x, base_y, msg->quality, std::sqrt(pos_var));
  }

  std::string tag_pose_topic_;
  std::string local_odom_topic_;
  std::string base_pose_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string tag_frame_;

  double base_stddev_m_{0.15};
  double max_stddev_m_{2.0};
  double measurement_delay_s_{1.0};
  int min_quality_{5};

  bool has_local_yaw_{false};
  double latest_yaw_rad_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr base_pose_pub_;
  rclcpp::Subscription<bfmc_gps_position::msg::GpsTagPose>::SharedPtr tag_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsTagToBaseNode>());
  rclcpp::shutdown();
  return 0;
}
