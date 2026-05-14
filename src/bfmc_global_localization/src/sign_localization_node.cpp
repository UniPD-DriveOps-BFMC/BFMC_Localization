#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

struct MapSign {
  double x;
  double y;
};

class SignLocalizationNode : public rclcpp::Node
{
public:
  SignLocalizationNode()
  : Node("sign_localization_node")
  {
    sign_map_file_ = declare_parameter<std::string>("sign_map_file", "");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/traffic/detection");
    distance_topic_ = declare_parameter<std::string>("distance_topic", "/traffic/distance");
    local_odom_topic_ = declare_parameter<std::string>("local_odom_topic", "/odometry/local");
    output_pose_topic_ = declare_parameter<std::string>(
      "output_pose_topic", "/automobile/sign/base_pose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    // Uncertainty of the distance sensor reading (metres).
    distance_stddev_m_ = declare_parameter<double>("distance_stddev_m", 0.30);
    // Uncertainty of the bearing to the sign (radians).
    // Contributes position error proportional to distance: perp_err = dist * stddev_bearing.
    bearing_stddev_rad_ = declare_parameter<double>("bearing_stddev_rad", 0.10);
    // Only use a sign if the nearest map candidate is within this radius
    // of the current odometry estimate.
    max_association_m_ = declare_parameter<double>("max_association_m", 5.0);

    if (!loadSignMap(sign_map_file_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load sign map: %s", sign_map_file_.c_str());
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_pose_topic_, 10);

    detection_sub_ = create_subscription<std_msgs::msg::String>(
      detection_topic_, 10,
      std::bind(&SignLocalizationNode::detectionCallback, this, std::placeholders::_1));

    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      distance_topic_, 10,
      std::bind(&SignLocalizationNode::distanceCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_, 10,
      std::bind(&SignLocalizationNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Sign localization node started");
    RCLCPP_INFO(get_logger(), "Sign map  : %s", sign_map_file_.c_str());
    RCLCPP_INFO(get_logger(), "Detection : %s", detection_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Distance  : %s", distance_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output    : %s", output_pose_topic_.c_str());
  }

private:
  bool loadSignMap(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {return false;}

    std::string type;
    double x, y;
    int count = 0;
    while (file >> type >> x >> y) {
      sign_map_[type].push_back({x, y});
      ++count;
    }

    for (const auto & [t, signs] : sign_map_) {
      RCLCPP_INFO(get_logger(), "  %-15s %zu sign(s)", t.c_str(), signs.size());
    }
    RCLCPP_INFO(get_logger(), "Loaded %d signs (%zu types)", count, sign_map_.size());
    return count > 0;
  }

  void distanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_distance_m_ = static_cast<double>(msg->data);
    has_distance_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_x_ = msg->pose.pose.position.x;
    latest_y_ = msg->pose.pose.position.y;
    latest_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
    has_odom_ = true;
  }

  void detectionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!has_distance_ || !has_odom_) {
      RCLCPP_DEBUG(get_logger(), "Waiting for distance and odom");
      return;
    }

    const std::string & type = msg->data;
    auto it = sign_map_.find(type);
    if (it == sign_map_.end()) {
      RCLCPP_DEBUG(get_logger(), "Sign type '%s' not in map", type.c_str());
      return;
    }

    // Find the nearest map sign of this type to the current odometry estimate.
    const MapSign * best = nullptr;
    double best_dist = std::numeric_limits<double>::max();
    for (const auto & sign : it->second) {
      const double dx = sign.x - latest_x_;
      const double dy = sign.y - latest_y_;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d < best_dist) {
        best_dist = d;
        best = &sign;
      }
    }

    if (!best || best_dist > max_association_m_) {
      RCLCPP_DEBUG(
        get_logger(), "No '%s' within %.1f m (nearest %.1f m) — skipping",
        type.c_str(), max_association_m_, best_dist);
      return;
    }

    // Car is behind the sign at distance d along the current heading.
    const double car_x = best->x - latest_distance_m_ * std::cos(latest_yaw_);
    const double car_y = best->y - latest_distance_m_ * std::sin(latest_yaw_);

    // Variance: radial error from distance uncertainty + lateral error from bearing uncertainty.
    const double var_radial = distance_stddev_m_ * distance_stddev_m_;
    const double var_lateral = latest_distance_m_ * bearing_stddev_rad_;
    const double pos_var = var_radial + var_lateral * var_lateral;

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = now();
    out.header.frame_id = map_frame_;
    out.pose.pose.position.x = car_x;
    out.pose.pose.position.y = car_y;
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = yawToQuaternion(latest_yaw_);

    out.pose.covariance[0]  = pos_var;    // x
    out.pose.covariance[7]  = pos_var;    // y
    out.pose.covariance[14] = 999.0;      // z
    out.pose.covariance[21] = 999.0;      // roll
    out.pose.covariance[28] = 999.0;      // pitch
    out.pose.covariance[35] = 999999.0;   // yaw — not from sign detection

    pose_pub_->publish(out);

    RCLCPP_DEBUG(
      get_logger(),
      "Sign '%s' at (%.2f,%.2f) dist=%.2f → car=(%.2f,%.2f) var=%.4f",
      type.c_str(), best->x, best->y, latest_distance_m_, car_x, car_y, pos_var);
  }

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
  }

  std::string sign_map_file_;
  std::string detection_topic_;
  std::string distance_topic_;
  std::string local_odom_topic_;
  std::string output_pose_topic_;
  std::string map_frame_;

  double distance_stddev_m_{0.30};
  double bearing_stddev_rad_{0.10};
  double max_association_m_{5.0};

  std::unordered_map<std::string, std::vector<MapSign>> sign_map_;

  bool has_distance_{false};
  bool has_odom_{false};
  double latest_distance_m_{0.0};
  double latest_x_{0.0};
  double latest_y_{0.0};
  double latest_yaw_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
