#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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
    sign_map_file_     = declare_parameter<std::string>("sign_map_file", "");
    detection_topic_   = declare_parameter<std::string>("detection_topic", "/traffic/detection");
    local_odom_topic_  = declare_parameter<std::string>("local_odom_topic", "/odometry/local");
    output_pose_topic_ = declare_parameter<std::string>("output_pose_topic", "/automobile/sign/base_pose");
    map_frame_         = declare_parameter<std::string>("map_frame", "map");

    distance_stddev_m_  = declare_parameter<double>("distance_stddev_m", 0.30);
    bearing_stddev_rad_ = declare_parameter<double>("bearing_stddev_rad", 0.10);
    max_association_m_  = declare_parameter<double>("max_association_m", 5.0);
    // Detections below this confidence are ignored.
    min_confidence_     = declare_parameter<double>("min_confidence", 0.50);

    if (!loadSignMap(sign_map_file_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load sign map: %s", sign_map_file_.c_str());
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_pose_topic_, 10);

    detection_sub_ = create_subscription<std_msgs::msg::String>(
      detection_topic_, 10,
      std::bind(&SignLocalizationNode::detectionCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_, 10,
      std::bind(&SignLocalizationNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Sign localization node started");
    RCLCPP_INFO(get_logger(), "Sign map  : %s", sign_map_file_.c_str());
    RCLCPP_INFO(get_logger(), "Detection : %s", detection_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output    : %s", output_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Min confidence: %.2f", min_confidence_);
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

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_x_   = msg->pose.pose.position.x;
    latest_y_   = msg->pose.pose.position.y;
    latest_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
    has_odom_   = true;
  }

  void detectionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!has_odom_) {
      RCLCPP_DEBUG(get_logger(), "Waiting for odom");
      return;
    }

    // Parse JSON: {"sign": "stop", "distance_m": 0.439, "confidence": 0.859}
    nlohmann::json j;
    try {
      j = nlohmann::json::parse(msg->data);
    } catch (const nlohmann::json::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to parse detection JSON: %s", e.what());
      return;
    }

    if (!j.contains("sign") || !j.contains("distance_m") || !j.contains("confidence")) {
      RCLCPP_WARN(get_logger(), "Detection JSON missing required fields: %s", msg->data.c_str());
      return;
    }

    const std::string type      = j["sign"].get<std::string>();
    const double distance_m     = j["distance_m"].get<double>();
    const double confidence     = j["confidence"].get<double>();

    if (confidence < min_confidence_) {
      RCLCPP_DEBUG(get_logger(), "Low confidence %.2f < %.2f for '%s', skipping",
        confidence, min_confidence_, type.c_str());
      return;
    }

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
      const double d  = std::sqrt(dx * dx + dy * dy);
      if (d < best_dist) {
        best_dist = d;
        best = &sign;
      }
    }

    if (!best || best_dist > max_association_m_) {
      RCLCPP_DEBUG(get_logger(), "No '%s' within %.1f m (nearest %.1f m) — skipping",
        type.c_str(), max_association_m_, best_dist);
      return;
    }

    // Signs are beside the road, not directly ahead — compute the geometric
    // bearing from the current odometry position to the sign and apply
    // distance_m along that direction rather than along the car heading.
    const double bearing = std::atan2(best->y - latest_y_, best->x - latest_x_);
    const double car_x = best->x - distance_m * std::cos(bearing);
    const double car_y = best->y - distance_m * std::sin(bearing);

    // Base variance from distance and bearing uncertainty.
    const double var_radial  = distance_stddev_m_ * distance_stddev_m_;
    const double var_lateral = distance_m * bearing_stddev_rad_;
    double pos_var = var_radial + var_lateral * var_lateral;

    // Scale variance by confidence: lower confidence → larger uncertainty.
    pos_var /= std::max(confidence, 0.01);

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp    = now();
    out.header.frame_id = map_frame_;
    out.pose.pose.position.x  = car_x;
    out.pose.pose.position.y  = car_y;
    out.pose.pose.position.z  = 0.0;
    out.pose.pose.orientation = yawToQuaternion(latest_yaw_);

    out.pose.covariance[0]  = pos_var;     // x
    out.pose.covariance[7]  = pos_var;     // y
    out.pose.covariance[14] = 999.0;       // z
    out.pose.covariance[21] = 999.0;       // roll
    out.pose.covariance[28] = 999.0;       // pitch
    out.pose.covariance[35] = 999999.0;    // yaw — not from sign detection

    pose_pub_->publish(out);

    RCLCPP_DEBUG(get_logger(),
      "Sign '%s' at (%.2f,%.2f) dist=%.2f conf=%.2f → car=(%.2f,%.2f) var=%.4f",
      type.c_str(), best->x, best->y, distance_m, confidence, car_x, car_y, pos_var);
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
  std::string local_odom_topic_;
  std::string output_pose_topic_;
  std::string map_frame_;

  double distance_stddev_m_{0.30};
  double bearing_stddev_rad_{0.10};
  double max_association_m_{5.0};
  double min_confidence_{0.50};

  std::unordered_map<std::string, std::vector<MapSign>> sign_map_;

  bool has_odom_{false};
  double latest_x_{0.0};
  double latest_y_{0.0};
  double latest_yaw_{0.0};

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
