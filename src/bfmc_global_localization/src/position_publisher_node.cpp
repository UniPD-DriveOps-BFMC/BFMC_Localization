#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

struct GraphNode {
  double x;
  double y;
};

class PositionPublisherNode : public rclcpp::Node
{
public:
  PositionPublisherNode()
  : Node("position_publisher_node")
  {
    use_gps_           = declare_parameter<bool>("use_gps", true);
    odom_topic_        = declare_parameter<std::string>("odom_topic", "/odometry/global");
    local_odom_topic_  = declare_parameter<std::string>("local_odom_topic", "/odometry/local");
    coordinate_topic_  = declare_parameter<std::string>(
      "coordinate_topic", "/automobile/current_coordinate");
    node_topic_        = declare_parameter<std::string>("node_topic", "/automobile/current_node");
    distance_topic_    = declare_parameter<std::string>(
      "distance_topic", "/automobile/total_distance");
    speed_topic_       = declare_parameter<std::string>("speed_topic", "/automobile/current_speed");
    graph_file_        = declare_parameter<std::string>("graph_file", "");

    if (!loadGraphNodes(graph_file_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load graph: %s", graph_file_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu graph nodes", graph_nodes_.size());
    }

    coordinate_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(coordinate_topic_, 10);
    node_pub_       = create_publisher<std_msgs::msg::Int32>(node_topic_, 10);
    distance_pub_   = create_publisher<std_msgs::msg::Float64>(distance_topic_, 10);
    speed_pub_      = create_publisher<std_msgs::msg::Float64>(speed_topic_, 10);

    // Global odom: coordinate, node, speed
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&PositionPublisherNode::odomCallback, this, std::placeholders::_1));

    // Local odom: total distance accumulation (odom frame = relative to start, no drift reset)
    local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_, 10,
      std::bind(&PositionPublisherNode::localOdomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Position publisher started");
    RCLCPP_INFO(
      get_logger(), "Mode       : %s",
      use_gps_ ? "GPS (map origin = 0,0)" : "no-GPS (start = 0,0)");
    RCLCPP_INFO(get_logger(), "Global odom: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Local odom : %s", local_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Coordinate : %s", coordinate_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Node       : %s", node_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Distance   : %s", distance_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Speed      : %s", speed_topic_.c_str());
  }

private:
  bool loadGraphNodes(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {return false;}

    std::string line;
    int current_id = -1;
    double x = 0.0, y = 0.0;
    bool has_x = false, has_y = false;

    auto extractValue = [](const std::string & l) -> double {
      const size_t s = l.find('>') + 1;
      const size_t e = l.find('<', s);
      return std::stod(l.substr(s, e - s));
    };

    while (std::getline(file, line)) {
      if (line.find("<node id=\"") != std::string::npos) {
        const size_t s = line.find("<node id=\"") + 10;
        const size_t e = line.find('"', s);
        current_id = std::stoi(line.substr(s, e - s));
        has_x = false;
        has_y = false;
      } else if (current_id >= 0 && line.find("key=\"d0\"") != std::string::npos) {
        x = extractValue(line);
        has_x = true;
      } else if (current_id >= 0 && line.find("key=\"d1\"") != std::string::npos) {
        y = extractValue(line);
        has_y = true;
      } else if (line.find("</node>") != std::string::npos) {
        if (current_id >= 0 && has_x && has_y) {
          graph_nodes_[current_id] = {x, y};
        }
        current_id = -1;
      }
    }

    return !graph_nodes_.empty();
  }

  int findNearestNode(const double px, const double py) const
  {
    int best_id = -1;
    double best_dist = std::numeric_limits<double>::max();

    for (const auto & [id, node] : graph_nodes_) {
      const double dx = node.x - px;
      const double dy = node.y - py;
      const double d = dx * dx + dy * dy;
      if (d < best_dist) {
        best_dist = d;
        best_id = id;
      }
    }
    return best_id;
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

  // Accumulate total path length from local odometry (odom frame, no GPS dependency).
  void localOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;

    if (has_prev_local_) {
      const double dx = x - prev_local_x_;
      const double dy = y - prev_local_y_;
      total_distance_ += std::sqrt(dx * dx + dy * dy);
    }
    prev_local_x_ = x;
    prev_local_y_ = y;
    has_prev_local_ = true;

    std_msgs::msg::Float64 d_msg;
    d_msg.data = total_distance_;
    distance_pub_->publish(d_msg);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Speed from fused odometry twist (forward + lateral, 2D magnitude).
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = std::sqrt(vx * vx + vy * vy);
    speed_pub_->publish(speed_msg);

    const double raw_x   = msg->pose.pose.position.x;
    const double raw_y   = msg->pose.pose.position.y;
    const double raw_yaw = quaternionToYaw(msg->pose.pose.orientation);

    // No-GPS mode: latch start pose on first message.
    if (!use_gps_ && !origin_set_) {
      origin_x_   = raw_x;
      origin_y_   = raw_y;
      origin_yaw_ = raw_yaw;
      origin_set_ = true;
      RCLCPP_INFO(
        get_logger(), "Start origin latched at (%.3f, %.3f, yaw=%.3f rad)",
        origin_x_, origin_y_, origin_yaw_);
    }

    double pub_x, pub_y, pub_yaw;

    if (use_gps_) {
      pub_x   = raw_x;
      pub_y   = raw_y;
      pub_yaw = raw_yaw;
    } else {
      // Rotate into the start frame so start heading = forward (+x).
      const double dx = raw_x - origin_x_;
      const double dy = raw_y - origin_y_;
      const double c  = std::cos(-origin_yaw_);
      const double s  = std::sin(-origin_yaw_);
      pub_x   = c * dx - s * dy;
      pub_y   = s * dx + c * dy;
      pub_yaw = raw_yaw - origin_yaw_;
    }

    // Publish current coordinate.
    geometry_msgs::msg::PoseStamped coord;
    coord.header.stamp    = msg->header.stamp;
    coord.header.frame_id = use_gps_ ? "map" : "start";
    coord.pose.position.x    = pub_x;
    coord.pose.position.y    = pub_y;
    coord.pose.position.z    = 0.0;
    coord.pose.orientation   = yawToQuaternion(pub_yaw);
    coordinate_pub_->publish(coord);

    // Nearest graph node (always in map-frame coordinates).
    if (!graph_nodes_.empty()) {
      const int nearest = findNearestNode(raw_x, raw_y);
      std_msgs::msg::Int32 node_msg;
      node_msg.data = nearest;
      node_pub_->publish(node_msg);
    }
  }

  bool use_gps_{true};
  std::string odom_topic_;
  std::string local_odom_topic_;
  std::string coordinate_topic_;
  std::string node_topic_;
  std::string distance_topic_;
  std::string speed_topic_;
  std::string graph_file_;

  bool origin_set_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};

  bool has_prev_local_{false};
  double prev_local_x_{0.0};
  double prev_local_y_{0.0};
  double total_distance_{0.0};

  std::map<int, GraphNode> graph_nodes_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr coordinate_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            node_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          speed_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
