#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class RgbMapMatchingNode : public rclcpp::Node
{
public:
  RgbMapMatchingNode()
  : Node("rgb_map_matching_node")
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/oak/rgb/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/oak/rgb/camera_info");
    local_odom_topic_ = declare_parameter<std::string>("local_odom_topic", "/odometry/local");

    map_match_pose_topic_ = declare_parameter<std::string>(
      "map_match_pose_topic", "/automobile/map_match/base_pose");

    lane_mask_topic_ = declare_parameter<std::string>(
      "lane_mask_topic", "/automobile/map_match/lane_mask");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    graphml_map_file_ = declare_parameter<std::string>(
      "graphml_map_file", "Competition_track_graph.graphml");

    png_map_file_ = declare_parameter<std::string>(
      "png_map_file", "Competition_track_graph.png");

    svg_map_file_ = declare_parameter<std::string>(
      "svg_map_file", "Track.svg");

    white_threshold_ = declare_parameter<int>("white_threshold", 180);
    saturation_threshold_ = declare_parameter<int>("saturation_threshold", 80);

    pose_xy_stddev_m_ = declare_parameter<double>("pose_xy_stddev_m", 0.30);
    pose_yaw_stddev_rad_ = declare_parameter<double>("pose_yaw_stddev_rad", 0.50);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RgbMapMatchingNode::imageCallback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&RgbMapMatchingNode::cameraInfoCallback, this, std::placeholders::_1));

    local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_,
      10,
      std::bind(&RgbMapMatchingNode::localOdomCallback, this, std::placeholders::_1));

    lane_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(
      lane_mask_topic_, 10);

    map_match_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        map_match_pose_topic_, 10);

    RCLCPP_INFO(get_logger(), "RGB map matching node started");
    RCLCPP_INFO(get_logger(), "Image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Camera info topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Local odom topic: %s", local_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "GraphML map: %s", graphml_map_file_.c_str());
    RCLCPP_INFO(get_logger(), "PNG map: %s", png_map_file_.c_str());
    RCLCPP_INFO(get_logger(), "SVG map: %s", svg_map_file_.c_str());
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    latest_camera_info_ = *msg;
    has_camera_info_ = true;
  }

  void localOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    has_local_odom_ = true;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "cv_bridge conversion failed: %s",
        e.what());
      return;
    }

    const cv::Mat bgr = cv_ptr->image;
    if (bgr.empty()) {
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat white_mask;
    cv::inRange(
      hsv,
      cv::Scalar(0, 0, white_threshold_),
      cv::Scalar(180, saturation_threshold_, 255),
      white_mask);

    cv::Mat cleaned;
    cv::morphologyEx(
      white_mask,
      cleaned,
      cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    cv::morphologyEx(
      cleaned,
      cleaned,
      cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    auto mask_msg = cv_bridge::CvImage(
      msg->header,
      "mono8",
      cleaned).toImageMsg();

    lane_mask_pub_->publish(*mask_msg);

    publishPlaceholderMapMatchPose(msg->header.stamp);
  }

  void publishPlaceholderMapMatchPose(const rclcpp::Time & stamp)
  {
    if (!has_local_odom_) {
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = map_frame_;

    // Placeholder for first package test.
    // Later this will become the corrected pose from real image-vs-map matching.
    out.pose.pose = latest_odom_.pose.pose;

    out.pose.covariance.fill(0.0);

    const double xy_var = pose_xy_stddev_m_ * pose_xy_stddev_m_;
    const double yaw_var = pose_yaw_stddev_rad_ * pose_yaw_stddev_rad_;

    out.pose.covariance[0] = xy_var;
    out.pose.covariance[7] = xy_var;
    out.pose.covariance[14] = 999.0;
    out.pose.covariance[21] = 999.0;
    out.pose.covariance[28] = 999.0;
    out.pose.covariance[35] = yaw_var;

    map_match_pose_pub_->publish(out);
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string local_odom_topic_;
  std::string map_match_pose_topic_;
  std::string lane_mask_topic_;
  std::string map_frame_;
  std::string base_frame_;

  std::string graphml_map_file_;
  std::string png_map_file_;
  std::string svg_map_file_;

  int white_threshold_{180};
  int saturation_threshold_{80};

  double pose_xy_stddev_m_{0.30};
  double pose_yaw_stddev_rad_{0.50};

  bool has_camera_info_{false};
  bool has_local_odom_{false};

  sensor_msgs::msg::CameraInfo latest_camera_info_;
  nav_msgs::msg::Odometry latest_odom_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lane_mask_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_match_pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbMapMatchingNode>());
  rclcpp::shutdown();
  return 0;
}
