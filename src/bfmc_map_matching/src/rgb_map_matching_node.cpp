#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

// Lane marking physical width (BFMC spec)
static constexpr double LANE_MARKING_WIDTH_M = 0.02;

class RgbMapMatchingNode : public rclcpp::Node
{
public:
  RgbMapMatchingNode()
  : Node("rgb_map_matching_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    image_topic_          = declare_parameter<std::string>("image_topic", "/oak/rgb/image_rect");
    camera_info_topic_    = declare_parameter<std::string>("camera_info_topic", "/oak/rgb/camera_info");
    gps_pose_topic_       = declare_parameter<std::string>("gps_pose_topic", "/automobile/gps/base_pose");
    map_match_pose_topic_ = declare_parameter<std::string>("map_match_pose_topic", "/automobile/map_match/base_pose");
    lane_mask_topic_      = declare_parameter<std::string>("lane_mask_topic", "/automobile/map_match/lane_mask");

    map_frame_    = declare_parameter<std::string>("map_frame", "map");
    base_frame_   = declare_parameter<std::string>("base_frame", "base_link");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "oak_rgb_camera_optical_frame");

    maps_dir_         = declare_parameter<std::string>("maps_dir", "");
    png_map_file_     = declare_parameter<std::string>("png_map_file", "Competition_track_graph.png");
    graphml_map_file_ = declare_parameter<std::string>("graphml_map_file", "Competition_track_graph.graphml");

    white_threshold_      = declare_parameter<int>("white_threshold", 180);
    saturation_threshold_ = declare_parameter<int>("saturation_threshold", 80);

    // GPS search radius around current estimate
    gps_search_radius_m_ = declare_parameter<double>("gps_search_radius_m", 3.0);
    map_y_flipped_       = declare_parameter<bool>("map_y_flipped", true);

    // Minimum correlation score (0–1) to accept the match
    min_correlation_     = declare_parameter<double>("min_correlation", 0.25);

    pose_xy_stddev_m_    = declare_parameter<double>("pose_xy_stddev_m", 0.20);
    pose_yaw_stddev_rad_ = declare_parameter<double>("pose_yaw_stddev_rad", 0.30);

    map_match_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      map_match_pose_topic_, 10);
    lane_mask_pub_ = create_publisher<sensor_msgs::msg::Image>(lane_mask_topic_, 10);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RgbMapMatchingNode::imageCallback, this, std::placeholders::_1));

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RgbMapMatchingNode::cameraInfoCallback, this, std::placeholders::_1));

    gps_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      gps_pose_topic_, 10,
      std::bind(&RgbMapMatchingNode::gpsPoseCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RgbMapMatchingNode::timerCallback, this));

    if (!loadMap()) {
      RCLCPP_ERROR(get_logger(), "Map load failed — check maps_dir and filenames");
    }

    RCLCPP_INFO(get_logger(), "RGB map matching node ready (10 Hz)");
    RCLCPP_INFO(get_logger(),
      "Method: project map markings into camera view, correlate with detected mask");
  }

private:
  // ── map loading ───────────────────────────────────────────────────────────

  bool loadMap()
  {
    if (!parseGraphMLBounds(maps_dir_ + "/" + graphml_map_file_)) {
      RCLCPP_ERROR(get_logger(), "GraphML parse failed");
      return false;
    }

    const cv::Mat raw = cv::imread(maps_dir_ + "/" + png_map_file_, cv::IMREAD_COLOR);
    if (raw.empty()) {
      RCLCPP_ERROR(get_logger(), "PNG load failed: %s",
        (maps_dir_ + "/" + png_map_file_).c_str());
      return false;
    }

    scale_x_ = static_cast<double>(raw.cols) / (world_x_max_ - world_x_min_);
    scale_y_ = static_cast<double>(raw.rows) / (world_y_max_ - world_y_min_);

    // Extract lane markings from map PNG
    cv::Mat hsv;
    cv::cvtColor(raw, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
      cv::Scalar(0, 0, white_threshold_),
      cv::Scalar(180, saturation_threshold_, 255),
      map_lane_mask_);
    cv::morphologyEx(map_lane_mask_, map_lane_mask_, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::morphologyEx(map_lane_mask_, map_lane_mask_, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    map_loaded_ = true;
    RCLCPP_INFO(get_logger(),
      "Map loaded: %dx%d px  scale(%.1f×%.1f px/m)  lane_px=%d",
      raw.cols, raw.rows, scale_x_, scale_y_, cv::countNonZero(map_lane_mask_));
    return true;
  }

  bool parseGraphMLBounds(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {return false;}

    world_x_min_ = world_y_min_ =  std::numeric_limits<double>::max();
    world_x_max_ = world_y_max_ = -std::numeric_limits<double>::max();

    std::string line;
    bool next_x = false, next_y = false;
    double cur_x = 0.0;
    bool has_x = false;

    while (std::getline(file, line)) {
      if      (line.find("key=\"d0\"") != std::string::npos) {next_x = true;}
      else if (line.find("key=\"d1\"") != std::string::npos) {next_y = true;}

      if (next_x || next_y) {
        const size_t s = line.find('>');
        const size_t e = line.find('<', s + 1);
        if (s != std::string::npos && e != std::string::npos) {
          const double v = std::stod(line.substr(s + 1, e - s - 1));
          if (next_x) {
            cur_x = v; has_x = true;
            world_x_min_ = std::min(world_x_min_, v);
            world_x_max_ = std::max(world_x_max_, v);
            next_x = false;
          } else {
            if (has_x) {
              world_y_min_ = std::min(world_y_min_, v);
              world_y_max_ = std::max(world_y_max_, v);
              has_x = false;
            }
            next_y = false;
          }
        }
      }
    }
    (void)cur_x;
    return world_x_min_ < world_x_max_;
  }

  // ── coordinate helpers ────────────────────────────────────────────────────

  cv::Point2d worldToMapPixel(double wx, double wy) const
  {
    return {
      (wx - world_x_min_) * scale_x_,
      map_y_flipped_
        ? (world_y_max_ - wy) * scale_y_
        : (wy - world_y_min_) * scale_y_
    };
  }

  cv::Point2d mapPixelToWorld(double px, double py) const
  {
    return {
      px / scale_x_ + world_x_min_,
      map_y_flipped_
        ? world_y_max_ - py / scale_y_
        : py / scale_y_ + world_y_min_
    };
  }

  // ── callbacks ─────────────────────────────────────────────────────────────

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      latest_image_       = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
      latest_image_stamp_ = msg->header.stamp;
      has_image_ = true;
    } catch (const cv_bridge::Exception &) {}
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!has_camera_info_) {
      camera_info_ = *msg;
      has_camera_info_ = true;
      RCLCPP_INFO(get_logger(),
        "Camera info received: fx=%.1f fy=%.1f  "
        "2cm marking = %.1f px at 1m, %.1f px at 2m",
        msg->k[0], msg->k[4],
        msg->k[0] * LANE_MARKING_WIDTH_M / 1.0,
        msg->k[0] * LANE_MARKING_WIDTH_M / 2.0);
    }
  }

  void gpsPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    gps_x_   = msg->pose.pose.position.x;
    gps_y_   = msg->pose.pose.position.y;
    // Snapshot the current odometry so we can propagate GPS forward by the delay
    try {
      const auto T = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      odom_x_at_gps_ = T.transform.translation.x;
      odom_y_at_gps_ = T.transform.translation.y;
    } catch (const tf2::TransformException &) {}
    has_gps_ = true;
  }

  // ── main 10 Hz loop ───────────────────────────────────────────────────────

  void timerCallback()
  {
    if (!map_loaded_ || !has_image_ || !has_camera_info_ || !has_gps_) {
      return;
    }

    const cv::Mat  bgr   = latest_image_.clone();
    const rclcpp::Time stamp = latest_image_stamp_;
    const int img_w = bgr.cols;
    const int img_h = bgr.rows;

    // ── camera intrinsics from camera_info ──────────────────────────────────
    const double fx = camera_info_.k[0];
    const double fy = camera_info_.k[4];
    const double cx = camera_info_.k[2];
    const double cy = camera_info_.k[5];

    // Pixel width of a 2 cm marking at distance Z:  pw = fx * 0.02 / Z
    // Used below to draw projected markings with the correct apparent width.

    // ── get robot pose in map frame via TF ──────────────────────────────────
    double robot_x = gps_x_, robot_y = gps_y_, robot_yaw = 0.0;
    cv::Matx33d R_w2c = cv::Matx33d::eye();
    cv::Vec3d   t_w2c(0, 0, 1);
    bool tf_ok = false;

    try {
      // map → camera optical frame (world-to-camera transform)
      const auto T_map_to_cam = tf_buffer_.lookupTransform(
        camera_frame_, map_frame_, tf2::TimePointZero);

      const auto & q = T_map_to_cam.transform.rotation;
      const double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
      R_w2c = cv::Matx33d(
        1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw),
          2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw),
          2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy));
      const auto & t = T_map_to_cam.transform.translation;
      t_w2c = cv::Vec3d(t.x, t.y, t.z);

      // map → base_link for robot pose
      const auto T_map_to_base = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
      robot_x   = T_map_to_base.transform.translation.x;
      robot_y   = T_map_to_base.transform.translation.y;
      const auto & qb = T_map_to_base.transform.rotation;
      robot_yaw = std::atan2(
        2.0*(qb.w*qb.z + qb.x*qb.y),
        1.0 - 2.0*(qb.y*qb.y + qb.z*qb.z));
      tf_ok = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed: %s", ex.what());
    }

    if (!tf_ok) {return;}

    // ── detect lane markings in camera image ────────────────────────────────
    cv::Mat hsv, cam_mask, cleaned;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
      cv::Scalar(0, 0, white_threshold_),
      cv::Scalar(180, saturation_threshold_, 255),
      cam_mask);
    cv::morphologyEx(cam_mask, cleaned, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::morphologyEx(cleaned, cleaned, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // Publish camera lane mask
    auto mask_msg = cv_bridge::CvImage({}, "mono8", cleaned).toImageMsg();
    mask_msg->header.stamp    = stamp;
    mask_msg->header.frame_id = camera_frame_;
    lane_mask_pub_->publish(*mask_msg);

    // ── build BEV canvas of camera lane detections ───────────────────────────
    // Canvas covers ±gps_search_radius_m_ around the GPS position at map scale.
    // Both the BEV and the local map crop share this coordinate system so the
    // homography translation directly gives the world position error.
    const int canvas_w = static_cast<int>(2.0 * gps_search_radius_m_ * scale_x_);
    const int canvas_h = static_cast<int>(2.0 * gps_search_radius_m_ * scale_y_);
    if (canvas_w < 10 || canvas_h < 10) {return;}

    // GPS-anchored but odometry-propagated centre: removes the ~1s GPS delay
    // (at 40 cm/s max speed that is up to 40 cm of canvas misalignment).
    const double centre_x = gps_x_ + (robot_x - odom_x_at_gps_);
    const double centre_y = gps_y_ + (robot_y - odom_y_at_gps_);

    const cv::Matx33d R_c2w   = R_w2c.t();
    const cv::Vec3d   C_world = -R_c2w * t_w2c;

    // Sample a grid of camera pixels in the lower half (ground visible),
    // intersect each ray with z=0, convert to canvas coords → compute H.
    std::vector<cv::Point2f> cam_pts_h, canvas_pts_h;
    for (float fu : {0.1f, 0.3f, 0.5f, 0.7f, 0.9f}) {
      for (float fv : {0.5f, 0.65f, 0.8f, 0.95f}) {
        const double pu = fu * img_w, pv = fv * img_h;
        const cv::Vec3d d_w = R_c2w * cv::Vec3d((pu - cx) / fx, (pv - cy) / fy, 1.0);
        if (std::abs(d_w[2]) < 1e-9) {continue;}
        const double t = -C_world[2] / d_w[2];
        if (t <= 0.0) {continue;}
        const double wx = C_world[0] + t * d_w[0];
        const double wy = C_world[1] + t * d_w[1];
        const double mx = (wx - (centre_x - gps_search_radius_m_)) * scale_x_;
        const double my = map_y_flipped_
          ? (centre_y + gps_search_radius_m_ - wy) * scale_y_
          : (wy - (centre_y - gps_search_radius_m_)) * scale_y_;
        if (mx < 0 || mx >= canvas_w || my < 0 || my >= canvas_h) {continue;}
        cam_pts_h.push_back({(float)pu, (float)pv});
        canvas_pts_h.push_back({(float)mx, (float)my});
      }
    }

    if (cam_pts_h.size() < 4) {
      RCLCPP_DEBUG(get_logger(), "Camera footprint does not overlap GPS ROI canvas");
      return;
    }

    const cv::Mat H_cam_to_canvas = cv::findHomography(cam_pts_h, canvas_pts_h, 0);
    if (H_cam_to_canvas.empty()) {return;}

    cv::Mat bev_cam;
    cv::warpPerspective(cleaned, bev_cam, H_cam_to_canvas, cv::Size(canvas_w, canvas_h));

    // ── crop local map around GPS ─────────────────────────────────────────────
    const cv::Point2d centre_map_px = worldToMapPixel(centre_x, centre_y);
    const int mx0 = static_cast<int>(std::round(centre_map_px.x)) - canvas_w / 2;
    const int my0 = static_cast<int>(std::round(centre_map_px.y)) - canvas_h / 2;
    cv::Mat local_map = cv::Mat::zeros(canvas_h, canvas_w, CV_8UC1);
    const int sx0 = std::max(0, mx0), sy0 = std::max(0, my0);
    const int sx1 = std::min(map_lane_mask_.cols, mx0 + canvas_w);
    const int sy1 = std::min(map_lane_mask_.rows, my0 + canvas_h);
    if (sx0 < sx1 && sy0 < sy1) {
      map_lane_mask_(cv::Rect(sx0, sy0, sx1 - sx0, sy1 - sy0))
        .copyTo(local_map(cv::Rect(sx0 - mx0, sy0 - my0, sx1 - sx0, sy1 - sy0)));
    }

    if (cv::countNonZero(bev_cam) < 50) {
      RCLCPP_DEBUG(get_logger(), "Too few BEV lane pixels");
      return;
    }
    if (cv::countNonZero(local_map) < 50) {
      RCLCPP_DEBUG(get_logger(), "Too few local map lane pixels in GPS ROI");
      return;
    }

    // ── AKAZE feature matching ────────────────────────────────────────────────
    std::vector<cv::KeyPoint> kp_bev, kp_map;
    cv::Mat desc_bev, desc_map;
    akaze_->detectAndCompute(bev_cam,   cv::noArray(), kp_bev, desc_bev);
    akaze_->detectAndCompute(local_map, cv::noArray(), kp_map, desc_map);

    if (desc_bev.empty() || desc_map.empty() ||
        kp_bev.size() < 4 || kp_map.size() < 4) {
      RCLCPP_DEBUG(get_logger(), "Too few features detected");
      return;
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(desc_bev, desc_map, knn, 2);

    std::vector<cv::Point2f> pts_bev, pts_map;
    for (const auto & m : knn) {
      if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance) {
        pts_bev.push_back(kp_bev[m[0].queryIdx].pt);
        pts_map.push_back(kp_map[m[0].trainIdx].pt);
      }
    }

    if (pts_bev.size() < 4) {
      RCLCPP_DEBUG(get_logger(), "Too few good matches: %zu", pts_bev.size());
      return;
    }

    // ── homography estimation (RANSAC) ────────────────────────────────────────
    // H maps local_map canvas → bev_cam canvas.
    // Both images are in the same coordinate system (map px, GPS-centred),
    // so H translation directly encodes the world position error.
    cv::Mat inlier_mask;
    const cv::Mat H = cv::findHomography(pts_map, pts_bev, cv::RANSAC, 3.0, inlier_mask);
    if (H.empty()) {
      RCLCPP_DEBUG(get_logger(), "Homography estimation failed");
      return;
    }

    const double inlier_ratio =
      static_cast<double>(cv::countNonZero(inlier_mask)) / pts_bev.size();
    if (inlier_ratio < min_correlation_) {
      RCLCPP_DEBUG(get_logger(), "Inlier ratio %.2f < %.2f threshold",
        inlier_ratio, min_correlation_);
      return;
    }

    // ── world pose correction ─────────────────────────────────────────────────
    // H translation in canvas pixels → world metres via map scale.
    const double h22      = H.at<double>(2, 2);
    const double tx_c     = H.at<double>(0, 2) / h22;
    const double ty_c     = H.at<double>(1, 2) / h22;
    const double world_dx = tx_c / scale_x_;
    const double world_dy = (map_y_flipped_ ? -ty_c : ty_c) / scale_y_;

    // Rotation component of H → yaw correction
    const double delta_yaw = std::atan2(
      H.at<double>(1, 0) / h22, H.at<double>(0, 0) / h22);

    const double corrected_x   = robot_x - world_dx;
    const double corrected_y   = robot_y - world_dy;
    const double corrected_yaw = robot_yaw - delta_yaw;

    // ── publish corrected pose ───────────────────────────────────────────────
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp    = stamp;
    out.header.frame_id = map_frame_;
    out.pose.pose.position.x = corrected_x;
    out.pose.pose.position.y = corrected_y;
    out.pose.pose.position.z = 0.0;

    const double half_yaw = 0.5 * corrected_yaw;
    out.pose.pose.orientation.x = 0.0;
    out.pose.pose.orientation.y = 0.0;
    out.pose.pose.orientation.z = std::sin(half_yaw);
    out.pose.pose.orientation.w = std::cos(half_yaw);

    out.pose.covariance.fill(0.0);
    // Scale uncertainty inversely with inlier ratio: weak match → larger covariance.
    const double cov_scale = 1.0 / inlier_ratio;
    out.pose.covariance[0]  = pose_xy_stddev_m_    * pose_xy_stddev_m_    * cov_scale;
    out.pose.covariance[7]  = pose_xy_stddev_m_    * pose_xy_stddev_m_    * cov_scale;
    out.pose.covariance[14] = 999.0;
    out.pose.covariance[21] = 999.0;
    out.pose.covariance[28] = 999.0;
    out.pose.covariance[35] = pose_yaw_stddev_rad_ * pose_yaw_stddev_rad_ * cov_scale;

    map_match_pose_pub_->publish(out);

    RCLCPP_DEBUG(get_logger(),
      "Map match: inliers=%.0f%%  correction(%.3f,%.3f)m  dyaw=%.2f deg",
      inlier_ratio * 100.0, world_dx, world_dy, delta_yaw * 180.0 / M_PI);
  }

  // ── parameters ────────────────────────────────────────────────────────────
  std::string image_topic_, camera_info_topic_, gps_pose_topic_;
  std::string map_match_pose_topic_, lane_mask_topic_;
  std::string map_frame_, base_frame_, camera_frame_;
  std::string maps_dir_, png_map_file_, graphml_map_file_;

  int    white_threshold_{180};
  int    saturation_threshold_{80};
  double gps_search_radius_m_{3.0};
  double min_correlation_{0.25};
  double pose_xy_stddev_m_{0.20};
  double pose_yaw_stddev_rad_{0.30};
  bool   map_y_flipped_{true};

  // ── map data ──────────────────────────────────────────────────────────────
  cv::Mat                     map_lane_mask_;
  cv::Ptr<cv::AKAZE>          akaze_{cv::AKAZE::create()};
  double world_x_min_{0}, world_x_max_{0};
  double world_y_min_{0}, world_y_max_{0};
  double scale_x_{1}, scale_y_{1};
  bool   map_loaded_{false};

  // ── runtime state ─────────────────────────────────────────────────────────
  cv::Mat      latest_image_;
  rclcpp::Time latest_image_stamp_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::CameraInfo camera_info_;
  double gps_x_{0}, gps_y_{0};
  double odom_x_at_gps_{0}, odom_y_at_gps_{0};  // odometry snapshot when GPS arrived
  bool   has_image_{false}, has_camera_info_{false}, has_gps_{false};

  // ── ROS handles ───────────────────────────────────────────────────────────
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr    camera_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_match_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr                        lane_mask_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbMapMatchingNode>());
  rclcpp::shutdown();
  return 0;
}
