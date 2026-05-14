# BFMC Localization — Topic Pipeline (Run Order)

Based on the data flow dependencies, here is the pipeline in run order.

---

## Run Order & Topics

### 1. `bfmc_state_odometry`
*(depends on: car hardware only)*

| Direction | Topic | Type |
|-----------|-------|------|
| SUB | `/automobile/encoder/speed` | `std_msgs/Float32` |
| SUB | `/automobile/encoder/distance` | `std_msgs/Float32` |
| SUB | `/automobile/IMU` | `sensor_msgs/Imu` |
| PUB | `/encoder_odom` | `nav_msgs/Odometry` |
| PUB | `/car/imu/data` | `sensor_msgs/Imu` |

---

### 2. `bfmc_isaac_visual_odom`
*(depends on: OAK-D camera driver only)*

| Direction | Topic | Type |
|-----------|-------|------|
| SUB | `/oak/rgb/image_rect` *(RGBD)* or `/oak/left/image_mono` + `/oak/right/image_mono` *(Stereo)* | `sensor_msgs/Image` |
| SUB | `/oak/rgb/camera_info` *(RGBD)* or `/oak/left/camera_info` + `/oak/right/camera_info` *(Stereo)* | `sensor_msgs/CameraInfo` |
| SUB | `/oak/stereo/image_raw` *(RGBD only)* | `sensor_msgs/Image` |
| SUB | `/oak/imu/data` *(Stereo only)* | `sensor_msgs/Imu` |
| SUB | `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` |
| PUB | `/visual_odom` | `nav_msgs/Odometry` |
| PUB | `/visual_odom_planar` | `nav_msgs/Odometry` |

> **Note:** Steps 1 and 2 can be launched in parallel — they are independent source nodes.

---

### 3. `bfmc_odometry_fusion`
*(depends on: `/encoder_odom`, `/car/imu/data` from step 1 and `/visual_odom_planar` from step 2)*

| Direction | Topic | Type |
|-----------|-------|------|
| SUB | `/visual_odom_planar` | `nav_msgs/Odometry` |
| SUB | `/encoder_odom` | `nav_msgs/Odometry` |
| SUB | `/car/imu/data` | `sensor_msgs/Imu` |
| PUB | `/odometry/local` | `nav_msgs/Odometry` |

---

### 4a. `bfmc_gps_position`
*(depends on: `/odometry/local` from step 3 + external AprilTag detection)*

| Direction | Topic | Type |
|-----------|-------|------|
| SUB | `/automobile/gps/tag_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| SUB | `/odometry/local` | `nav_msgs/Odometry` |
| PUB | `/automobile/gps/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` |

---

### 4b. `bfmc_map_matching`
*(depends on: `/odometry/local` from step 3 + OAK-D camera driver)*

| Direction | Topic | Type |
|-----------|-------|------|
| SUB | `/oak/rgb/image_raw` | `sensor_msgs/Image` |
| SUB | `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| SUB | `/odometry/local` | `nav_msgs/Odometry` |
| PUB | `/automobile/map_match/lane_mask` | `sensor_msgs/Image` |
| PUB | `/automobile/map_match/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` |

> **Note:** Steps 4a and 4b both depend only on `/odometry/local` and can be launched in parallel after step 3.
