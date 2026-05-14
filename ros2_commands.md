# ROS 2 Package Commands — BFMC Localization

## Docker (required for Isaac ROS visual odometry)

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev --docker_arg "--privileged"
```

Inside the container:
```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/isaac_ros-dev/install/setup.bash
```

---

## Master Launch (all packages at once)

```bash
# Show all available arguments
ros2 launch ~/ros2_workspaces/BFMC_Localization/src/bfmc_localization.launch.py --show-args
```

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `use_gps` | `true` | `true` / `false` | Enable GPS fusion |
| `camera_mode` | `stereo` | `stereo` / `rgbd` | Visual odometry camera mode |

```bash
# Default — GPS on, stereo camera
ros2 launch ~/ros2_workspaces/BFMC_Localization/src/bfmc_localization.launch.py

# GPS off
ros2 launch ~/ros2_workspaces/BFMC_Localization/src/bfmc_localization.launch.py use_gps:=false

# RGBD camera
ros2 launch ~/ros2_workspaces/BFMC_Localization/src/bfmc_localization.launch.py camera_mode:=rgbd

# GPS off + RGBD
ros2 launch ~/ros2_workspaces/BFMC_Localization/src/bfmc_localization.launch.py use_gps:=false camera_mode:=rgbd
```

---

## Setup — source first (every terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_workspaces/BFMC_Localization/install/setup.bash
```

---

## 1. `bfmc_state_odometry`

```bash
# Run
ros2 launch bfmc_state_odometry state_odometry.launch.py

# Topics
ros2 topic info /encoder_odom
ros2 topic info /car/imu/data

# Data
ros2 topic echo /encoder_odom
ros2 topic echo /car/imu/data

# Msg types
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show sensor_msgs/msg/Imu
```

---

## 2. `bfmc_isaac_visual_odom`

```bash
# Run (choose one)
ros2 launch bfmc_isaac_visual_odom isaac_visual_odom_stereo.launch.py
ros2 launch bfmc_isaac_visual_odom isaac_visual_odom_rgbd.launch.py

# Topics
ros2 topic info /visual_odom
ros2 topic info /visual_odom_planar

# Data
ros2 topic echo /visual_odom
ros2 topic echo /visual_odom_planar

# Msg type
ros2 interface show nav_msgs/msg/Odometry
```

---

## 3. `bfmc_odometry_fusion`

```bash
# Run
ros2 launch bfmc_odometry_fusion local_ekf.launch.py

# Topics
ros2 topic info /odometry/local
ros2 topic info /odom_distance
ros2 topic info /odom_velocity

# Data
ros2 topic echo /odometry/local
ros2 topic echo /odom_distance
ros2 topic echo /odom_velocity

# Msg types
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show std_msgs/msg/Float32
```

---

## 4. `bfmc_gps_position`

```bash
# Run
ros2 launch bfmc_gps_position gps_position.launch.py

# Topics
ros2 topic info /automobile/gps/base_pose

# Data
ros2 topic echo /automobile/gps/base_pose

# Msg type
ros2 interface show geometry_msgs/msg/PoseWithCovarianceStamped
```

---

## 5. `bfmc_map_matching`

```bash
# Run
ros2 launch bfmc_map_matching rgb_map_matching.launch.py

# Topics
ros2 topic info /automobile/map_match/base_pose
ros2 topic info /automobile/map_match/lane_mask

# Data
ros2 topic echo /automobile/map_match/base_pose
ros2 topic echo /automobile/map_match/lane_mask --no-arr   # suppress image bytes

# Msg types
ros2 interface show geometry_msgs/msg/PoseWithCovarianceStamped
ros2 interface show sensor_msgs/msg/Image
```

---

## 6. `bfmc_global_localization`

```bash
# Run
ros2 launch bfmc_global_localization global_ekf.launch.py              # GPS on (default)
ros2 launch bfmc_global_localization global_ekf.launch.py use_gps:=false  # GPS off

# Topics
ros2 topic info /odometry/global
ros2 topic info /automobile/sign/base_pose
ros2 topic info /automobile/current_coordinate
ros2 topic info /automobile/current_node

# Data
ros2 topic echo /odometry/global
ros2 topic echo /automobile/sign/base_pose
ros2 topic echo /automobile/current_coordinate
ros2 topic echo /automobile/current_node

# Msg types
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show geometry_msgs/msg/PoseWithCovarianceStamped
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 interface show std_msgs/msg/Int32
```

---

## Useful extras

```bash
# See all active topics at once
ros2 topic list

# Check publish rate of any topic
ros2 topic hz /odometry/local
ros2 topic hz /automobile/current_coordinate

# See full topic graph
ros2 run rqt_graph rqt_graph
```
