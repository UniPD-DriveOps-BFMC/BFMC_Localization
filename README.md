# BFMC Localization ROS 2 Workspace

ROS 2 localization stack for the Bosch Future Mobility Challenge autonomous vehicle platform.

This workspace runs inside the **Isaac ROS Docker container** (ROS 2 Humble) on the **Jetson Orin Nano**. The host-side ROS 2 Jazzy installation should not be mixed with the Docker Humble environment.

---

## Package Overview

| # | Package | Output topics |
|---|---------|---------------|
| 0 | `automobile_imu` | `/automobile/imu/data` |
| 1 | `bfmc_car_description` | TF frames: `base_link`, `imu_link`, `oak_*`, `gps_tag_link` |
| 2 | `bfmc_isaac_visual_odom` | `/visual_odom`, `/visual_odom_planar` |
| 3 | `bfmc_state_odometry` | `/encoder_odom`, `/car/imu/data` |
| 4 | `bfmc_odometry_fusion` | `/odometry/local`, `/odom_distance`, `/odom_velocity` |
| 5 | `bfmc_gps_position` | `/automobile/gps/base_pose` |
| 6 | `bfmc_map_matching` | `/automobile/map_match/base_pose`, `/automobile/map_match/lane_mask` |
| 7 | `bfmc_global_localization` | `/odometry/global`, `/automobile/current_coordinate`, `/automobile/current_node`, `/automobile/sign/base_pose`, `/automobile/total_distance`, `/automobile/current_speed` |

---

## External Inputs

| Topic | Type | Source | Notes |
|-------|------|--------|-------|
| `/automobile/encoder/speed` | `std_msgs/Float32` | Car firmware | m/s |
| `/automobile/encoder/distance` | `std_msgs/Float32` | Car firmware | metres |
| `/automobile/localisation` | `bfmc_gps_position/GpsTagPose` | GPS team | fields: `x`, `y`, `z` (metres), `quality` (0–100) |
| `/oak/left/image_mono` + `/oak/right/image_mono` | `sensor_msgs/Image` | Camera driver | stereo mode |
| `/oak/rgb/image_rect` + `/oak/rgb/camera_info` | `sensor_msgs/Image` | Camera driver | RGB rectified + intrinsics |
| `/traffic/detection` | `std_msgs/String` | Detection team | JSON: `{"sign": "stop", "distance_m": 0.5, "confidence": 0.9}` |

> BNO055 IMU is handled internally by `automobile_imu` over I2C — no external dependency.

## Final Outputs

| Topic | Type | Meaning |
|-------|------|---------|
| `/odometry/local` | `nav_msgs/Odometry` | Position + velocity relative to start position (odom frame, 50 Hz) |
| `/odometry/global` | `nav_msgs/Odometry` | Position + velocity relative to GPS (0, 0) (map frame, 50 Hz) |
| `/automobile/current_coordinate` | `geometry_msgs/PoseStamped` | X, Y in metres relative to GPS (0, 0); relative to start if `use_gps:=false` |
| `/automobile/current_node` | `std_msgs/Int32` | Current node ID in the competition track GraphML map |
| `/automobile/total_distance` | `std_msgs/Float64` | Total path length travelled from start (metres) |
| `/automobile/current_speed` | `std_msgs/Float64` | Current speed from fused odometry (m/s) |
| `/automobile/sign/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Sign-derived pose in map frame |

---

## TF Tree

```text
map
 └── odom                        ← global EKF (bfmc_global_localization)
      └── base_link               ← local EKF (bfmc_odometry_fusion)
           ├── imu_link            ← BNO055 IMU (robot_state_publisher)
           ├── gps_tag_link        ← GPS antenna mount (robot_state_publisher)
           └── camera_mount_link
                └── camera_base_link
                     └── oak-d-base_frame   ← depthai_descriptions
                          ├── oak_imu_frame
                          ├── oak_model_origin
                          ├── oak_left_camera_frame
                          │    └── oak_left_camera_optical_frame
                          ├── oak_rgb_camera_frame
                          │    └── oak_rgb_camera_optical_frame   ← used by map matching
                          └── oak_right_camera_frame
                               └── oak_right_camera_optical_frame
```

---

## Topic Pipeline

```text
Hardware                   Package                    Output
─────────────────────────────────────────────────────────────────────────────
Encoder + IMU  ──────────► bfmc_state_odometry    ──► /encoder_odom
                                                   ──► /car/imu/data

OAK-D camera   ──────────► bfmc_isaac_visual_odom ──► /visual_odom_planar

/encoder_odom ┐
/car/imu/data ├────────────► bfmc_odometry_fusion ──► /odometry/local
/visual_odom_planar ┘

/automobile/localisation ───► bfmc_gps_position    ──► /automobile/gps/base_pose
/oak/rgb/image_rect ────────► bfmc_map_matching   ──► /automobile/map_match/base_pose

/odometry/local ┐
/automobile/gps/base_pose  ├─► bfmc_global_localization ──► /odometry/global
/automobile/map_match/...  │                             ──► /automobile/current_coordinate
/traffic/detection         ┘                             ──► /automobile/current_node
                                                         ──► /automobile/sign/base_pose
                                                         ──► /automobile/total_distance
                                                         ──► /automobile/current_speed
```

### Per-package topic details

#### `bfmc_state_odometry`
| Dir | Topic | Type |
|-----|-------|------|
| SUB | `/automobile/encoder/speed` | `std_msgs/Float32` |
| SUB | `/automobile/encoder/distance` | `std_msgs/Float32` |
| SUB | `/automobile/imu/data` | `sensor_msgs/Imu` |
| PUB | `/encoder_odom` | `nav_msgs/Odometry` |
| PUB | `/car/imu/data` | `sensor_msgs/Imu` |

#### `bfmc_isaac_visual_odom`
| Dir | Topic | Type |
|-----|-------|------|
| SUB | `/oak/left/image_mono` + `/oak/right/image_mono` *(stereo)* | `sensor_msgs/Image` |
| SUB | `/oak/rgb/image_rect` + `/oak/stereo/image_raw` *(rgbd)* | `sensor_msgs/Image` |
| SUB | `/oak/imu/data` | `sensor_msgs/Imu` |
| SUB | `/visual_slam/tracking/odometry` *(internal — from Isaac SLAM node)* | `nav_msgs/Odometry` |
| PUB | `/visual_odom` | `nav_msgs/Odometry` |
| PUB | `/visual_odom_planar` | `nav_msgs/Odometry` |

#### `bfmc_odometry_fusion`
| Dir | Topic | Type |
|-----|-------|------|
| SUB | `/encoder_odom` | `nav_msgs/Odometry` |
| SUB | `/car/imu/data` | `sensor_msgs/Imu` |
| SUB | `/visual_odom_planar` | `nav_msgs/Odometry` |
| PUB | `/odometry/local` | `nav_msgs/Odometry` |
| PUB | `/odom_distance` | `std_msgs/Float32` |
| PUB | `/odom_velocity` | `std_msgs/Float32` |

#### `bfmc_gps_position`
| Dir | Topic | Type |
|-----|-------|------|
| SUB | `/automobile/localisation` | `bfmc_gps_position/GpsTagPose` |
| SUB | `/odometry/local` | `nav_msgs/Odometry` |
| PUB | `/automobile/gps/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` |

#### `bfmc_map_matching`
| Dir | Topic | Type |
|-----|-------|------|
| SUB | `/oak/rgb/image_rect` | `sensor_msgs/Image` |
| SUB | `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| SUB | `/automobile/gps/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| TF  | `map` → `oak_rgb_camera_optical_frame` | BEV projection |
| TF  | `map` → `base_link` | robot pose |
| PUB | `/automobile/map_match/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| PUB | `/automobile/map_match/lane_mask` | `sensor_msgs/Image` |

#### `bfmc_global_localization`
| Dir | Topic | Type | Node |
|-----|-------|------|------|
| SUB | `/odometry/local` | `nav_msgs/Odometry` | global EKF (vx, ωyaw) |
| SUB | `/automobile/gps/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` | global EKF |
| SUB | `/automobile/map_match/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` | global EKF |
| SUB | `/automobile/sign/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` | global EKF |
| SUB | `/traffic/detection` | `std_msgs/String` (JSON) | sign_localization_node |
| SUB | `/odometry/global` | `nav_msgs/Odometry` | sign_localization_node, position_publisher_node |
| SUB | `/odometry/local` | `nav_msgs/Odometry` | position_publisher_node (distance) |
| PUB | `/odometry/global` | `nav_msgs/Odometry` | global EKF (50 Hz) |
| PUB | `/automobile/sign/base_pose` | `geometry_msgs/PoseWithCovarianceStamped` | sign_localization_node |
| PUB | `/automobile/current_coordinate` | `geometry_msgs/PoseStamped` | position_publisher_node |
| PUB | `/automobile/current_node` | `std_msgs/Int32` | position_publisher_node |
| PUB | `/automobile/total_distance` | `std_msgs/Float64` | position_publisher_node |
| PUB | `/automobile/current_speed` | `std_msgs/Float64` | position_publisher_node |

---

## Shell Scripts

### `run_localization.sh` — Jetson launcher

Runs on the **Jetson**. Syncs packages and the Dockerfile into `isaac_ros-dev`, writes `run_bfmc.sh` into the workspace, and starts the Isaac ROS Docker container. Once the container shell opens, run the generated script to build and launch everything.

```bash
# Normal start — EKF seeded to the physical start box
./run_localization.sh --normal-start

# Random start — EKF seeded from first map-matched node
./run_localization.sh --random-start

# Without GPS
./run_localization.sh --normal-start --use-gps false

# RGBD camera mode
./run_localization.sh --normal-start --camera-mode rgbd

# Skip rebuild (already built, just relaunch)
./run_localization.sh --normal-start --no-rebuild

# Skip both sync and rebuild (fastest relaunch)
./run_localization.sh --normal-start --no-sync --no-rebuild
```

Once the container shell opens (`admin@ubuntu:/workspaces/isaac_ros-dev$`), run:

```bash
sudo bash /workspaces/isaac_ros-dev/run_bfmc.sh
```

This single command sources ROS, fixes I2C permissions, builds the BFMC packages, launches the localization stack, and starts the path planner — all with the options passed to `run_localization.sh`.

| Option | Default | Description |
|--------|---------|-------------|
| `--normal-start` | *(default)* | Seed EKF to the physical start box; begin path from node 472 |
| `--random-start` | off | Wait for `/automobile/current_node` and begin from that position |
| `--use-gps true\|false` | `true` | Enable GPS fusion |
| `--camera-mode stereo\|rgbd` | `stereo` | Visual odometry camera mode |
| `--no-rebuild` | off | Skip colcon build step |
| `--no-sync` | off | Skip syncing packages to isaac_ros-dev |
| `--i2c-bus N` | `7` | I2C bus number for BNO055 IMU |

---

### `monitor_localization.sh` — Local machine monitor

Runs on a **local machine** connected to the same LAN as the Jetson. Records all final output topics to a rosbag and optionally opens a `tmux` session with live topic echo panes.

```bash
# Rosbag only
./monitor_localization.sh

# Rosbag + live topic viewer
./monitor_localization.sh --topic-publish

# Custom domain ID (must match Jetson's ROS_DOMAIN_ID)
./monitor_localization.sh --topic-publish --domain-id 0

# Custom bag save location
./monitor_localization.sh --bag-dir /media/usb/bfmc_bags

# No bag, viewer only
./monitor_localization.sh --topic-publish --no-bag
```

| Option | Default | Description |
|--------|---------|-------------|
| `--topic-publish` | off | Open tmux with live topic echo panes |
| `--domain-id N` | `0` | ROS_DOMAIN_ID — must match Jetson |
| `--bag-dir DIR` | `~/bfmc_bags` | Directory to save rosbags |
| `--no-bag` | off | Disable rosbag recording |
| `--ros-distro NAME` | `jazzy` | ROS 2 distro on local machine |

**Rosbag** is saved to `~/bfmc_bags/bfmc_YYYYMMDD_HHMMSS/` and stops cleanly on Ctrl+C.

**tmux window layout** (with `--topic-publish`):

```
Window 0 "outputs"       Window 1 "odometry"      Window 2 "sensors"
┌────────────┬────────┐   ┌──────────┬──────────┐   ┌──────────┬──────────┐
│ coordinate │  node  │   │  local   │  global  │   │   GPS    │ map match│
├────────────┼────────┤   │  odom    │  odom    │   │  pose    ├──────────┤
│   speed    │  dist  │   │          │          │   │          │   sign   │
└────────────┴────────┘   └──────────┴──────────┘   └──────────┴──────────┘

Switch windows : Ctrl+B then 0 / 1 / 2
Detach         : Ctrl+B then D
Kill session   : tmux kill-session -t bfmc_monitor
```

**Topics recorded:**

| Group | Topic | Content |
|-------|-------|---------|
| Hardware inputs | `/automobile/encoder/speed` | Wheel encoder speed (m/s) |
| | `/automobile/encoder/distance` | Wheel encoder distance (m) |
| | `/automobile/imu/data` | Raw BNO055 IMU |
| | `/automobile/localisation` | Raw GPS tag pose from car firmware |
| | `/visual_slam/tracking/odometry` | Isaac ROS visual SLAM output |
| | `/oak/rgb/camera_info` | Camera intrinsics |
| | `/oak/rgb/image_rect` | Rectified RGB image *(large — remove if storage limited)* |
| | `/traffic/detection` | Sign detection JSON |
| Internal | `/car/imu/data` | IMU republished with `imu_link` frame |
| | `/encoder_odom` | Encoder odometry |
| | `/visual_odom` | 3D visual odometry |
| | `/visual_odom_planar` | Flattened 2D visual odometry |
| | `/odometry/local` | Local EKF output |
| | `/automobile/gps/base_pose` | GPS pose in map frame |
| | `/automobile/map_match/base_pose` | Map-matched pose |
| | `/automobile/map_match/lane_mask` | Lane mask image |
| | `/automobile/sign/base_pose` | Sign-derived pose |
| | `/odometry/global` | Global EKF output |
| | `/odom_distance` | Cumulative distance from local odom |
| | `/odom_velocity` | Speed from local odom |
| Final outputs | `/automobile/current_coordinate` | x, y position (map or start frame) |
| | `/automobile/current_node` | Current graph node ID |
| | `/automobile/current_speed` | Speed in m/s |
| | `/automobile/total_distance` | Total path length from start (m) |
| Transforms | `/tf` | Dynamic transforms |
| | `/tf_static` | Static transforms (required for playback) |

**Network setup** — set `ROS_DOMAIN_ID` on both machines before running:

```bash
# On Jetson (inside Docker, before ros2 launch)
export ROS_DOMAIN_ID=0

# monitor_localization.sh passes --domain-id automatically
./monitor_localization.sh --topic-publish --domain-id 0
```

If topic discovery fails, check that UDP multicast is not blocked by a firewall between the two machines.

---

## 1. Install Isaac ROS on Jetson (first time only)

### 1.1 Fix host GPU access

```bash
nvidia-smi
```

If it fails, fix the NVIDIA driver first. Then:

```bash
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Test:

```bash
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

### 1.2 Clone isaac_ros_common with Git LFS

```bash
sudo apt install -y git-lfs
git lfs install

mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src

git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
cd isaac_ros_common
git lfs pull
```

### 1.3 Create the Isaac ROS config

Tells `run_dev.sh` to include the BFMC custom Docker layer:

```bash
cat > ~/.isaac_ros_common-config <<'EOF'
CONFIG_IMAGE_KEY="ros2_humble.bfmc"
CONFIG_DOCKER_SEARCH_DIRS=("$HOME/workspaces/isaac_ros-dev/docker")
EOF
```

Expected image key after this: `aarch64.ros2_humble.bfmc`

### 1.4 Create the docker directory

`run_localization.sh` syncs `docker/Dockerfile.bfmc` from the repo into `isaac_ros-dev/docker/` automatically on every run. Just create the target directory:

```bash
mkdir -p ~/workspaces/isaac_ros-dev/docker
```

---

## 2. First Time Compiling (inside Docker)

### 2.1 Build Isaac Visual SLAM dependencies (once)

This must complete before the BFMC packages can be built. Run it manually inside the container — takes ~10 minutes on first run.

Enter the container first (see step 3.2 below), then:

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to isaac_ros_visual_slam

source install/setup.bash
```

> If this fails at `gxf_isaac_image_flip` with `magic_enum::magic_enum not found`, the BFMC Dockerfile was not applied — check that the image key is `aarch64.ros2_humble.bfmc`.

### 2.2 First full run

After the SLAM dependencies are built, use `run_localization.sh` for everything:

```bash
cd ~/workspaces/BFMC_Localization
./run_localization.sh --normal-start
```

When the container shell opens:

```bash
sudo bash /workspaces/isaac_ros-dev/run_bfmc.sh
```

### 2.3 Verify

```bash
ros2 pkg list | grep -E "automobile_imu|bfmc"
```

### Clean rebuild (when build state is inconsistent)

```bash
cd /workspaces/isaac_ros-dev

rm -rf \
  build/automobile_imu           install/automobile_imu \
  build/bfmc_car_description     install/bfmc_car_description \
  build/bfmc_isaac_visual_odom   install/bfmc_isaac_visual_odom \
  build/bfmc_state_odometry      install/bfmc_state_odometry \
  build/bfmc_odometry_fusion     install/bfmc_odometry_fusion \
  build/bfmc_gps_position        install/bfmc_gps_position \
  build/bfmc_map_matching        install/bfmc_map_matching \
  build/bfmc_global_localization install/bfmc_global_localization
```

Then run `run_bfmc.sh` again.

---

## 3. Daily Workflow

### 3.1 Pull and launch (on Jetson host)

```bash
cd ~/workspaces/BFMC_Localization
git pull
./run_localization.sh --normal-start
```

`run_localization.sh` re-syncs all packages and the Dockerfile, regenerates `run_bfmc.sh`, and starts the container.

### 3.2 Enter Docker

When `run_localization.sh` drops to a shell, you are inside the container (`admin@ubuntu:/workspaces/isaac_ros-dev$`). Run:

```bash
sudo bash /workspaces/isaac_ros-dev/run_bfmc.sh
```

To open a second terminal into the same running container:

```bash
docker ps --format "table {{.Names}}\t{{.Status}}"
docker exec -it isaac_ros_dev-aarch64-container bash
```

Then source ROS manually in that terminal:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

### 3.3 Rebuild changed packages (inside container, without relaunching)

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
    automobile_imu \
    bfmc_car_description \
    bfmc_isaac_visual_odom \
    bfmc_state_odometry \
    bfmc_odometry_fusion \
    bfmc_gps_position \
    bfmc_map_matching \
    bfmc_global_localization \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

To rebuild only one specific package:

```bash
colcon build --symlink-install \
  --packages-select <package_name> \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

### 3.4 Run the full stack — master launch

Runs all packages in dependency order (TF tree, IMU, visual odom, EKF, GPS, map matching, global localization):

```bash
ros2 launch bfmc_global_localization bfmc_localization.launch.py
```

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `use_gps` | `true` | `true` / `false` | Enable GPS fusion |
| `camera_mode` | `stereo` | `stereo` / `rgbd` | Visual odometry camera mode |

```bash
# No GPS
ros2 launch bfmc_global_localization bfmc_localization.launch.py use_gps:=false

# RGBD camera
ros2 launch bfmc_global_localization bfmc_localization.launch.py camera_mode:=rgbd

# No GPS + RGBD
ros2 launch bfmc_global_localization bfmc_localization.launch.py use_gps:=false camera_mode:=rgbd
```

### 3.5 Run packages individually (manual order)

#### IMU driver
```bash
ros2 run automobile_imu bno055_imu_node
```

#### TF tree
```bash
ros2 launch bfmc_car_description robot_state_publisher.launch.py
```

#### Isaac Visual Odometry
```bash
ros2 launch bfmc_isaac_visual_odom isaac_visual_odom_stereo.launch.py
# or
ros2 launch bfmc_isaac_visual_odom isaac_visual_odom_rgbd.launch.py
```

#### Encoder + state odometry
```bash
ros2 launch bfmc_state_odometry state_odometry.launch.py
```

#### Local EKF fusion
```bash
ros2 launch bfmc_odometry_fusion local_ekf.launch.py
```

#### GPS position
```bash
ros2 launch bfmc_gps_position gps_position.launch.py
```

#### Map matching
```bash
ros2 launch bfmc_map_matching rgb_map_matching.launch.py
```

#### Global localization
```bash
ros2 launch bfmc_global_localization global_ekf.launch.py
# or without GPS:
ros2 launch bfmc_global_localization global_ekf.launch.py use_gps:=false
```

### 3.6 Verify topics and TF

Check all localization topics:

```bash
ros2 topic list | grep -E "imu|visual|odom|gps|map_match|sign|current"
```

Expected active topics:

```text
/automobile/imu/data
/car/imu/data
/encoder_odom
/visual_odom
/visual_odom_planar
/odometry/local
/odom_distance
/odom_velocity
/automobile/gps/base_pose
/automobile/map_match/base_pose
/automobile/map_match/lane_mask
/automobile/sign/base_pose
/odometry/global
/automobile/current_coordinate
/automobile/current_node
/automobile/total_distance
/automobile/current_speed
```

Check publish rates:

```bash
ros2 topic hz /automobile/imu/data
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global
ros2 topic hz /automobile/current_coordinate
```

Check TF frames:

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link gps_tag_link
ros2 run tf2_ros tf2_echo base_link oak_left_camera_optical_frame
ros2 run tf2_tools view_frames
```

Check ROS graph:

```bash
ros2 run rqt_graph rqt_graph
```

---

## Debugging Commands

### Topic inspection

```bash
# List all active topics
ros2 topic list

# Check publish rate
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global
ros2 topic hz /automobile/imu/data
ros2 topic hz /visual_odom_planar
ros2 topic hz /automobile/current_coordinate

# Print one message
ros2 topic echo /encoder_odom --once
ros2 topic echo /automobile/imu/data --once
ros2 topic echo /odometry/local --once
ros2 topic echo /odometry/global --once
ros2 topic echo /automobile/gps/base_pose --once
ros2 topic echo /automobile/map_match/base_pose --once
ros2 topic echo /automobile/current_coordinate --once
ros2 topic echo /automobile/current_node --once
ros2 topic echo /automobile/sign/base_pose --once

# Print image topic without raw bytes
ros2 topic echo /automobile/map_match/lane_mask --no-arr

# Check publisher/subscriber info
ros2 topic info /odometry/local
ros2 topic info /visual_odom_planar
ros2 topic info /automobile/gps/base_pose
ros2 topic info /automobile/map_match/base_pose
ros2 topic info /odometry/global
```

### Message type inspection

```bash
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show sensor_msgs/msg/Imu
ros2 interface show geometry_msgs/msg/PoseWithCovarianceStamped
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 interface show std_msgs/msg/Float32
ros2 interface show std_msgs/msg/Int32
ros2 interface show sensor_msgs/msg/Image
```

### TF inspection

```bash
# Check individual transforms
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link gps_tag_link
ros2 run tf2_ros tf2_echo base_link oak_left_camera_optical_frame
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom

# Save TF tree to PDF
ros2 run tf2_tools view_frames
```

### Node and graph inspection

```bash
# List all active nodes
ros2 node list

# Show node details
ros2 node info /gps_tag_to_base_node
ros2 node info /ekf_filter_node

# Full ROS graph
ros2 run rqt_graph rqt_graph
```

### IMU hardware

```bash
# Check I2C devices
ls -l /dev/i2c*
sudo chmod 666 /dev/i2c-7
i2cdetect -y -r 7
# Expected BNO055 at address 0x28:
# 20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
```

### Docker

```bash
# List running containers
docker ps --format "table {{.Names}}\t{{.Status}}"

# Enter a running container
docker exec -it --user admin <container_name> bash

# Check which image the container is using
docker inspect <container_name> | grep Image

# Check Docker image was built with BFMC layer
docker history isaac_ros_dev-aarch64 | head -20
```

### Parallelism notes

Steps that can run in parallel once their inputs are ready:

- `bfmc_state_odometry` and `bfmc_isaac_visual_odom` are **independent** — launch both at the same time
- `bfmc_gps_position` and `bfmc_map_matching` both depend only on `/odometry/local` — launch both at the same time after `bfmc_odometry_fusion` is running

---

## Common Problems

### `i2cdetect: command not found`

The BFMC Docker layer was not used. Check that `~/.isaac_ros_common-config` sets `CONFIG_IMAGE_KEY="ros2_humble.bfmc"` and that `Dockerfile.bfmc` exists in `~/workspaces/isaac_ros-dev/docker/`.

### `Permission denied: /dev/i2c-7`

```bash
sudo chmod 666 /dev/i2c-7
```

### `ModuleNotFoundError: No module named smbus2`

The BFMC Docker layer was not applied. Rebuild the image.

### `magic_enum::magic_enum target was not found`

The Isaac GXF packages require `magic_enum`. It must be installed from source in `Dockerfile.bfmc`. Do not use `libmagic-enum-dev` — it is not available in the Jetson apt repositories.

### Build shows only `aarch64.ros2_humble` (missing `.bfmc`)

Check `~/.isaac_ros_common-config`:

```bash
cat ~/.isaac_ros_common-config
# Should contain:
# CONFIG_IMAGE_KEY="ros2_humble.bfmc"
# CONFIG_DOCKER_SEARCH_DIRS=("$HOME/workspaces/isaac_ros-dev/docker")
```

Check the Dockerfile exists:

```bash
ls ~/workspaces/isaac_ros-dev/docker/Dockerfile.bfmc
```

### `file 'bfmc_localization.launch.py' was not found`

Rebuild `bfmc_global_localization` after syncing:

```bash
colcon build --symlink-install --packages-select bfmc_global_localization --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```
