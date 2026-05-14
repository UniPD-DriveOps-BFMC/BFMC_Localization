# BFMC Localization ROS 2 Workspace

Full localization stack for the Bosch Future Mobility Challenge competition car.

---

## Prerequisites (Jetson host — run once)

```bash
sudo apt install ros-jazzy-robot-localization
```

`robot_localization` provides the `ekf_node` binary used by `bfmc_odometry_fusion` and `bfmc_global_localization` at runtime. It is not a compile-time dependency so the Docker build does not need it installed.

---

## Package Overview

| # | Package | Output topics |
|---|---------|---------------|
| 0 | `automobile_imu` | `/automobile/imu/data` |
| 1 | `bfmc_car_description` | TF frames (`base_link`, `oak_*`, `gps_tag_link`) |
| 2 | `bfmc_isaac_visual_odom` | `/visual_odom`, `/visual_odom_planar` |
| 3 | `bfmc_state_odometry` | `/encoder_odom`, `/car/imu/data` |
| 4 | `bfmc_odometry_fusion` | `/odometry/local`, `/odom_distance`, `/odom_velocity` |
| 5 | `bfmc_gps_position` | `/automobile/gps/base_pose` |
| 6 | `bfmc_map_matching` | `/automobile/map_match/base_pose`, `/automobile/map_match/lane_mask` |
| 7 | `bfmc_global_localization` | `/odometry/global`, `/automobile/current_coordinate`, `/automobile/current_node`, `/automobile/sign/base_pose` |

---

## TF Tree

```
map
 └── odom          ← published by global EKF (bfmc_global_localization)
      └── base_link ← published by local EKF (bfmc_odometry_fusion)
           ├── oak_left_camera_optical_frame
           ├── oak_right_camera_optical_frame
           └── gps_tag_link
```

---

## Sync to Isaac ROS Workspace (run on Jetson host)

Run this once before each Docker build session whenever packages change:

```bash
rsync -av --delete \
  ~/workspaces/BFMC_Localization/src/automobile_imu \
  ~/workspaces/BFMC_Localization/src/bfmc_car_description \
  ~/workspaces/BFMC_Localization/src/bfmc_isaac_visual_odom \
  ~/workspaces/BFMC_Localization/src/bfmc_state_odometry \
  ~/workspaces/BFMC_Localization/src/bfmc_odometry_fusion \
  ~/workspaces/BFMC_Localization/src/bfmc_gps_position \
  ~/workspaces/BFMC_Localization/src/bfmc_map_matching \
  ~/workspaces/BFMC_Localization/src/bfmc_global_localization \
  ~/workspaces/isaac_ros-dev/src/ && \
rsync -av --delete \
  ~/workspaces/BFMC_Localization/docker/ \
  ~/workspaces/isaac_ros-dev/docker/
```

Preview without copying (dry run):

```bash
rsync -avn --delete \
  ~/workspaces/BFMC_Localization/src/automobile_imu \
  ~/workspaces/BFMC_Localization/src/bfmc_car_description \
  ~/workspaces/BFMC_Localization/src/bfmc_isaac_visual_odom \
  ~/workspaces/BFMC_Localization/src/bfmc_state_odometry \
  ~/workspaces/BFMC_Localization/src/bfmc_odometry_fusion \
  ~/workspaces/BFMC_Localization/src/bfmc_gps_position \
  ~/workspaces/BFMC_Localization/src/bfmc_map_matching \
  ~/workspaces/BFMC_Localization/src/bfmc_global_localization \
  ~/workspaces/isaac_ros-dev/src/
```

One-shot sync script (save once, run any time):

```bash
cat > ~/sync_bfmc_to_isaac.sh <<'EOF'
#!/bin/bash
set -e
SRC="$HOME/workspaces/BFMC_Localization/src"
WS="$HOME/workspaces/BFMC_Localization"
DST="$HOME/workspaces/isaac_ros-dev/src"
rsync -av --delete \
  "$SRC/automobile_imu" \
  "$SRC/bfmc_car_description" \
  "$SRC/bfmc_isaac_visual_odom" \
  "$SRC/bfmc_state_odometry" \
  "$SRC/bfmc_odometry_fusion" \
  "$SRC/bfmc_gps_position" \
  "$SRC/bfmc_map_matching" \
  "$SRC/bfmc_global_localization" \
  "$DST/"
rsync -av --delete "$WS/docker/" "$HOME/workspaces/isaac_ros-dev/docker/"
echo "Sync complete."
EOF
chmod +x ~/sync_bfmc_to_isaac.sh
```

Then later:

```bash
~/sync_bfmc_to_isaac.sh
```

---

## Permanent Docker Dependencies (run once on Jetson)

All apt/pip dependencies are baked into a custom Docker image layer so they
survive container restarts. This only needs to be set up once, then every
`run_dev.sh` automatically uses the pre-built image.

**Step 1 — copy the config file into the Isaac ROS workspace:**

```bash
cat > ~/workspaces/isaac_ros-dev/.isaac_ros_common-config <<'EOF'
CONFIG_IMAGE_KEY="ros2_humble.bfmc"
CONFIG_DOCKER_SEARCH_DIRS=("$HOME/workspaces/isaac_ros-dev/docker")
EOF
```

**Step 2 — sync the BFMC workspace (includes the Dockerfile):**

```bash
~/sync_bfmc_to_isaac.sh
```

**Step 3 — rebuild the Docker image (takes a few minutes, only needed when `Dockerfile.bfmc` changes):**

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev --docker_arg "--privileged"
```

`run_dev.sh` detects the new config, builds a new image layer with all dependencies installed, and drops you into a container. Subsequent runs reuse the cached image — no rebuild unless the Dockerfile changes.

**What gets installed permanently:**

| Dependency | Why |
|------------|-----|
| `ros-humble-robot-localization` | `ekf_node` binary for local and global EKF |
| `i2c-tools` | I2C bus debugging (`i2cdetect`) |
| `smbus2` (pip) | BNO055 IMU driver (`automobile_imu`) |

---

## Enter the Isaac ROS Docker Container

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev --docker_arg "--privileged"
```

Grant I2C access inside the container (needed for BNO055 IMU):

```bash
sudo chmod 666 /dev/i2c-7
```

Verify the IMU is detected:

```bash
i2cdetect -y -r 7
# Expected: device at 0x28
```

Open an extra terminal into the same running container:

```bash
docker ps
docker exec -it --user admin <container_name> bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Inside every container terminal:

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## Build Inside Docker

Clean build (run after syncing new changes):

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash

rm -rf \
  build/automobile_imu           install/automobile_imu \
  build/bfmc_car_description     install/bfmc_car_description \
  build/bfmc_isaac_visual_odom   install/bfmc_isaac_visual_odom \
  build/bfmc_state_odometry      install/bfmc_state_odometry \
  build/bfmc_odometry_fusion     install/bfmc_odometry_fusion \
  build/bfmc_gps_position        install/bfmc_gps_position \
  build/bfmc_map_matching        install/bfmc_map_matching \
  build/bfmc_global_localization install/bfmc_global_localization

colcon build --symlink-install \
  --packages-select \
    automobile_imu \
    bfmc_car_description \
    bfmc_isaac_visual_odom \
    bfmc_state_odometry \
    bfmc_odometry_fusion \
    bfmc_gps_position \
    bfmc_map_matching \
    bfmc_global_localization

source install/setup.bash
```

Verify packages are installed:

```bash
ros2 pkg list | grep bfmc
```

---

## Build on Jetson Host (non-Isaac packages only)

```bash
cd ~/workspaces/BFMC_Localization
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-select \
    automobile_imu \
    bfmc_car_description \
    bfmc_state_odometry \
    bfmc_odometry_fusion \
    bfmc_gps_position \
    bfmc_map_matching \
    bfmc_global_localization

source install/setup.bash
```

---

## Run the Full Stack

Source in every new terminal (Jazzy host):

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspaces/BFMC_Localization/install/setup.bash
```

Or use the master launch file to start everything at once — see [Master Launch](#master-launch) below.

---

### Step 0 — BNO055 IMU driver

Reads the car IMU over I2C and publishes raw IMU data.

```bash
ros2 run automobile_imu bno055_imu_node
```

Check:

```bash
ros2 topic hz /automobile/imu/data
ros2 topic echo /automobile/imu/data --once
```

---

### Step 1 — Robot TF tree

```bash
ros2 launch bfmc_car_description robot_state_publisher.launch.py
```

Check:

```bash
ros2 run tf2_ros tf2_echo base_link oak_left_camera_optical_frame
ros2 run tf2_ros tf2_echo base_link gps_tag_link
```

---

### Step 2 — OAK Camera driver

Start your existing DepthAI / depthai-ros launch. After it starts:

```bash
ros2 topic hz /oak/left/image_mono
ros2 topic hz /oak/right/image_mono
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/imu
```

---

### Step 3 — Isaac Visual Odometry (inside Docker)

```bash
# Stereo (default)
ros2 launch bfmc_isaac_visual_odom isaac_visual_odom_stereo.launch.py

# RGBD (alternative)
ros2 launch bfmc_isaac_visual_odom isaac_visual_odom_rgbd.launch.py
```

Check:

```bash
ros2 topic hz /visual_odom_planar
ros2 topic echo /visual_odom_planar --once
```

---

### Step 4 — Encoder + IMU odometry

```bash
ros2 launch bfmc_state_odometry state_odometry.launch.py
```

Check:

```bash
ros2 topic echo /encoder_odom --once
ros2 topic echo /car/imu/data --once
```

---

### Step 5 — Local EKF fusion

Fuses visual odometry, encoder odometry, and car IMU into `odom → base_link`.

```bash
ros2 launch bfmc_odometry_fusion local_ekf.launch.py
```

Check:

```bash
ros2 topic echo /odometry/local --once
ros2 topic hz /odometry/local
ros2 topic echo /odom_distance
ros2 topic echo /odom_velocity
```

---

### Step 6 — GPS position

Converts raw GPS tag pose → corrected base-link pose with dynamic covariance (from GPS quality).

```bash
ros2 launch bfmc_gps_position gps_position.launch.py
```

Check:

```bash
ros2 topic echo /automobile/gps/base_pose --once
```

---

### Step 7 — Map matching

Matches the camera image against the track map to produce a global pose correction.

```bash
ros2 launch bfmc_map_matching rgb_map_matching.launch.py
```

Check:

```bash
ros2 topic hz /automobile/map_match/lane_mask
ros2 topic echo /automobile/map_match/base_pose --once
```

---

### Step 8 — Global localization

Fuses local odometry, GPS, map matching, and sign detections into a `map → odom` transform.
Publishes the final position as a coordinate and nearest graph node.

```bash
# GPS enabled (default)
ros2 launch bfmc_global_localization global_ekf.launch.py

# GPS disabled
ros2 launch bfmc_global_localization global_ekf.launch.py use_gps:=false
```

Check:

```bash
ros2 topic echo /odometry/global --once
ros2 topic echo /automobile/current_coordinate --once
ros2 topic echo /automobile/current_node --once
ros2 topic echo /automobile/sign/base_pose --once
```

---

## Master Launch

Starts all packages in order with a single command (Jazzy host, no Docker — Isaac visual odom still requires Docker).

```bash
# Show all arguments
ros2 launch ~/workspaces/BFMC_Localization/src/bfmc_localization.launch.py --show-args
```

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `use_gps` | `true` | `true` / `false` | Enable GPS fusion in global EKF |
| `camera_mode` | `stereo` | `stereo` / `rgbd` | Visual odometry camera mode |

```bash
# Default — GPS on, stereo camera
ros2 launch ~/workspaces/BFMC_Localization/src/bfmc_localization.launch.py

# GPS off
ros2 launch ~/workspaces/BFMC_Localization/src/bfmc_localization.launch.py use_gps:=false

# RGBD camera
ros2 launch ~/workspaces/BFMC_Localization/src/bfmc_localization.launch.py camera_mode:=rgbd

# GPS off + RGBD
ros2 launch ~/workspaces/BFMC_Localization/src/bfmc_localization.launch.py use_gps:=false camera_mode:=rgbd
```

---

## Final Output Verification

```bash
ros2 topic list | grep -E "visual|odom|gps|map_match|sign|current"
```

Expected active topics:

```
/automobile/imu/data
/visual_odom
/visual_odom_planar
/encoder_odom
/car/imu/data
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
```

Full TF tree check:

```bash
ros2 run tf2_tools view_frames
ros2 run rqt_graph rqt_graph
```

Publish rates:

```bash
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global
ros2 topic hz /automobile/current_coordinate
```
