# BFMC Localization ROS 2 Workspace

ROS 2 localization stack for the Bosch Future Mobility Challenge autonomous vehicle platform.

This workspace is intended to run mainly inside the Isaac ROS Docker environment on the Jetson Orin Nano. The Isaac ROS container uses ROS 2 Humble. Host-side ROS 2 Jazzy is optional and should not be mixed with the Docker Humble environment unless you explicitly know the DDS compatibility constraints.

---

## Recommended Workflow Summary

Use this workflow for normal development:

1. Edit packages in the host workspace:
   
   ```bash
   ~/workspaces/BFMC_Localization
   ```

2. Sync the packages into the Isaac ROS workspace:
   
   ```bash
   ~/sync_bfmc_to_isaac.sh
   ```

3. Enter the Isaac ROS Docker container with privileged device access:
   
   ```bash
   cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
   ./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev --docker_arg "--privileged"
   ```

4. Inside Docker, source ROS and the workspace:
   
   ```bash
   cd /workspaces/isaac_ros-dev
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

5. Give temporary I2C permission for the BNO055 IMU:
   
   ```bash
   sudo chmod 666 /dev/i2c-7
   ```

6. Build and run the localization packages.

---

## Package Overview

| #   | Package                    | Main output topics / function                                                                                  |
| --- | -------------------------- | -------------------------------------------------------------------------------------------------------------- |
| 0   | `automobile_imu`           | `/automobile/imu/data`                                                                                         |
| 1   | `bfmc_car_description`     | robot TF frames: `base_link`, `imu_link`, `oak_*`, `gps_tag_link`                                              |
| 2   | `bfmc_isaac_visual_odom`   | `/visual_odom`, `/visual_odom_planar`                                                                          |
| 3   | `bfmc_state_odometry`      | `/encoder_odom`, processed odometry inputs                                                                     |
| 4   | `bfmc_odometry_fusion`     | `/odometry/local`, `/odom_distance`, `/odom_velocity`                                                          |
| 5   | `bfmc_gps_position`        | `/automobile/gps/base_pose`                                                                                    |
| 6   | `bfmc_map_matching`        | `/automobile/map_match/base_pose`, `/automobile/map_match/lane_mask`                                           |
| 7   | `bfmc_global_localization` | `/odometry/global`, `/automobile/current_coordinate`, `/automobile/current_node`, `/automobile/sign/base_pose` |

---

## TF Tree

Expected high-level TF structure:

```text
map
 └── odom                # published by global EKF: bfmc_global_localization
      └── base_link      # published by local EKF: bfmc_odometry_fusion
           ├── imu_link
           ├── oak_left_camera_optical_frame
           ├── oak_right_camera_optical_frame
           └── gps_tag_link
```

The BNO055 IMU publisher uses `imu_link` as the `header.frame_id`, so `bfmc_car_description` must publish a fixed transform from `base_link` to `imu_link`.

---

## One-Time Docker Setup

The custom BFMC Docker layer installs permanent dependencies such as `robot_localization`, `i2c-tools`, `smbus2`, and `magic_enum`. These dependencies persist in the built Docker image. Runtime permissions such as `chmod 666 /dev/i2c-7` must still be applied again when a new container is created.

### 1. Create Isaac ROS config on the Jetson host

Run on the Jetson host, not inside Docker:

```bash
cat > ~/.isaac_ros_common-config <<'CONFIGEOF'
CONFIG_IMAGE_KEY="ros2_humble.bfmc"
CONFIG_DOCKER_SEARCH_DIRS=("$HOME/workspaces/isaac_ros-dev/docker")
CONFIGEOF
```

This should make `run_dev.sh` use the image key:

```text
aarch64.ros2_humble.bfmc
```

### 2. Create the BFMC Dockerfile

Create:

```bash
mkdir -p ~/workspaces/BFMC_Localization/docker
nano ~/workspaces/BFMC_Localization/docker/Dockerfile.bfmc
```

Use this Dockerfile:

```dockerfile
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-robot-localization \
    ros-humble-tf2-tools \
    ros-humble-rqt-graph \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-depthai-ros \
    ros-humble-depthai-bridge \
    ros-humble-depthai-descriptions \
    ros-humble-depthai-examples \
    i2c-tools \
    python3-pip \
    python3-smbus \
    python3-dev \
    python3-numpy \
    python3-opencv \
    git \
    cmake \
    build-essential \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    v4l-utils \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/Neargye/magic_enum.git /tmp/magic_enum && \
    cmake -S /tmp/magic_enum -B /tmp/magic_enum/build -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/magic_enum/build --target install && \
    rm -rf /tmp/magic_enum

RUN pip3 install --no-cache-dir smbus2
```

`magic_enum` is installed from source because `libmagic-enum-dev` is not available in the Jetson Ubuntu repositories used by the Isaac ROS image.

### 3. Sync Dockerfile and packages to Isaac workspace

Create the sync script once:

```bash
cat > ~/sync_bfmc_to_isaac.sh <<'SYNCEOF'
#!/bin/bash
set -e

SRC="$HOME/workspaces/BFMC_Localization/src"
WS="$HOME/workspaces/BFMC_Localization"
DST="$HOME/workspaces/isaac_ros-dev/src"
DOCKER_DST="$HOME/workspaces/isaac_ros-dev/docker"

mkdir -p "$DST"
mkdir -p "$DOCKER_DST"

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

rsync -av --delete "$WS/docker/" "$DOCKER_DST/"

echo "Sync complete."
SYNCEOF

chmod +x ~/sync_bfmc_to_isaac.sh
```

Then run it whenever the BFMC packages or Dockerfile change:

```bash
~/sync_bfmc_to_isaac.sh
```

### 4. Build/re-enter the custom Docker container

Run on the Jetson host:

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev --docker_arg "--privileged"
```

The log should show:

```text
Launching Isaac ROS Dev container with image key aarch64.ros2_humble.bfmc
Resolved ... Dockerfile.bfmc
```

If the log shows only `aarch64.ros2_humble`, then the custom BFMC Docker layer is not being used.

---

## Enter Existing Docker Container from Another Terminal

From another host terminal:

```bash
docker ps --format "table {{.Names}}\t{{.Status}}"
```

Then enter the running container:

```bash
docker exec -it --user admin <container_name> bash
```

Inside the container:

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## IMU Hardware Verification

Inside Docker:

```bash
ls -l /dev/i2c*
sudo chmod 666 /dev/i2c-7
i2cdetect -y -r 7
```

Expected BNO055 detection:

```text
20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- --
```

If `i2cdetect` is not found, your BFMC Dockerfile was not included in the image, or the image was not rebuilt after adding `i2c-tools`.

---

## Build Inside Docker

Always source ROS first:

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
```

### Option A — Build BFMC packages except Isaac visual odometry

Use this when you only need IMU, TF, encoder odometry, GPS, map matching, and EKF localization:

```bash
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

This avoids Isaac Visual SLAM dependencies.

### Option B — Build NVIDIA Isaac Visual SLAM dependencies

Use this only when `bfmc_isaac_visual_odom` is needed:

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to isaac_ros_visual_slam

source install/setup.bash
```

If the build fails at `gxf_isaac_image_flip` with `magic_enum::magic_enum` not found, verify that the BFMC Dockerfile installed `magic_enum` from source.

### Option C — Build all BFMC localization packages including Isaac visual odometry

Run this after Option B succeeds:

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash

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

### Clean rebuild of BFMC packages

Use only when package build state is inconsistent:

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

Then rebuild using Option A or Option C.

Verify installed packages:

```bash
ros2 pkg list | grep -E "automobile_imu|bfmc"
```

---

## Runtime Setup in Every Docker Terminal

Inside every new Docker terminal:

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If the terminal will access the BNO055 IMU:

```bash
sudo chmod 666 /dev/i2c-7
```

---

## Run the Full Stack Manually

### Step 0 — BNO055 IMU driver

Reads the BNO055 over I2C and publishes `sensor_msgs/msg/Imu`:

```bash
ros2 run automobile_imu bno055_imu_node
```

Check from another Docker terminal:

```bash
ros2 topic hz /automobile/imu/data
ros2 topic echo /automobile/imu/data --once
```

### Step 1 — Robot TF tree

```bash
ros2 launch bfmc_car_description robot_state_publisher.launch.py
```

Check:

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link oak_left_camera_optical_frame
ros2 run tf2_ros tf2_echo base_link gps_tag_link
```

### Step 2 — OAK camera driver

Start your DepthAI/depthai-ros launch. After it starts, check:

```bash
ros2 topic hz /oak/left/image_mono
ros2 topic hz /oak/right/image_mono
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/imu
```

### Step 3 — Isaac Visual Odometry

Only run this if Isaac Visual SLAM dependencies were built successfully.

```bash
ros2 launch bfmc_isaac_visual_odom isaac_visual_odom_stereo.launch.py
```

Alternative RGBD mode:

```bash
ros2 launch bfmc_isaac_visual_odom isaac_visual_odom_rgbd.launch.py
```

Check:

```bash
ros2 topic hz /visual_odom_planar
ros2 topic echo /visual_odom_planar --once
```

### Step 4 — Encoder odometry / state odometry

```bash
ros2 launch bfmc_state_odometry state_odometry.launch.py
```

Check:

```bash
ros2 topic echo /encoder_odom --once
ros2 topic echo /automobile/imu/data --once
```

### Step 5 — Local EKF fusion

Fuses visual odometry, encoder odometry, and IMU data into local odometry.

```bash
ros2 launch bfmc_odometry_fusion local_ekf.launch.py
```

Check:

```bash
ros2 topic echo /odometry/local --once
ros2 topic hz /odometry/local
ros2 topic echo /odom_distance --once
ros2 topic echo /odom_velocity --once
```

### Step 6 — GPS position

Converts raw GPS tag pose to corrected base-link pose with covariance.

```bash
ros2 launch bfmc_gps_position gps_position.launch.py
```

Check:

```bash
ros2 topic echo /automobile/gps/base_pose --once
```

### Step 7 — Map matching

Matches camera/lane observations against the track map and publishes a global pose correction.

```bash
ros2 launch bfmc_map_matching rgb_map_matching.launch.py
```

Check:

```bash
ros2 topic hz /automobile/map_match/lane_mask
ros2 topic echo /automobile/map_match/base_pose --once
```

### Step 8 — Global localization

Fuses local odometry, GPS, map matching, and sign detections into global localization.

```bash
ros2 launch bfmc_global_localization global_ekf.launch.py
```

Without GPS:

```bash
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

Use this only if a package named `bfmc_localization` exists and installs the master launch file.

Show launch arguments:

```bash
ros2 launch bfmc_localization bfmc_localization.launch.py --show-args
```

Run default configuration:

```bash
ros2 launch bfmc_localization bfmc_localization.launch.py
```

Run without GPS:

```bash
ros2 launch bfmc_localization bfmc_localization.launch.py use_gps:=false
```

Run RGBD visual odometry mode:

```bash
ros2 launch bfmc_localization bfmc_localization.launch.py camera_mode:=rgbd
```

Run without GPS and with RGBD:

```bash
ros2 launch bfmc_localization bfmc_localization.launch.py use_gps:=false camera_mode:=rgbd
```

If no `bfmc_localization` package exists, run the stack manually using the steps above.

---

## Final Output Verification

List relevant topics:

```bash
ros2 topic list | grep -E "imu|visual|odom|gps|map_match|sign|current"
```

Expected active topics, depending on which modules are running:

```text
/automobile/imu/data
/visual_odom
/visual_odom_planar
/encoder_odom
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

Check TF tree and ROS graph:

```bash
ros2 run tf2_tools view_frames
ros2 run rqt_graph rqt_graph
```

Check publish rates:

```bash
ros2 topic hz /automobile/imu/data
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global
ros2 topic hz /automobile/current_coordinate
```

---

## Common Problems

### `i2cdetect: command not found`

The container does not include `i2c-tools`. Check that the BFMC Docker layer was actually used.

Expected image key:

```text
aarch64.ros2_humble.bfmc
```

### `Permission denied: /dev/i2c-7`

Run inside Docker:

```bash
sudo chmod 666 /dev/i2c-7
```

### `ModuleNotFoundError: No module named smbus2`

The BFMC Docker layer was not used, or `smbus2` was not installed. Rebuild the image from `Dockerfile.bfmc`.

### `ModuleNotFoundError: No module named board`

Do not use the Adafruit `board`/`busio` implementation inside Docker. Use the direct `smbus2` BNO055 node instead.

### `magic_enum::magic_enum target was not found`

- The Isaac GXF package needs `magic_enum`. Install it from source in `Dockerfile.bfmc`; do not use `libmagic-enum-dev` on Jetson because it may not exist in the apt repositories.
