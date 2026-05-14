Use this clean workflow.

### 1. Host setup

```bash
sudo apt update
sudo apt install -y git git-lfs curl docker.io python3-pip

git lfs install

sudo usermod -aG docker $USER
```

Logout/login or reboot.

### 2. Create Isaac workspace

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
```

### 3. Clone Isaac ROS packages

```bash
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git
```

### 4. Create BFMC Docker layer

```bash
mkdir -p ~/workspaces/isaac_ros-dev/docker

nano ~/workspaces/isaac_ros-dev/docker/Dockerfile.bfmc
```

Paste:

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
    ln -sf /usr/local/include/magic_enum/magic_enum.hpp /usr/local/include/magic_enum.hpp && \
    rm -rf /tmp/magic_enum

RUN pip3 install --no-cache-dir smbus2
```

### 5. Configure Isaac to use BFMC Docker image

```bash
cat > ~/.isaac_ros_common-config <<'EOF'
CONFIG_IMAGE_KEY="ros2_humble.bfmc"
CONFIG_DOCKER_SEARCH_DIRS=("$HOME/workspaces/isaac_ros-dev/docker")
EOF
```

### 6. Optional permanent host fixes

For `/dev/fb0` Docker issue:

```bash
sudo nano /etc/systemd/system/create-dummy-fb0.service
```

Paste:

```ini
[Unit]
Description=Create dummy /dev/fb0 for Isaac ROS Docker
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'if [ ! -e /dev/fb0 ]; then /bin/mknod /dev/fb0 c 29 0; fi; /bin/chmod 666 /dev/fb0'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable:

```bash
sudo systemctl daemon-reload
sudo systemctl enable create-dummy-fb0.service
sudo systemctl start create-dummy-fb0.service
```

For permanent I²C permissions:

```bash
sudo nano /etc/udev/rules.d/99-i2c.rules
```

Paste:

```text
KERNEL=="i2c-[0-9]*", MODE="0666"
```

Apply:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 7. Enter Isaac Docker

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common

./scripts/run_dev.sh \
  -d ~/workspaces/isaac_ros-dev \
  --docker_arg "--privileged"
```

Confirm the log says:

```text
image key aarch64.ros2_humble.bfmc
```

### 8. Build Isaac ROS Visual SLAM

Inside Docker:

```bash
cd /workspaces/isaac_ros-dev

source /opt/ros/humble/setup.bash

export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH

colcon build --symlink-install \
  --packages-up-to isaac_ros_visual_slam \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

### 9. If `magic_enum` CMake error appears

Patch:

```bash
nano /workspaces/isaac_ros-dev/src/isaac_ros_nitros/isaac_ros_nitros/CMakeLists.txt
```

Add before `ament_auto_find_build_dependencies()`:

```cmake
find_package(magic_enum REQUIRED)
```

Patch:

```bash
nano /workspaces/isaac_ros-dev/src/isaac_ros_image_pipeline/isaac_ros_gxf_extensions/gxf_isaac_image_flip/CMakeLists.txt
```

Add before `ament_auto_add_library(...)`:

```cmake
find_package(magic_enum REQUIRED)
```

Patch test issue if needed:

```bash
nano /workspaces/isaac_ros-dev/src/isaac_ros_image_pipeline/isaac_ros_image_proc/CMakeLists.txt
```

Wrap:

```cmake
if(TARGET alpha_blend_test)
  target_link_libraries(alpha_blend_test alpha_blend_node)
endif()
```

Then rebuild:

```bash
cd /workspaces/isaac_ros-dev

rm -rf build/isaac_ros_nitros install/isaac_ros_nitros
rm -rf build/gxf_isaac_image_flip install/gxf_isaac_image_flip
rm -rf build/isaac_ros_image_proc install/isaac_ros_image_proc
rm -rf build/isaac_ros_visual_slam install/isaac_ros_visual_slam

source /opt/ros/humble/setup.bash

export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH

colcon build --symlink-install \
  --packages-up-to isaac_ros_visual_slam \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

### 10. Verify Isaac ROS install

```bash
ros2 pkg list | grep isaac_ros_visual_slam
ros2 pkg list | grep isaac_ros_nitros
```

### 11. Sync BFMC packages

Exit Docker:

```bash
exit
```

On host:

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
  ~/workspaces/isaac_ros-dev/src/

rsync -av --delete \
  ~/workspaces/BFMC_Localization/docker/ \
  ~/workspaces/isaac_ros-dev/docker/
```

### 12. Build BFMC packages

Enter Docker again:

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common

./scripts/run_dev.sh \
  -d ~/workspaces/isaac_ros-dev \
  --docker_arg "--privileged"
```

Inside Docker:

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
    bfmc_global_localization \
  --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

### 13. Daily workflow

Start Docker:

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common

./scripts/run_dev.sh \
  -d ~/workspaces/isaac_ros-dev \
  --docker_arg "--privileged"
```

Inside Docker:

```bash
cd /workspaces/isaac_ros-dev

source /opt/ros/humble/setup.bash
source install/setup.bash
```

Run IMU:

```bash
i2cdetect -y -r 7
ros2 run automobile_imu bno055_imu_node
```

Check topic:

```bash
ros2 topic echo /automobile/imu/data
```

Do not rebuild Isaac every time. Only rebuild changed BFMC packages.

```bash
ros2 launch bfmc_global_localization bfmc_localization.launch.py
```