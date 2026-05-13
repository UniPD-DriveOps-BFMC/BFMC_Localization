Use this full clean Isaac ROS setup flow for your machine. Your workspace is:

```bash
/home/jayamalsigera/ros2_workspaces/BFMC_Localization
```

Current NVIDIA official Isaac ROS docs target newer Isaac ROS versions with ROS 2 Jazzy/JetPack 7, while your working stack is **ROS 2 Humble + Ubuntu 22.04 + Isaac ROS release-3 packages**, which matches what you already used successfully for `isaac_ros_visual_slam`. So keep using Humble for this BFMC workspace. NVIDIA’s Isaac ROS docs describe the development environment around `isaac_ros_common` and `run_dev.sh`, and the package is designed around containerized development. ([NVIDIA Isaac ROS][1])

First fix your host GPU problem. Your earlier error means Docker tried to attach NVIDIA GPU devices but the NVIDIA runtime could not access the GPU. On the host, run:

```bash
nvidia-smi
```

If it fails, fix the host NVIDIA driver first. Then run:

```bash
sudo apt update
sudo apt install -y nvidia-container-toolkit

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Test Docker GPU access:

```bash
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

If this fails, do not continue with Isaac Visual SLAM yet. Isaac package building can be done without GPU, but cuVSLAM runtime needs GPU.

Now prepare the workspace from top:

```bash
mkdir -p /home/jayamalsigera/ros2_workspaces/BFMC_Localization/src

cd /home/jayamalsigera/ros2_workspaces/BFMC_Localization/src
```

Install Git LFS:

```bash
sudo apt update
sudo apt install -y git-lfs
git lfs install
```

Clone Isaac ROS Common cleanly:

```bash
cd /home/jayamalsigera/ros2_workspaces/BFMC_Localization/src

rm -rf isaac_ros_common

git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

cd isaac_ros_common
git lfs pull
```

Start the Isaac ROS Docker:

```bash
cd /home/jayamalsigera/ros2_workspaces/BFMC_Localization/src/isaac_ros_common

./scripts/run_dev.sh /home/jayamalsigera/ros2_workspaces/BFMC_Localization
```

If the GPU error still appears and you only want to create/build packages, run without GPU:

```bash
./scripts/run_dev.sh --disable-gpu /home/jayamalsigera/ros2_workspaces/BFMC_Localization
```

Inside the container, verify:

```bash
cd /workspaces/isaac_ros-dev

source /opt/ros/humble/setup.bash

ls src
```

Install the packages you need inside the container:

```bash
sudo apt update

sudo apt install -y \
  ros-humble-isaac-ros-visual-slam \
  ros-humble-depthai-ros \
  ros-humble-robot-localization \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view
```

Then build your workspace packages:

```bash
cd /workspaces/isaac_ros-dev

source /opt/ros/humble/setup.bash

rm -rf build install log

colcon build --symlink-install

source install/setup.bash
```

Check your BFMC packages:

```bash
colcon list | grep bfmc
ros2 pkg list | grep bfmc
```

For daily use after the container already exists:

```bash
docker ps -a | grep isaac
```

Then enter it using the actual name:

```bash
docker start isaac_ros_dev-x86_64-container
docker exec -it --user admin isaac_ros_dev-x86_64-container bash
```

Inside:

```bash
cd /workspaces/isaac_ros-dev
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Run the basic localization stack in separate terminals:

```bash
ros2 launch bfmc_car_description robot_state_publisher.launch.py
```

```bash
ros2 launch bfmc_isaac_visual_odom isaac_visual_odom_stereo.launch.py
```

```bash
ros2 launch bfmc_gps_position gps_position.launch.py
```

```bash
ros2 launch bfmc_map_matching rgb_map_matching.launch.py
```

```bash
ros2 launch bfmc_global_localization global_localization.launch.py
```

Check outputs:

```bash
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic echo /odometry/global --once
ros2 topic echo /automobile/localization/final_xy --once
```

If `run_dev.sh` fails again with missing LFS files, run:

```bash
cd /home/jayamalsigera/ros2_workspaces/BFMC_Localization/src/isaac_ros_common
git lfs pull
```

If it fails with GPU/CDI errors, the issue is host NVIDIA Docker runtime, not ROS. Use `--disable-gpu` for package work and fix `nvidia-smi` + `nvidia-container-toolkit` before running Isaac Visual SLAM.

[1]: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_common/index.html?utm_source=chatgpt.com "Isaac ROS Common"

