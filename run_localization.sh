#!/bin/bash
# ============================================================
# BFMC Localization — one-command launcher
# Syncs packages, starts the Isaac ROS Docker container, and
# launches the full localization stack inside it.
#
# Usage:
#   ./run_localization.sh [OPTIONS]
#
# Options:
#   --use-gps      true|false    Enable GPS fusion          (default: true)
#   --camera-mode  stereo|rgbd   Visual odometry mode       (default: stereo)
#   --no-rebuild                 Skip rebuilding BFMC packages (default: builds)
#   --no-sync                    Skip rsync to isaac_ros-dev (default: off)
#   --i2c-bus      N             I2C bus for BNO055 IMU     (default: 7)
#   --normal-start               Set EKF pose to start box (15.48, 3.83) (default)
#   --random-start               Find nearest checkpoint from current fused position
#   -h, --help                   Show this message
#
# Examples:
#   ./run_localization.sh
#   ./run_localization.sh --normal-start
#   ./run_localization.sh --random-start --use-gps false
#   ./run_localization.sh --no-rebuild --camera-mode rgbd --normal-start
# ============================================================
set -euo pipefail

# ── paths ────────────────────────────────────────────────────────────────────
BFMC_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_WS="$HOME/workspaces/isaac_ros-dev"
ISAAC_COMMON="$ISAAC_WS/src/isaac_ros_common"
RUN_DEV="$ISAAC_COMMON/scripts/run_dev.sh"

# ── defaults ─────────────────────────────────────────────────────────────────
USE_GPS="true"
CAMERA_MODE="stereo"
REBUILD=true
NO_SYNC=false
I2C_BUS=7
START_MODE="normal"   # normal | random

# ── argument parsing ─────────────────────────────────────────────────────────
usage() {
  sed -n '/^# Usage/,/^# ====/p' "$0" | grep -v "^# ====" | sed 's/^# \?//'
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --use-gps)       USE_GPS="$2";      shift 2 ;;
    --camera-mode)   CAMERA_MODE="$2";  shift 2 ;;
    --no-rebuild)    REBUILD=false;     shift   ;;
    --no-sync)       NO_SYNC=true;      shift   ;;
    --i2c-bus)       I2C_BUS="$2";      shift 2 ;;
    --normal-start)  START_MODE="normal"; shift  ;;
    --random-start)  START_MODE="random"; shift  ;;
    -h|--help)       usage; exit 0      ;;
    *) echo "ERROR: unknown option '$1'"; usage; exit 1 ;;
  esac
done

# ── banner ───────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════╗"
echo "║      BFMC Localization Stack Launcher    ║"
echo "╚══════════════════════════════════════════╝"
echo "  GPS:         $USE_GPS"
echo "  Camera mode: $CAMERA_MODE"
echo "  Build:       $([ "$REBUILD" = true ] && echo yes || echo skipped)"
echo "  Symlink:     $([ "$NO_SYNC" = true ] && echo no || echo yes)"
echo "  I2C bus:     $I2C_BUS"
echo "  Start mode:  $START_MODE"
echo ""

# ── pre-flight checks ────────────────────────────────────────────────────────
if [ ! -f "$RUN_DEV" ]; then
  echo "ERROR: Isaac ROS run_dev.sh not found at:"
  echo "       $RUN_DEV"
  echo ""
  echo "Follow the 'Install Isaac ROS on Jetson' steps in README.md first."
  exit 1
fi

if ! docker info &>/dev/null; then
  echo "ERROR: Docker is not running. Start it with: sudo systemctl start docker"
  exit 1
fi

# ── step 1: symlink ───────────────────────────────────────────────────────────
if [ "$NO_SYNC" = false ]; then
  echo "[1/3] Symlinking BFMC packages → isaac_ros-dev..."
  # src packages — every directory directly under src/
  for pkg_path in "$BFMC_WS/src"/*/; do
    pkg=$(basename "$pkg_path")
    target="$ISAAC_WS/src/$pkg"
    [ -d "$target" ] && [ ! -L "$target" ] && rm -rf "$target"
    ln -sfn "$BFMC_WS/src/$pkg" "$target"
    echo "  linked: src/$pkg"
  done
  # Dockerfile so the OpenCV symlink fix reaches the image build
  if [ -f "$BFMC_WS/docker/Dockerfile.bfmc" ]; then
    ln -sf "$BFMC_WS/docker/Dockerfile.bfmc" "$ISAAC_WS/docker/Dockerfile.bfmc"
    echo "  linked: docker/Dockerfile.bfmc"
  fi
  echo "  Symlink done."
else
  echo "[1/3] Symlink skipped (--no-sync)."
fi

# ── step 2: build command (optional) ─────────────────────────────────────────
BUILD_CMD=""
if [ "$REBUILD" = true ]; then
  echo "[2/3] Will build BFMC packages inside Docker."
  BUILD_CMD=$(cat <<'BUILDEOF'
echo "--- Building BFMC packages ---"
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
echo "--- Build complete ---"
BUILDEOF
)
else
  echo "[2/3] Skipping build (--no-rebuild)."
fi

# ── step 3: inner script ──────────────────────────────────────────────────────
INNER_SCRIPT=$(cat <<INNEREOF
set -e
cd /workspaces/isaac_ros-dev

echo "--- Sourcing ROS ---"
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

echo "--- IMU hardware permission ---"
chmod 666 /dev/i2c-${I2C_BUS} 2>/dev/null && \
  echo "  /dev/i2c-${I2C_BUS} ready" || \
  echo "  WARNING: could not chmod /dev/i2c-${I2C_BUS} (IMU may not work)"

${BUILD_CMD}

source install/setup.bash

echo ""
echo "--- Launching localization stack ---"
echo "    use_gps:=${USE_GPS}  camera_mode:=${CAMERA_MODE}  start:=${START_MODE}"
echo ""
ros2 launch bfmc_global_localization bfmc_localization.launch.py \
  use_gps:=${USE_GPS} \
  camera_mode:=${CAMERA_MODE} &
LAUNCH_PID=\$!

echo "--- Starting path planner (${START_MODE}-start) ---"
python3 \$(ros2 pkg prefix bfmc_global_localization)/lib/bfmc_global_localization/path_planner.py \
  --${START_MODE}-start &
PLANNER_PID=\$!

wait \$LAUNCH_PID
kill \$PLANNER_PID 2>/dev/null || true
INNEREOF
)

# ── step 3: write run script into workspace so it is visible inside the container
SCRIPT_PATH="$ISAAC_WS/run_bfmc.sh"
printf '%s\n' "$INNER_SCRIPT" > "$SCRIPT_PATH"
chmod +x "$SCRIPT_PATH"
echo "[3/3] Startup script written to:"
echo "       $SCRIPT_PATH"
echo "       (visible inside container at /workspaces/isaac_ros-dev/run_bfmc.sh)"
echo ""

# ── step 3: start docker ──────────────────────────────────────────────────────
echo "Starting Isaac ROS Docker container..."
echo ""
echo "  ┌─ Once the shell opens, run: ───────────────────────────────┐"
echo "  │  sudo bash /workspaces/isaac_ros-dev/run_bfmc.sh           │"
echo "  └────────────────────────────────────────────────────────────┘"
echo ""
cd "$ISAAC_COMMON"
./scripts/run_dev.sh \
  -d "$ISAAC_WS" \
  --docker_arg "--privileged"
