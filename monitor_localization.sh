#!/bin/bash
# ============================================================
# BFMC Localization Monitor — run on LOCAL machine
# Connects to the Jetson over the same LAN via ROS 2 DDS.
#
# Usage:
#   ./monitor_localization.sh [OPTIONS]
#
# Options:
#   --topic-publish       Open tmux with live topic echo tabs
#   --domain-id    N      ROS_DOMAIN_ID to match Jetson      (default: 0)
#   --bag-dir      DIR    Directory to save rosbags           (default: ~/bfmc_bags)
#   --no-bag              Do not record a rosbag
#   --ros-distro   NAME   ROS 2 distro on local machine      (default: jazzy)
#   -h, --help
#
# Examples:
#   ./monitor_localization.sh --topic-publish
#   ./monitor_localization.sh --topic-publish --domain-id 42
#   ./monitor_localization.sh --no-bag --topic-publish
#   ./monitor_localization.sh --bag-dir /media/usb/bags
#
# Network note:
#   Both this machine and the Jetson must be on the same LAN and
#   share the same ROS_DOMAIN_ID. Multicast UDP must not be blocked
#   by a firewall. If discovery fails, set on BOTH machines:
#     export ROS_DOMAIN_ID=<N>
# ============================================================
set -euo pipefail

# ── final output topics ───────────────────────────────────────────────────────
declare -A TOPIC_LABEL=(
  ["/automobile/current_coordinate"]="Position (x,y)"
  ["/automobile/current_node"]="Current Node"
  ["/automobile/current_speed"]="Speed (m/s)"
  ["/automobile/total_distance"]="Total Distance (m)"
  ["/odometry/local"]="Local Odometry"
  ["/odometry/global"]="Global Odometry"
  ["/automobile/gps/base_pose"]="GPS Pose"
  ["/automobile/map_match/base_pose"]="Map Match Pose"
  ["/automobile/sign/base_pose"]="Sign Pose"
)

RECORD_TOPICS=(
  # ── hardware inputs ──────────────────────────────────────────────────────────
  /automobile/encoder/speed
  /automobile/encoder/distance
  /automobile/imu/data
  /automobile/localisation          # raw GPS tag pose from car firmware
  /visual_slam/tracking/odometry   # Isaac ROS visual SLAM
  /oak/rgb/camera_info
  /oak/rgb/image_rect               # large; remove if storage is limited
  /traffic/detection

  # ── internal localization ────────────────────────────────────────────────────
  /car/imu/data                    # IMU republished with imu_link frame
  /encoder_odom
  /visual_odom
  /visual_odom_planar
  /odometry/local
  /automobile/gps/base_pose
  /automobile/map_match/base_pose
  /automobile/map_match/lane_mask
  /automobile/sign/base_pose
  /odometry/global
  /odom_distance
  /odom_velocity

  # ── final outputs ────────────────────────────────────────────────────────────
  /automobile/current_coordinate
  /automobile/current_node
  /automobile/current_speed
  /automobile/total_distance

  # ── transforms (required for correct bag playback) ───────────────────────────
  /tf
  /tf_static
)

# ── defaults ──────────────────────────────────────────────────────────────────
TOPIC_PUBLISH=false
DOMAIN_ID=0
BAG_DIR="$HOME/bfmc_bags"
RECORD_BAG=true
ROS_DISTRO_NAME="jazzy"

# ── parse args ────────────────────────────────────────────────────────────────
usage() {
  sed -n '/^# Usage/,/^# ====/p' "$0" | grep -v "^# ====" | sed 's/^# \?//'
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --topic-publish)  TOPIC_PUBLISH=true;    shift   ;;
    --domain-id)      DOMAIN_ID="$2";        shift 2 ;;
    --bag-dir)        BAG_DIR="$2";          shift 2 ;;
    --no-bag)         RECORD_BAG=false;      shift   ;;
    --ros-distro)     ROS_DISTRO_NAME="$2";  shift 2 ;;
    -h|--help)        usage; exit 0          ;;
    *) echo "ERROR: unknown option '$1'"; usage; exit 1 ;;
  esac
done

ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"

# ── banner ────────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════╗"
echo "║     BFMC Localization Monitor (Local)    ║"
echo "╚══════════════════════════════════════════╝"
echo "  ROS distro:     $ROS_DISTRO_NAME"
echo "  ROS_DOMAIN_ID:  $DOMAIN_ID"
echo "  Rosbag:         $([ "$RECORD_BAG" = true ] && echo "yes → $BAG_DIR" || echo no)"
echo "  Topic viewer:   $([ "$TOPIC_PUBLISH" = true ] && echo "tmux" || echo no)"
echo ""

# ── pre-flight ────────────────────────────────────────────────────────────────
if [ ! -f "$ROS_SETUP" ]; then
  echo "ERROR: ROS 2 setup not found at $ROS_SETUP"
  echo "       Install ROS 2 $ROS_DISTRO_NAME or pass --ros-distro <distro>"
  exit 1
fi

if [ "$TOPIC_PUBLISH" = true ] && ! command -v tmux &>/dev/null; then
  echo "ERROR: tmux is required for --topic-publish"
  echo "       Install with: sudo apt install tmux"
  exit 1
fi

# Export domain ID for all child processes
export ROS_DOMAIN_ID="$DOMAIN_ID"

SOURCE_CMD="source $ROS_SETUP && export ROS_DOMAIN_ID=$DOMAIN_ID"

# ── rosbag recording ──────────────────────────────────────────────────────────
BAG_PID=""

start_bag() {
  mkdir -p "$BAG_DIR"
  local bag_name="bfmc_$(date +%Y%m%d_%H%M%S)"
  local bag_path="$BAG_DIR/$bag_name"
  echo "[BAG] Recording → $bag_path"
  bash -c "
    $SOURCE_CMD
    ros2 bag record -o '$bag_path' ${RECORD_TOPICS[*]}
  " &
  BAG_PID=$!
}

stop_bag() {
  if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
    echo ""
    echo "[BAG] Stopping rosbag (PID $BAG_PID)..."
    kill "$BAG_PID" 2>/dev/null
    wait "$BAG_PID" 2>/dev/null || true
    echo "[BAG] Saved."
  fi
}

trap 'stop_bag; echo ""; echo "Monitor stopped."; exit 0' INT TERM

if [ "$RECORD_BAG" = true ]; then
  start_bag
fi

# ── tmux topic viewer ─────────────────────────────────────────────────────────
SESSION="bfmc_monitor"

if [ "$TOPIC_PUBLISH" = true ]; then
  # Kill old session if exists
  tmux kill-session -t "$SESSION" 2>/dev/null || true

  echo "[TMUX] Opening topic viewer (session: $SESSION)..."

  # Window layout:
  #  Window 0  "outputs"   — 2x2 grid: coordinate | node | speed | distance
  #  Window 1  "odometry"  — left/right: local odom | global odom
  #  Window 2  "sensors"   — thirds: GPS | map match | sign

  # ── window 0: key outputs (2×2 grid) ────────────────────────────────────────
  tmux new-session  -d -s "$SESSION" -n "outputs" \
    -x 220 -y 50

  tmux send-keys -t "$SESSION:outputs" \
    "$SOURCE_CMD && ros2 topic echo /automobile/current_coordinate" Enter

  tmux split-window -t "$SESSION:outputs" -h
  tmux send-keys -t "$SESSION:outputs" \
    "$SOURCE_CMD && ros2 topic echo /automobile/current_node" Enter

  tmux select-pane -t "$SESSION:outputs.0"
  tmux split-window -t "$SESSION:outputs" -v
  tmux send-keys -t "$SESSION:outputs" \
    "$SOURCE_CMD && ros2 topic echo /automobile/current_speed" Enter

  tmux select-pane -t "$SESSION:outputs.2"
  tmux split-window -t "$SESSION:outputs" -v
  tmux send-keys -t "$SESSION:outputs" \
    "$SOURCE_CMD && ros2 topic echo /automobile/total_distance" Enter

  # ── window 1: odometry ───────────────────────────────────────────────────────
  tmux new-window -t "$SESSION" -n "odometry"

  tmux send-keys -t "$SESSION:odometry" \
    "$SOURCE_CMD && ros2 topic echo /odometry/local" Enter

  tmux split-window -t "$SESSION:odometry" -h
  tmux send-keys -t "$SESSION:odometry" \
    "$SOURCE_CMD && ros2 topic echo /odometry/global" Enter

  # ── window 2: sensor poses ───────────────────────────────────────────────────
  tmux new-window -t "$SESSION" -n "sensors"

  tmux send-keys -t "$SESSION:sensors" \
    "$SOURCE_CMD && ros2 topic echo /automobile/gps/base_pose" Enter

  tmux split-window -t "$SESSION:sensors" -h
  tmux send-keys -t "$SESSION:sensors" \
    "$SOURCE_CMD && ros2 topic echo /automobile/map_match/base_pose" Enter

  tmux split-window -t "$SESSION:sensors" -v
  tmux send-keys -t "$SESSION:sensors" \
    "$SOURCE_CMD && ros2 topic echo /automobile/sign/base_pose" Enter

  # Focus window 0, pane 0
  tmux select-window -t "$SESSION:outputs"
  tmux select-pane   -t "$SESSION:outputs.0"

  echo ""
  echo "  tmux windows:"
  echo "    [0] outputs  — coordinate | node | speed | distance"
  echo "    [1] odometry — local odom | global odom"
  echo "    [2] sensors  — GPS | map match | sign"
  echo ""
  echo "  Switch windows: Ctrl+B then 0/1/2"
  echo "  Detach:         Ctrl+B then D"
  echo "  Kill session:   tmux kill-session -t $SESSION"
  echo ""

  # Attach to the session (blocks until detach or kill)
  tmux attach-session -t "$SESSION"
else
  echo "Bag recording running. Press Ctrl+C to stop."
  echo "(Add --topic-publish to also open the tmux topic viewer)"
  wait "$BAG_PID" 2>/dev/null || true
fi
