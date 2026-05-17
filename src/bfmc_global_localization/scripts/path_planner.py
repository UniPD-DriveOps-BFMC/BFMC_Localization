#!/usr/bin/env python3
"""
BFMC Path Planner
=================
Manages the checkpoint route and publishes the next target waypoint based on
the car's fused global position.

Start modes
-----------
--normal-start  Set EKF initial pose to the known start box (15.48, 3.83 m,
                yaw=0) and begin from the nearest checkpoint (node 306).
--random-start  Wait for the first node ID on /automobile/current_node,
                match it to the nearest checkpoint in CHECKPOINTS,
                and begin from there.

Publishes
---------
/initialpose                    geometry_msgs/PoseWithCovarianceStamped  (once)
/automobile/target_waypoint     geometry_msgs/PoseStamped                (10 Hz)
/automobile/checkpoint_index    std_msgs/Int32                           (10 Hz)

Subscribes
----------
/automobile/current_coordinate  geometry_msgs/PoseStamped
/automobile/current_node        std_msgs/Int32              (random-start only)

Standalone inspection (no ROS needed)
--------------------------------------
  python3 path_planner.py --inspect
"""

import argparse
import math
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ── route ─────────────────────────────────────────────────────────────────────

CHECKPOINTS: List[int] = [
    472, 451, 412, 393, 306, 150, 140, 121,
     92, 109, 130, 147, 175, 133, 123, 118,
     91, 163, 373, 406, 444,
]

# Index in CHECKPOINTS closest to the physical start box — pre-computed.
NORMAL_START_CHECKPOINT_IDX = 0   # node 472 @ (12.92, 2.02) — start box

# ── normal-start pose ─────────────────────────────────────────────────────────

NORMAL_START_X   = 12.49   # m
NORMAL_START_Y   =  1.72   # m
NORMAL_START_YAW     =  0.0                          # rad  (+x, along start straight)
NORMAL_START_STD     =  0.05                         # m    ±0.05 m position
NORMAL_START_YAW_STD =  5.0 * math.pi / 180.0       # rad  ±5 deg yaw

# ── tuning ────────────────────────────────────────────────────────────────────

WAYPOINT_ACCEPT_RADIUS = 0.6   # m — advance when closer than this
INITIAL_POSE_REPEATS   = 5     # publish /initialpose this many times at startup

# ── map ───────────────────────────────────────────────────────────────────────

_MAPS_DIR = (
    Path(__file__).resolve()
    .parent.parent.parent        # src/
    / 'bfmc_map_matching'
    / 'maps'
)
GRAPHML_PATH = _MAPS_DIR / 'Competition_track_graph.graphml'


# ── GraphML loader ────────────────────────────────────────────────────────────

def load_node_coords(graphml_path: Path) -> Dict[int, Tuple[float, float]]:
    """Return {node_id: (x_m, y_m)} from the competition GraphML."""
    if not graphml_path.exists():
        sys.exit(f"ERROR: GraphML not found at {graphml_path}")
    with open(str(graphml_path)) as f:
        content = f.read()
    coords: Dict[int, Tuple[float, float]] = {}
    for nid, x, y in re.findall(
        r'<node id="(\d+)">\s*<data key="d0">([\d.]+)</data>\s*<data key="d1">([\d.]+)</data>',
        content,
    ):
        coords[int(nid)] = (float(x), float(y))
    return coords


# ── helpers ───────────────────────────────────────────────────────────────────

def nearest_checkpoint_idx(
    x: float,
    y: float,
    node_coords: Dict[int, Tuple[float, float]],
) -> int:
    """Return the index into CHECKPOINTS whose node is closest to (x, y)."""
    best_idx, best_dist = 0, float('inf')
    for i, nid in enumerate(CHECKPOINTS):
        if nid not in node_coords:
            continue
        nx, ny = node_coords[nid]
        d = math.hypot(x - nx, y - ny)
        if d < best_dist:
            best_dist, best_idx = d, i
    return best_idx


def _yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """yaw (rad) → (qx, qy, qz, qw) with x=y=0."""
    h = yaw * 0.5
    return 0.0, 0.0, math.sin(h), math.cos(h)


# ── inspection (no ROS) ───────────────────────────────────────────────────────

def cmd_inspect(node_coords: Dict[int, Tuple[float, float]]) -> None:
    missing = [nid for nid in CHECKPOINTS if nid not in node_coords]
    if missing:
        print(f"WARNING: nodes not in GraphML: {missing}", file=sys.stderr)

    print(f"\nCheckpoint route  ({len(CHECKPOINTS)} waypoints)\n")
    print(f"  {'#':>3}  {'Node':>5}  {'x (m)':>8}  {'y (m)':>8}  {'dist_next (m)':>14}")
    print(f"  {'─'*3}  {'─'*5}  {'─'*8}  {'─'*8}  {'─'*14}")
    for i, nid in enumerate(CHECKPOINTS):
        x, y = node_coords.get(nid, (float('nan'), float('nan')))
        ni = CHECKPOINTS[(i + 1) % len(CHECKPOINTS)]
        nx, ny = node_coords.get(ni, (x, y))
        dn = math.hypot(nx - x, ny - y)
        marker = '  ← normal start' if i == NORMAL_START_CHECKPOINT_IDX else ''
        print(f"  {i:>3}  {nid:>5}  {x:>8.4f}  {y:>8.4f}  {dn:>14.2f}{marker}")

    print(f"\nNormal start box : ({NORMAL_START_X}, {NORMAL_START_Y})"
          f"  yaw={NORMAL_START_YAW} rad"
          f"  std=±{NORMAL_START_STD} m  yaw_std=±{math.degrees(NORMAL_START_YAW_STD):.1f} deg")
    print(f"First waypoint   : checkpoint #{NORMAL_START_CHECKPOINT_IDX}"
          f" → node {CHECKPOINTS[NORMAL_START_CHECKPOINT_IDX]}"
          f" @ {node_coords.get(CHECKPOINTS[NORMAL_START_CHECKPOINT_IDX], '?')}\n")


# ── ROS node ──────────────────────────────────────────────────────────────────

def run_ros_node(
    start_mode: str,
    node_coords: Dict[int, Tuple[float, float]],
) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
        from std_msgs.msg import Int32
    except ImportError:
        sys.exit("ERROR: rclpy not found — run inside the ROS2 container.")

    class PathPlannerNode(Node):
        def __init__(self) -> None:
            super().__init__('bfmc_path_planner')
            self._nc    = node_coords
            self._route: List[int] = list(CHECKPOINTS)
            self._idx   = 0
            self._cur_x: Optional[float] = None
            self._cur_y: Optional[float] = None
            self._ready = False
            self._init_remaining = INITIAL_POSE_REPEATS

            self._init_pub = self.create_publisher(
                PoseWithCovarianceStamped, '/initialpose', 1)
            self._wp_pub   = self.create_publisher(
                PoseStamped, '/automobile/target_waypoint', 10)
            self._idx_pub  = self.create_publisher(
                Int32, '/automobile/checkpoint_index', 10)

            self.create_subscription(
                PoseStamped, '/automobile/current_coordinate',
                self._coord_cb, 10)
            self.create_subscription(
                Int32, '/automobile/current_node',
                self._node_cb, 10)

            if start_mode == 'normal':
                self._idx = NORMAL_START_CHECKPOINT_IDX
                self._ready = True
                nid = self._route[self._idx]
                tx, ty = self._nc.get(nid, (0.0, 0.0))
                self.get_logger().info(
                    f'[normal-start] Publishing initial pose '
                    f'({NORMAL_START_X}, {NORMAL_START_Y}) yaw={NORMAL_START_YAW} rad')
                self.get_logger().info(
                    f'[normal-start] First waypoint → node {nid} '
                    f'(checkpoint #{self._idx})  ({tx:.3f}, {ty:.3f})')
            else:
                self.get_logger().info(
                    '[random-start] Waiting for current node '
                    'on /automobile/current_node ...')

            self.create_timer(0.1, self._timer_cb)

        # ── callbacks ────────────────────────────────────────────────────────

        def _coord_cb(self, msg: 'PoseStamped') -> None:
            self._cur_x = msg.pose.position.x
            self._cur_y = msg.pose.position.y

        def _node_cb(self, msg: 'Int32') -> None:
            if self._ready or start_mode != 'random':
                return
            cur_node = msg.data
            if cur_node not in self._nc:
                return
            # Replace CHECKPOINTS[0] with the entry node; rest of route unchanged
            self._route = [cur_node] + list(CHECKPOINTS[1:])
            self._idx   = 0
            self._ready = True
            nid = self._route[1]
            tx, ty = self._nc.get(nid, (0.0, 0.0))
            self.get_logger().info(
                f'[random-start] Entry node {cur_node} '
                f'→ next checkpoint #1 node {nid} ({tx:.3f}, {ty:.3f})')

        def _timer_cb(self) -> None:
            # Repeat /initialpose a few times so EKF is guaranteed to receive it
            if self._init_remaining > 0 and start_mode == 'normal':
                self._publish_initial_pose()
                self._init_remaining -= 1

            if not self._ready or self._cur_x is None:
                return

            nid = self._route[self._idx]
            tx, ty = self._nc.get(nid, (0.0, 0.0))

            # Advance to next checkpoint when close enough
            dist = math.hypot(self._cur_x - tx, self._cur_y - ty)
            if dist < WAYPOINT_ACCEPT_RADIUS:
                self._idx = (self._idx + 1) % len(self._route)
                nid = self._route[self._idx]
                tx, ty = self._nc.get(nid, (0.0, 0.0))
                self.get_logger().info(
                    f'Checkpoint reached → #{self._idx} node {nid} '
                    f'({tx:.3f}, {ty:.3f})  dist_remaining={dist:.2f} m')

            # Publish target waypoint
            wp = PoseStamped()
            wp.header.stamp    = self.get_clock().now().to_msg()
            wp.header.frame_id = 'map'
            wp.pose.position.x = tx
            wp.pose.position.y = ty
            wp.pose.position.z = 0.0
            wp.pose.orientation.w = 1.0
            self._wp_pub.publish(wp)

            im = Int32()
            im.data = self._idx
            self._idx_pub.publish(im)

        # ── initial pose ─────────────────────────────────────────────────────

        def _publish_initial_pose(self) -> None:
            msg = PoseWithCovarianceStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = NORMAL_START_X
            msg.pose.pose.position.y = NORMAL_START_Y
            msg.pose.pose.position.z = 0.0
            _, _, qz, qw = _yaw_to_quat(NORMAL_START_YAW)
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            cov = NORMAL_START_STD ** 2
            msg.pose.covariance[0]  = cov                          # x
            msg.pose.covariance[7]  = cov                          # y
            msg.pose.covariance[35] = NORMAL_START_YAW_STD ** 2   # yaw  (±5 deg)
            self._init_pub.publish(msg)

    rclpy.init()
    rclpy.spin(PathPlannerNode())
    rclpy.shutdown()


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description='BFMC Path Planner — checkpoint route manager',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument(
        '--normal-start', action='store_true',
        help=f'Set EKF pose to start box ({NORMAL_START_X}, {NORMAL_START_Y}) '
             f'and begin at checkpoint #{NORMAL_START_CHECKPOINT_IDX} '
             f'(node {CHECKPOINTS[NORMAL_START_CHECKPOINT_IDX]})',
    )
    mode.add_argument(
        '--random-start', action='store_true',
        help='Find nearest checkpoint to current fused position and begin there',
    )
    ap.add_argument(
        '--inspect', action='store_true',
        help='Print the full route table and exit (no ROS required)',
    )
    args = ap.parse_args()

    node_coords = load_node_coords(GRAPHML_PATH)

    missing = [nid for nid in CHECKPOINTS if nid not in node_coords]
    if missing:
        print(f"WARNING: checkpoint nodes missing from GraphML: {missing}",
              file=sys.stderr)

    if args.inspect:
        cmd_inspect(node_coords)
        return

    start_mode = 'random' if args.random_start else 'normal'
    run_ros_node(start_mode, node_coords)


if __name__ == '__main__':
    main()
