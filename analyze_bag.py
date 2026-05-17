#!/usr/bin/env python3
"""
BFMC Localization — Rosbag Plotter

Reads a ROS2 bag (SQLite3 format, no ROS install required) and produces:
  plot_distance.png — encoder/distance vs odom_distance vs total_distance
  plot_velocity.png — odom_velocity vs current_speed
  plot_map.png      — GPS localisation + fused position track on the competition map

Usage:
  python3 analyze_bag.py <bag_dir> [--map-dir <path>] [--out-dir <path>] [--show]

Example:
  python3 analyze_bag.py ~/bfmc_bags/bfmc_20260101_120000
  python3 analyze_bag.py ~/bfmc_bags/bfmc_20260101_120000 --show

Requirements:
  pip install matplotlib numpy
"""

import argparse
import sqlite3
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np


# ── CDR deserializer ──────────────────────────────────────────────────────────

class CdrReader:
    """
    Minimal CDR deserializer for ROS2 serialized messages.
    The first 4 bytes are the CDR encapsulation header; byte 1 signals endianness
    (0 = big-endian, 1 = little-endian). All alignment is relative to byte 0.
    """

    def __init__(self, data: bytes):
        self.buf = data
        self.pos = 4                    # skip 4-byte CDR encapsulation header
        self.le = len(data) > 1 and data[1] == 1

    def _align(self, n: int) -> None:
        r = self.pos % n
        if r:
            self.pos += n - r

    def u32(self) -> int:
        self._align(4)
        v = struct.unpack_from('<I' if self.le else '>I', self.buf, self.pos)[0]
        self.pos += 4
        return v

    def i32(self) -> int:
        self._align(4)
        v = struct.unpack_from('<i' if self.le else '>i', self.buf, self.pos)[0]
        self.pos += 4
        return v

    def f32(self) -> float:
        self._align(4)
        v = struct.unpack_from('<f' if self.le else '>f', self.buf, self.pos)[0]
        self.pos += 4
        return v

    def f64(self) -> float:
        self._align(8)
        v = struct.unpack_from('<d' if self.le else '>d', self.buf, self.pos)[0]
        self.pos += 8
        return v

    def string(self) -> str:
        n = self.u32()
        if n == 0:
            return ''
        s = self.buf[self.pos:self.pos + n - 1].decode('utf-8', errors='replace')
        self.pos += n
        return s

    def stamp(self) -> float:
        """builtin_interfaces/Time → float seconds"""
        sec = self.u32()
        nsec = self.u32()
        return sec + nsec * 1e-9

    def header(self) -> Tuple[float, str]:
        """std_msgs/Header → (timestamp_s, frame_id)"""
        t = self.stamp()
        frame_id = self.string()
        return t, frame_id


# ── per-type parsers ──────────────────────────────────────────────────────────

def _parse_float32(data: bytes) -> dict:
    """std_msgs/Float32"""
    return {'value': CdrReader(data).f32()}


def _parse_float64(data: bytes) -> dict:
    """std_msgs/Float64"""
    return {'value': CdrReader(data).f64()}


def _parse_gps_tag_pose(data: bytes) -> dict:
    """bfmc_gps_position/GpsTagPose: header + float32 x/y/z + int32 quality"""
    r = CdrReader(data)
    t, _ = r.header()
    x, y, z = r.f32(), r.f32(), r.f32()
    quality = r.i32()
    return {'t': t, 'x': float(x), 'y': float(y), 'z': float(z), 'quality': quality}


def _parse_pose_stamped(data: bytes) -> dict:
    """geometry_msgs/PoseStamped → t, frame_id, x, y"""
    r = CdrReader(data)
    t, frame_id = r.header()
    x = r.f64()
    y = r.f64()
    return {'t': t, 'frame_id': frame_id, 'x': x, 'y': y}


def _parse_pose_with_covariance_stamped(data: bytes) -> dict:
    """geometry_msgs/PoseWithCovarianceStamped → t, frame_id, x, y"""
    r = CdrReader(data)
    t, frame_id = r.header()
    x = r.f64()
    y = r.f64()
    return {'t': t, 'frame_id': frame_id, 'x': x, 'y': y}


# ── bag reader ────────────────────────────────────────────────────────────────

_PARSERS = {
    '/automobile/encoder/distance':     _parse_float32,
    '/odom_distance':                   _parse_float32,
    '/automobile/total_distance':       _parse_float64,
    '/odom_velocity':                   _parse_float32,
    '/automobile/current_speed':        _parse_float64,
    '/automobile/localisation':         _parse_gps_tag_pose,
    '/automobile/gps/base_pose':        _parse_pose_with_covariance_stamped,
    '/automobile/current_coordinate':   _parse_pose_stamped,
}


def read_bag(bag_dir: Path) -> Dict[str, List[dict]]:
    """Read all relevant topics from the bag's SQLite3 database."""
    db_files = sorted(bag_dir.glob('*.db3'))
    if not db_files:
        sys.exit(f"ERROR: No .db3 file found in {bag_dir}\n"
                 "  Is this a valid ROS2 bag directory?")

    data: Dict[str, List[dict]] = {topic: [] for topic in _PARSERS}

    for db_path in db_files:
        with sqlite3.connect(str(db_path)) as conn:
            conn.row_factory = sqlite3.Row
            cur = conn.cursor()

            cur.execute("SELECT id, name FROM topics")
            topic_map = {row['id']: row['name'] for row in cur.fetchall()}

            wanted = {name: tid for tid, name in topic_map.items() if name in _PARSERS}

            for topic_name, topic_id in wanted.items():
                cur.execute(
                    "SELECT timestamp, data FROM messages "
                    "WHERE topic_id=? ORDER BY timestamp",
                    (topic_id,)
                )
                parser = _PARSERS[topic_name]
                for row in cur.fetchall():
                    try:
                        msg = parser(bytes(row['data']))
                        msg['bag_t'] = row['timestamp'] * 1e-9
                        data[topic_name].append(msg)
                    except Exception:
                        pass  # skip malformed messages

    return data


# ── coordinate helpers ────────────────────────────────────────────────────────

def _parse_graphml_bounds(graphml_path: Path) -> Optional[Tuple[float, float, float, float]]:
    """Extract world bounds (x_min, x_max, y_min, y_max) in metres from competition GraphML."""
    try:
        tree = ET.parse(str(graphml_path))
    except Exception as e:
        print(f"WARNING: Could not parse GraphML: {e}")
        return None

    xs, ys = [], []
    for elem in tree.iter():
        tag = elem.tag.split('}')[-1]
        if tag == 'data' and elem.text:
            key = elem.get('key', '')
            try:
                if key == 'd0':
                    xs.append(float(elem.text))
                elif key == 'd1':
                    ys.append(float(elem.text))
            except ValueError:
                pass

    if not xs or not ys:
        return None
    return min(xs), max(xs), min(ys), max(ys)


# ── plotting ──────────────────────────────────────────────────────────────────

def _time_and_values(
    records: List[dict],
    value_key: str = 'value',
) -> Tuple[np.ndarray, np.ndarray]:
    """Extract (relative_time, values) arrays from message records."""
    if not records:
        return np.array([]), np.array([])
    t0 = records[0]['bag_t']
    ts = np.array([r['bag_t'] - t0 for r in records])
    vs = np.array([r.get(value_key, float('nan')) for r in records], dtype=float)
    return ts, vs


def _no_data_text(ax, msg: str = 'No data in bag') -> None:
    ax.text(0.5, 0.5, msg, ha='center', va='center', transform=ax.transAxes,
            fontsize=13, color='grey', style='italic')


def plot_distance(data: Dict, out_path: Path, show: bool) -> None:
    fig, ax = plt.subplots(figsize=(13, 5))

    series = [
        ('/automobile/encoder/distance', 'Encoder distance (raw firmware)',
         'tab:blue',   '-',   1.8),
        ('/odom_distance',               'Odom distance (local EKF)',
         'tab:orange', '--',  1.6),
        ('/automobile/total_distance',   'Total distance (fused, global EKF)',
         'tab:green',  '-.',  1.6),
    ]

    any_data = False
    for topic, label, color, ls, lw in series:
        records = data.get(topic, [])
        ts, vs = _time_and_values(records)
        if len(ts) == 0:
            continue
        # Normalize so all series start at 0
        vs = vs - vs[0]
        ax.plot(ts, vs, ls, color=color, label=label, linewidth=lw)
        any_data = True

    if not any_data:
        _no_data_text(ax)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Distance comparison — encoder / odom / fused')
    ax.legend()
    ax.grid(True, alpha=0.35)
    fig.tight_layout()

    if show:
        plt.show()
    else:
        fig.savefig(str(out_path), dpi=150)
        plt.close(fig)
        print(f"  Saved: {out_path}")


def plot_velocity(data: Dict, out_path: Path, show: bool) -> None:
    fig, ax = plt.subplots(figsize=(13, 5))

    series = [
        ('/odom_velocity',            'Odom velocity (local EKF vx)',
         'tab:blue',   '-',   1.8),
        ('/automobile/current_speed', 'Current speed (fused magnitude)',
         'tab:orange', '--',  1.6),
    ]

    any_data = False
    for topic, label, color, ls, lw in series:
        records = data.get(topic, [])
        ts, vs = _time_and_values(records)
        if len(ts) == 0:
            continue
        ax.plot(ts, vs, ls, color=color, label=label, linewidth=lw)
        any_data = True

    if not any_data:
        _no_data_text(ax)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title('Velocity comparison — odom / fused')
    ax.legend()
    ax.grid(True, alpha=0.35)
    fig.tight_layout()

    if show:
        plt.show()
    else:
        fig.savefig(str(out_path), dpi=150)
        plt.close(fig)
        print(f"  Saved: {out_path}")


def plot_map(
    data: Dict,
    map_png: Optional[Path],
    graphml: Optional[Path],
    out_path: Path,
    show: bool,
) -> None:
    from matplotlib.collections import LineCollection
    from matplotlib.colors import Normalize
    from matplotlib.lines import Line2D

    fig, ax = plt.subplots(figsize=(11, 11))

    # ── parse world bounds ────────────────────────────────────────────────────
    bounds = _parse_graphml_bounds(graphml) if graphml and graphml.exists() else None

    if bounds:
        x_min, x_max, y_min, y_max = bounds
    else:
        x_min, x_max, y_min, y_max = 0.0, 1.0, 0.0, 1.0
        ax.text(0.5, 0.98, 'GraphML not found — coordinates shown as-is (metres)',
                ha='center', va='top', transform=ax.transAxes,
                fontsize=9, color='red')

    # ── background map in world coordinates ──────────────────────────────────
    # imshow extent=[left, right, bottom, top] with origin='upper':
    #   row 0 of the image → y_max (top), last row → y_min (bottom).
    # This matches map_y_flipped=True used by the map-matching node.
    if map_png and map_png.exists() and bounds:
        img = mpimg.imread(str(map_png))
        ax.imshow(img,
                  extent=[x_min, x_max, y_min, y_max],
                  origin='upper', aspect='equal', zorder=0)
    else:
        ax.set_facecolor('#1a1a2e')
        if bounds:
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
        if not map_png:
            ax.text(0.5, 0.01, 'Map image not found — use --map-dir to specify location',
                    ha='center', va='bottom', transform=ax.transAxes,
                    fontsize=9, color='yellow')

    # All data below is plotted directly in world coordinates (metres).
    any_data = False

    # ── raw GPS localisation — coloured by quality ────────────────────────────
    gps_records = data.get('/automobile/localisation', [])
    if gps_records:
        xs = [r['x'] for r in gps_records]
        ys = [r['y'] for r in gps_records]
        qualities = [r['quality'] for r in gps_records]
        sc = ax.scatter(xs, ys,
                        c=qualities, cmap='RdYlGn', s=20, alpha=0.75,
                        vmin=0, vmax=100, zorder=3,
                        label='/automobile/localisation (raw GPS)')
        cbar = plt.colorbar(sc, ax=ax, fraction=0.025, pad=0.01)
        cbar.set_label('GPS quality (0–100)', fontsize=9)
        any_data = True

    # ── EKF-corrected GPS base pose ───────────────────────────────────────────
    gps_base_records = data.get('/automobile/gps/base_pose', [])
    if gps_base_records:
        xs = [r['x'] for r in gps_base_records]
        ys = [r['y'] for r in gps_base_records]
        ax.scatter(xs, ys,
                   color='yellow', s=16, alpha=0.65, marker='^', zorder=4,
                   label='/automobile/gps/base_pose (EKF GPS)')
        any_data = True

    # ── fused position track — coloured by time ───────────────────────────────
    coord_records = data.get('/automobile/current_coordinate', [])
    if coord_records:
        map_recs = [r for r in coord_records if r.get('frame_id', 'map') == 'map']
        if not map_recs:
            map_recs = coord_records
        xs = np.array([r['x'] for r in map_recs])
        ys = np.array([r['y'] for r in map_recs])

        points = np.stack([xs, ys], axis=1).reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        norm = Normalize(vmin=0, vmax=max(len(segments) - 1, 1))
        lc = LineCollection(segments, cmap='cool', norm=norm, linewidth=2.2, zorder=5)
        lc.set_array(np.arange(len(segments), dtype=float))
        ax.add_collection(lc)

        ax.plot(xs[0], ys[0], 'o', color='lime', markersize=11, zorder=7,
                label='Start', markeredgecolor='black', markeredgewidth=0.8)
        ax.plot(xs[-1], ys[-1], 's', color='red', markersize=11, zorder=7,
                label='End', markeredgecolor='black', markeredgewidth=0.8)
        ax.add_artist(Line2D([0], [0], color='dodgerblue', linewidth=2.2,
                             label='/automobile/current_coordinate (time: blue→red)'))
        any_data = True

    if not any_data:
        _no_data_text(ax, 'No position data in bag')

    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_title('Position on competition map  [world coordinates in metres]')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.2, color='white' if not (map_png and map_png.exists()) else 'grey')
    fig.tight_layout(pad=0.5)

    if show:
        plt.show()
    else:
        fig.savefig(str(out_path), dpi=150)
        plt.close(fig)
        print(f"  Saved: {out_path}")


# ── main ──────────────────────────────────────────────────────────────────────

def _find_map_dir(script_dir: Path, user_arg: Optional[str]) -> Optional[Path]:
    candidates = []
    if user_arg:
        candidates.append(Path(user_arg).expanduser())
    candidates += [
        script_dir / 'src/bfmc_map_matching/maps',
        script_dir.parent / 'src/bfmc_map_matching/maps',
    ]
    for d in candidates:
        if (d / 'Competition_track_graph.png').exists():
            return d
    return None


def main() -> None:
    ap = argparse.ArgumentParser(
        description='BFMC Localization — rosbag plotter (no ROS install required)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument('bag', metavar='BAG_DIR',
                    help='ROS2 bag directory (contains *.db3 + metadata.yaml)')
    ap.add_argument('--map-dir', default=None,
                    help='Directory with Competition_track_graph.{png,graphml} '
                         '(auto-detected from script location if omitted)')
    ap.add_argument('--out-dir', default=None,
                    help='Output directory for PNGs (default: same as bag dir)')
    ap.add_argument('--show', action='store_true',
                    help='Show plots interactively (requires a display)')
    args = ap.parse_args()

    bag_dir = Path(args.bag).expanduser().resolve()
    if not bag_dir.is_dir():
        sys.exit(f"ERROR: {bag_dir} is not a directory")

    out_dir = Path(args.out_dir).expanduser() if args.out_dir else bag_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    script_dir = Path(__file__).resolve().parent
    map_dir = _find_map_dir(script_dir, args.map_dir)
    map_png     = map_dir / 'Competition_track_graph.png'     if map_dir else None
    map_graphml = map_dir / 'Competition_track_graph.graphml' if map_dir else None

    if map_dir:
        print(f"Map directory : {map_dir}")
    else:
        print("WARNING: Map files not found. Use --map-dir <path> to specify location.")

    print(f"Reading bag   : {bag_dir}")
    data = read_bag(bag_dir)

    print("\nMessages read per topic:")
    for topic in _PARSERS:
        n = len(data[topic])
        status = f"{n:>6} msgs" if n else "       — (not in bag)"
        print(f"  {topic:<45} {status}")

    print("\nGenerating plots ...")
    show = args.show

    plot_distance(data, out_dir / 'plot_distance.png', show)
    plot_velocity(data, out_dir / 'plot_velocity.png', show)
    plot_map(data, map_png, map_graphml, out_dir / 'plot_map.png', show)

    if not show:
        print(f"\nAll plots saved to: {out_dir}")


if __name__ == '__main__':
    main()
