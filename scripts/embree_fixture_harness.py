#!/usr/bin/env python3

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


FIXTURE_NAMES = {
    "baseline": "baseline_fixture.json",
    "dynamic": "dynamic_fixture.json",
    "pinhole": "pinhole_fixture.json",
    "o1dn": "o1dn_fixture.json",
    "ondn": "ondn_fixture.json",
    "plane": "plane_fixture.json",
    "heightmap": "heightmap_fixture.json",
    "ignore_link": "ignore_link_fixture.json",
    "optix_baseline": "optix_baseline_fixture.json",
    "optix_dynamic": "optix_dynamic_fixture.json",
    "optix_noise": "optix_noise_fixture.json",
    "optix_multi": "optix_multi_fixture.json",
    "vertical": "vertical_fixture.json",
    "zombie": "zombie_fixture.json",
}

WORLD_NAMES = {
    "baseline": "gz_embree_baseline.sdf",
    "dynamic": "gz_embree_dynamic.sdf",
    "pinhole": "gz_embree_pinhole.sdf",
    "o1dn": "gz_embree_o1dn.sdf",
    "ondn": "gz_embree_ondn.sdf",
    "plane": "gz_embree_plane.sdf",
    "heightmap": "gz_embree_heightmap.sdf",
    "ignore_link": "gz_embree_ignore_link.sdf",
    # gz_optix_baseline.sdf/gz_optix_dynamic.sdf already existed on disk
    # before this session -- identical scenes to their embree_baseline/
    # embree_dynamic counterparts (only the plugin filenames differ, see
    # rmagine_optix_map_system/rmagine_optix_sensor_system), just never
    # wired into this harness or colcon test at all. Same ray-triangle
    # geometry either way, so the same expected numeric ranges apply.
    "optix_baseline": "gz_optix_baseline.sdf",
    "optix_dynamic": "gz_optix_dynamic.sdf",
    "optix_noise": "gz_optix_noise.sdf",
    "optix_multi": "gz_optix_multi.sdf",
    # Same target_box/sensor_model layout as "baseline" -- see their own
    # world files' comments for what each adds on top of it.
    "vertical": "gz_embree_vertical.sdf",
    "zombie": "gz_embree_zombie.sdf",
}

# Spherical is the only model type with a flat/single-row representation,
# so it's the only one that publishes LaserScan (see
# PublishLaserScanIfApplicable in sensor_model_publish.hpp) -- worlds using
# Pinhole/O1Dn/OnDn never will, by design, not because something is broken.
EXPECTS_SCAN = {
    "baseline": True,
    "dynamic": True,
    "pinhole": False,
    "o1dn": False,
    "ondn": False,
    "plane": True,
    "heightmap": True,
    "ignore_link": True,
    "optix_baseline": True,
    "optix_dynamic": True,
    "optix_noise": True,
    "optix_multi": True,
    # phi.size == 9 (a real multi-ring 3D scan) -- LaserScan is only
    # published for a single-ring (2D) spherical model, see
    # PublishLaserScanIfApplicable in sensor_model_publish.hpp.
    "vertical": False,
    "zombie": True,
}

# O1Dn/OnDn need a `rays_file` SDF element pointing at a real filesystem
# path -- can't hardcode a build-tree-relative or install-tree-relative
# path into a committed .sdf, so the committed world files carry a
# placeholder token instead, resolved to the installed testdata path at
# launch time (see launch_world()).
RAYS_FILES = {
    "o1dn": "o1dn_single_ray.yaml",
    "ondn": "ondn_two_origins.yaml",
}
RAYS_FILE_PLACEHOLDER = "RAYS_FILE_PLACEHOLDER"

# Same placeholder-resolution need as RAYS_FILES, for the heightmap
# fixture's <uri> -- a committed .sdf can't contain a real filesystem path.
HEIGHTMAP_FILES = {
    "heightmap": "heightmap_fixture.png",
}
HEIGHTMAP_URI_PLACEHOLDER = "HEIGHTMAP_URI_PLACEHOLDER"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_output_path(world: str) -> Path:
    return Path("/tmp") / f"rmagine_embree_{world}_capture.json"


def default_log_path(world: str) -> Path:
    return Path("/tmp") / f"rmagine_embree_{world}_gz.log"


def ensure_ros_log_dir() -> None:
    log_dir = Path("/tmp") / "rmagine_gazebo_plugins_ros_logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_LOG_DIR", str(log_dir))


def share_root() -> Path:
    return Path(get_package_share_directory("rmagine_gazebo_plugins"))


def fixture_path(world: str) -> Path:
    return share_root() / "testdata" / "embree_harmonic" / FIXTURE_NAMES[world]


def world_path(world: str) -> Path:
    return share_root() / "worlds" / WORLD_NAMES[world]


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def stamp_to_float(msg) -> Optional[float]:
    if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
      return None
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def finite_values(values: List[float], range_min: float, range_max: float) -> List[float]:
    return [value for value in values if math.isfinite(value) and range_min <= value <= range_max]


def rate_from_stamps(stamps: List[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    deltas = [b - a for a, b in zip(stamps[:-1], stamps[1:]) if b > a]
    if not deltas:
        return 0.0
    return 1.0 / (sum(deltas) / len(deltas))


def summarize_scan(msg: LaserScan) -> Dict[str, object]:
    ranges = list(msg.ranges)
    finite = finite_values(ranges, msg.range_min, msg.range_max)
    center_index = len(ranges) // 2
    window_radius = 5
    window_start = max(0, center_index - window_radius)
    window_stop = min(len(ranges), center_index + window_radius + 1)
    center_window = ranges[window_start:window_stop]
    center_window_finite = finite_values(center_window, msg.range_min, msg.range_max)

    latest = {
        "finite_count": len(finite),
        "finite_ratio": (len(finite) / len(ranges)) if ranges else 0.0,
        "min_finite_range": min(finite) if finite else None,
        "max_finite_range": max(finite) if finite else None,
        "median_finite_range": statistics.median(finite) if finite else None,
        "center_index": center_index,
        "center_value": ranges[center_index] if ranges else None,
        "center_window": center_window,
        "center_window_finite_count": len(center_window_finite),
    }

    return {
        "sensor_config": {
            "frame_id": msg.header.frame_id,
            "beam_count": len(ranges),
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "scan_time": msg.scan_time,
            "time_increment": msg.time_increment,
        },
        "latest": latest,
        "probe": {
            "center_index": center_index,
            "center_value": latest["center_value"],
            "center_window": center_window,
        },
    }


def summarize_cloud(msg: PointCloud2) -> Dict[str, object]:
    finite_points: List[List[float]] = []
    nan_points = 0

    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            finite_points.append([x, y, z])
        else:
            nan_points += 1

    bbox = None
    sample_points = []
    if finite_points:
        xs = [p[0] for p in finite_points]
        ys = [p[1] for p in finite_points]
        zs = [p[2] for p in finite_points]
        bbox = {
            "min": [min(xs), min(ys), min(zs)],
            "max": [max(xs), max(ys), max(zs)],
            "midpoint": [
                (min(xs) + max(xs)) / 2.0,
                (min(ys) + max(ys)) / 2.0,
                (min(zs) + max(zs)) / 2.0,
            ],
        }
        sample_points = finite_points[:5]

    return {
        "sensor_config": {
            "frame_id": msg.header.frame_id,
            "height": msg.height,
            "width": msg.width,
            "point_step": msg.point_step,
            "row_step": msg.row_step,
        },
        "latest": {
            "finite_point_count": len(finite_points),
            "nan_point_count": nan_points,
            "bbox": bbox,
            "sample_points": sample_points,
        },
        "probe": {
            "finite_point_count": len(finite_points),
            "bbox_midpoint_x": bbox["midpoint"][0] if bbox else None,
        },
    }


# Multi-topic fan-out fixtures (gz_optix_multi.sdf/gz_multi_topic_test.sdf
# equivalent) need to prove the EXTRA output topics also receive messages,
# not just the primary topic_scan/topic_points -- otherwise a broken
# fan-out could pass by only ever checking the primary path. Message
# counts only (not full summaries): the falsifiable question here is
# "does the extra topic get fanned-out messages at all", not "are its
# values identical to the primary" (they publish the same underlying scan,
# so that would be redundant with the primary checks already run).
WORLD_EXTRA_TOPICS = {
    "optix_multi": {"scan": ["scan2"], "points": ["points2"]},
}


class TopicCollector(Node):
    def __init__(self, world: str = "") -> None:
        super().__init__("embree_fixture_collector")
        self.scan_messages: List[Dict[str, object]] = []
        self.points_messages: List[Dict[str, object]] = []
        self.extra_scan_counts: Dict[str, int] = {}
        self.extra_points_counts: Dict[str, int] = {}
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.create_subscription(PointCloud2, "points", self._on_points, 10)

        extra = WORLD_EXTRA_TOPICS.get(world, {})
        for topic in extra.get("scan", []):
            self.extra_scan_counts[topic] = 0
            self.create_subscription(
                LaserScan, topic,
                lambda msg, t=topic: self.extra_scan_counts.__setitem__(t, self.extra_scan_counts[t] + 1),
                10)
        for topic in extra.get("points", []):
            self.extra_points_counts[topic] = 0
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, t=topic: self.extra_points_counts.__setitem__(t, self.extra_points_counts[t] + 1),
                10)

    def _on_scan(self, msg: LaserScan) -> None:
        summary = summarize_scan(msg)
        summary["stamp"] = stamp_to_float(msg)
        self.scan_messages.append(summary)

    def _on_points(self, msg: PointCloud2) -> None:
        summary = summarize_cloud(msg)
        summary["stamp"] = stamp_to_float(msg)
        self.points_messages.append(summary)


def finalize_capture(world: str, collector: TopicCollector, duration_sec: float, log_path: Path) -> Dict[str, object]:
    scan_stamps = [msg["stamp"] for msg in collector.scan_messages if msg["stamp"] is not None]
    point_stamps = [msg["stamp"] for msg in collector.points_messages if msg["stamp"] is not None]

    latest_scan = collector.scan_messages[-1] if collector.scan_messages else None
    latest_points = collector.points_messages[-1] if collector.points_messages else None

    center_series = [
        msg["probe"]["center_value"] for msg in collector.scan_messages
        if msg["probe"]["center_value"] is not None and math.isfinite(msg["probe"]["center_value"])
    ]
    center_span = (max(center_series) - min(center_series)) if len(center_series) >= 2 else 0.0
    center_stddev = statistics.pstdev(center_series) if len(center_series) >= 2 else 0.0

    point_count_series = [
        int(msg["probe"]["finite_point_count"]) for msg in collector.points_messages
    ]
    point_count_span = (max(point_count_series) - min(point_count_series)) if len(point_count_series) >= 2 else 0

    bbox_midpoint_series = [
        msg["probe"]["bbox_midpoint_x"] for msg in collector.points_messages
        if msg["probe"]["bbox_midpoint_x"] is not None
    ]
    bbox_midpoint_span = (
        max(bbox_midpoint_series) - min(bbox_midpoint_series)
        if len(bbox_midpoint_series) >= 2 else 0.0
    )

    result = {
        "world": world,
        "world_file": WORLD_NAMES[world],
        "topics": {"scan": "scan", "points": "points"},
        "capture": {
            "started_at": iso_now(),
            "duration_sec": duration_sec,
            "message_timeout_sec": 20.0,
            "gz_log_path": str(log_path),
        },
        "scan": {
            "message_count": len(collector.scan_messages),
            "rate_hz_estimate": rate_from_stamps(scan_stamps),
            "latest": latest_scan["latest"] if latest_scan else None,
            "sensor_config": latest_scan["sensor_config"] if latest_scan else None,
            "series": {
                "center_value": center_series,
                "center_value_span": center_span,
                "center_value_stddev": center_stddev,
                "finite_count": [msg["latest"]["finite_count"] for msg in collector.scan_messages],
            },
            "representative_sample": latest_scan["probe"] if latest_scan else None,
        },
        "points": {
            "message_count": len(collector.points_messages),
            "rate_hz_estimate": rate_from_stamps(point_stamps),
            "latest": latest_points["latest"] if latest_points else None,
            "sensor_config": latest_points["sensor_config"] if latest_points else None,
            "series": {
                "finite_point_count": point_count_series,
                "finite_point_count_span": point_count_span,
                "bbox_midpoint_x": bbox_midpoint_series,
                "bbox_midpoint_x_span": bbox_midpoint_span,
            },
            "representative_sample": latest_points["latest"]["sample_points"] if latest_points else None,
        },
        "extra_topics": {
            "scan": dict(collector.extra_scan_counts),
            "points": dict(collector.extra_points_counts),
        },
    }
    return result


def resolved_world_path(world: str) -> Path:
    """Returns a world file ready to hand to `gz sim`. For worlds with a
    rays_file/heightmap-uri placeholder, writes a resolved copy to /tmp
    first (the committed .sdf can't contain a real filesystem path -- see
    RAYS_FILES/RAYS_FILE_PLACEHOLDER and HEIGHTMAP_FILES/
    HEIGHTMAP_URI_PLACEHOLDER)."""
    if world not in RAYS_FILES and world not in HEIGHTMAP_FILES:
        return world_path(world)

    text = world_path(world).read_text(encoding="utf-8")
    if world in RAYS_FILES:
        rays_file = share_root() / "testdata" / "embree_harmonic" / RAYS_FILES[world]
        text = text.replace(RAYS_FILE_PLACEHOLDER, str(rays_file))
    if world in HEIGHTMAP_FILES:
        heightmap_file = share_root() / "testdata" / "embree_harmonic" / HEIGHTMAP_FILES[world]
        text = text.replace(HEIGHTMAP_URI_PLACEHOLDER, str(heightmap_file))
    resolved = Path("/tmp") / f"rmagine_embree_{world}_resolved.sdf"
    resolved.write_text(text, encoding="utf-8")
    return resolved


def launch_world(world: str, log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    cmd = [
        "gz",
        "sim",
        "-s",
        "-r",
        "--headless-rendering",
        "-v",
        "1",
        str(resolved_world_path(world)),
    ]
    return subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )


def stop_world(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)


def capture_world(world: str, output_path: Path, duration_sec: float, timeout_sec: float) -> int:
    log_path = default_log_path(world)
    process = launch_world(world, log_path)
    ensure_ros_log_dir()
    rclpy.init(args=None)
    collector = TopicCollector(world)

    try:
        first_deadline = time.monotonic() + timeout_sec
        while time.monotonic() < first_deadline:
            if process.poll() is not None:
                print(f"gz sim exited early with code {process.returncode}. See {log_path}", file=sys.stderr)
                return 1
            rclpy.spin_once(collector, timeout_sec=0.1)
            have_scan = (not EXPECTS_SCAN[world]) or bool(collector.scan_messages)
            if have_scan and collector.points_messages:
                break
        else:
            print(f"Timed out waiting for /scan and /points. See {log_path}", file=sys.stderr)
            return 1

        capture_end = time.monotonic() + duration_sec
        while time.monotonic() < capture_end:
            if process.poll() is not None:
                print(f"gz sim exited during capture with code {process.returncode}. See {log_path}", file=sys.stderr)
                return 1
            rclpy.spin_once(collector, timeout_sec=0.1)

        result = finalize_capture(world, collector, duration_sec, log_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Wrote capture for {world} to {output_path}")
        print(f"Gazebo log saved to {log_path}")
        return 0
    finally:
        try:
            collector.destroy_node()
        finally:
            rclpy.shutdown()
            stop_world(process)


def load_json(path: Path) -> Dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def ensure(condition: bool, message: str, failures: List[str]) -> None:
    if not condition:
        failures.append(message)


def compare_metric_range(
    actual: Optional[float],
    expected: Dict[str, float],
    label: str,
    failures: List[str],
) -> None:
    if actual is None:
        failures.append(f"{label}: missing value")
        return
    minimum = expected.get("min")
    maximum = expected.get("max")
    if minimum is not None and actual < minimum:
        failures.append(f"{label}: {actual:.3f} < min {minimum:.3f}")
    if maximum is not None and actual > maximum:
        failures.append(f"{label}: {actual:.3f} > max {maximum:.3f}")


def compare_capture(world: str, result_path: Path) -> int:
    fixture = load_json(fixture_path(world))
    result = load_json(result_path)
    failures: List[str] = []

    ensure(result.get("world") == world, f"world mismatch: expected {world}, got {result.get('world')}", failures)

    scan = result.get("scan") or {}
    points = result.get("points") or {}
    scan_latest = scan.get("latest") or {}
    point_latest = points.get("latest") or {}
    # Every check below is keyed off whether its field is present in the
    # fixture's "expected" dict, not off the world's name -- lets
    # scan-less worlds (Pinhole/O1Dn/OnDn, see EXPECTS_SCAN) and
    # static/dynamic worlds share one comparison path by simply omitting
    # the fields that don't apply to them, instead of an `if world ==
    # "baseline"` special case per new world.
    expected_scan = fixture["expected"].get("scan")
    expected_points = fixture["expected"].get("points")

    if expected_scan:
        if "min_messages" in expected_scan:
            ensure(scan.get("message_count", 0) >= expected_scan["min_messages"],
                   f"{world} scan message_count {scan.get('message_count', 0)} < {expected_scan['min_messages']}",
                   failures)
        if "min_rate_hz" in expected_scan:
            ensure(scan.get("rate_hz_estimate", 0.0) >= expected_scan["min_rate_hz"],
                   f"{world} scan rate {scan.get('rate_hz_estimate', 0.0):.3f} < {expected_scan['min_rate_hz']:.3f}",
                   failures)
        if "latest_finite_count_min" in expected_scan:
            ensure(scan_latest.get("finite_count", 0) >= expected_scan["latest_finite_count_min"],
                   f"{world} scan finite_count {scan_latest.get('finite_count', 0)} < {expected_scan['latest_finite_count_min']}",
                   failures)
        if "center_value_range" in expected_scan:
            compare_metric_range(
                scan_latest.get("center_value"), expected_scan["center_value_range"],
                f"{world} scan center_value", failures)
        if "center_value_stddev_max" in expected_scan:
            ensure(scan.get("series", {}).get("center_value_stddev", 0.0) <= expected_scan["center_value_stddev_max"],
                   f"{world} scan center stddev {scan.get('series', {}).get('center_value_stddev', 0.0):.3f} > {expected_scan['center_value_stddev_max']:.3f}",
                   failures)
        if "center_value_stddev_min" in expected_scan:
            ensure(scan.get("series", {}).get("center_value_stddev", 0.0) >= expected_scan["center_value_stddev_min"],
                   f"{world} scan center stddev {scan.get('series', {}).get('center_value_stddev', 0.0):.3f} < {expected_scan['center_value_stddev_min']:.3f}",
                   failures)
        if "center_value_span_min" in expected_scan:
            ensure(scan.get("series", {}).get("center_value_span", 0.0) >= expected_scan["center_value_span_min"],
                   f"{world} scan center span {scan.get('series', {}).get('center_value_span', 0.0):.3f} < {expected_scan['center_value_span_min']:.3f}",
                   failures)
        if "center_window_finite_min" in expected_scan:
            ensure(scan_latest.get("center_window_finite_count", 0) >= expected_scan["center_window_finite_min"],
                   f"{world} scan center_window_finite_count {scan_latest.get('center_window_finite_count', 0)} < {expected_scan['center_window_finite_min']}",
                   failures)

    if expected_points:
        if "min_messages" in expected_points:
            ensure(points.get("message_count", 0) >= expected_points["min_messages"],
                   f"{world} points message_count {points.get('message_count', 0)} < {expected_points['min_messages']}",
                   failures)
        if "min_rate_hz" in expected_points:
            ensure(points.get("rate_hz_estimate", 0.0) >= expected_points["min_rate_hz"],
                   f"{world} points rate {points.get('rate_hz_estimate', 0.0):.3f} < {expected_points['min_rate_hz']:.3f}",
                   failures)
        if "latest_finite_points_min" in expected_points:
            ensure(point_latest.get("finite_point_count", 0) >= expected_points["latest_finite_points_min"],
                   f"{world} points finite_point_count {point_latest.get('finite_point_count', 0)} < {expected_points['latest_finite_points_min']}",
                   failures)

        bbox = point_latest.get("bbox")
        if "bbox_midpoint_x_range" in expected_points:
            if not bbox:
                failures.append(f"{world} points bbox missing")
            else:
                compare_metric_range(bbox["midpoint"][0], expected_points["bbox_midpoint_x_range"],
                                      f"{world} points bbox_midpoint_x", failures)
        if "bbox_y_span_min" in expected_points:
            if not bbox:
                failures.append(f"{world} points bbox missing (for y span check)")
            else:
                ensure((bbox["max"][1] - bbox["min"][1]) >= expected_points["bbox_y_span_min"],
                       f"{world} points bbox y span {(bbox['max'][1] - bbox['min'][1]):.3f} < {expected_points['bbox_y_span_min']:.3f}",
                       failures)
        if "bbox_midpoint_x_span_min" in expected_points:
            ensure(points.get("series", {}).get("bbox_midpoint_x_span", 0.0) >= expected_points["bbox_midpoint_x_span_min"],
                   f"{world} points bbox_midpoint_x span {points.get('series', {}).get('bbox_midpoint_x_span', 0.0):.3f} < {expected_points['bbox_midpoint_x_span_min']:.3f}",
                   failures)
        if "finite_point_count_span_min" in expected_points:
            ensure(points.get("series", {}).get("finite_point_count_span", 0) >= expected_points["finite_point_count_span_min"],
                   f"{world} finite_point_count span {points.get('series', {}).get('finite_point_count_span', 0)} < {expected_points['finite_point_count_span_min']}",
                   failures)

    # Multi-topic fan-out: prove the EXTRA output topics actually received
    # messages, not just the primary topic_scan/topic_points.
    expected_extra = fixture["expected"].get("extra_topics")
    if expected_extra:
        extra_result = result.get("extra_topics") or {}
        for kind in ("scan", "points"):
            for topic, min_count in expected_extra.get(kind, {}).items():
                actual = extra_result.get(kind, {}).get(topic, 0)
                ensure(actual >= min_count,
                       f"{world} extra {kind} topic '{topic}' message_count {actual} < {min_count} "
                       f"(fan-out not reaching this topic)",
                       failures)

    if failures:
        print(f"Comparison against {fixture_path(world)} failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print(f"{world} capture matches expected fixture {fixture_path(world)}")
    return 0


def parse_args(argv: List[str]) -> argparse.Namespace:
    command_name = Path(argv[0]).name
    parser = argparse.ArgumentParser(prog=command_name)
    parser.add_argument("world", choices=sorted(FIXTURE_NAMES.keys()))
    parser.add_argument("--output", type=Path, help="Output path for capture JSON")
    parser.add_argument("--result", type=Path, help="Result JSON to compare")
    parser.add_argument("--duration", type=float, help="Capture duration in seconds")
    parser.add_argument("--timeout", type=float, default=20.0, help="Timeout waiting for the first scan and point cloud")
    return parser.parse_args(argv[1:])


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    command_name = Path(argv[0]).name

    if "capture" in command_name:
        output = args.output or default_output_path(args.world)
        fixture = load_json(fixture_path(args.world))
        duration = args.duration or fixture["capture_defaults"]["duration_sec"]
        return capture_world(args.world, output, float(duration), float(args.timeout))

    if "compare" in command_name:
        result = args.result or default_output_path(args.world)
        if not result.exists():
            print(f"Result file not found: {result}", file=sys.stderr)
            return 1
        return compare_capture(args.world, result)

    print(f"Unsupported entrypoint name: {command_name}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
