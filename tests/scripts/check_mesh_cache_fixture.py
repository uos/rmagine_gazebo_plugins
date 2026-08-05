#!/usr/bin/env python3
"""Regression check for mesh-by-URI caching (README.md, "Mesh-by-URI
caching") -- the last of this migration's Classic-parity test-coverage
gaps that didn't fit the capture/compare-JSON pattern the other fixtures
use (embree_fixture_harness.py). This one is about an internal behavior
(how many times the mesh actually gets loaded from disk), not published
topic values, so it's a standalone log-grep + topic sanity check instead.

Launches gz_embree_mesh_cache.sdf (three static entities, all referencing
the same mesh file, at three different poses, plus an unrelated moving
box that forces many rebuild passes over the capture window) and checks
two things:

1. The map system's own debug log shows exactly 1 "(cache miss)" line for
   that mesh URI, total, ever -- despite 3 entities sharing it and dozens
   of rebuild passes over the capture window. Any other entity/pass
   referencing the same (uri, mesh_scale) key should always get a cache
   hit once the first load populates it. A count > 1 means caching isn't
   actually being shared -- either within a pass (multiple entities
   re-loading the same never-before-cached file) or across passes
   (re-loading a file that was already cached earlier). This is exactly
   the real bug found while first building this fixture: an earlier
   version of rmagine_embree_map_system.cpp's/rmagine_optix_map_system.cpp's
   caching gated hits behind an unnecessary per-pass first-use guard,
   so only the FIRST entity sharing a cached mesh in any given pass got
   the hit -- every other one reloaded from disk every single pass,
   forever. Fixed on both backends (see their own code comments).
2. The sensor topics still produce real, non-degenerate data -- a
   regression where the cache/instance system corrupted transforms (all
   three boxes collapsing onto one shared, wrongly-transformed geometry)
   would still show *some* signal, just wrong, so this is a coarse sanity
   check, not the falsifiable check for that failure mode (#1 already
   covers the caching behavior itself; a wrong-transform regression would
   most likely also show up as the existing plane/heightmap/ignore_link
   fixtures' own checks drifting, since they'd share the same underlying
   bug in EmbreeMapSystem's caching code path).
"""

import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


MESH_CACHE_URI_PLACEHOLDER = "MESH_CACHE_URI_PLACEHOLDER"
WORLD_FILE = "gz_embree_mesh_cache.sdf"
MESH_FILE = "box_test.ply"


def share_root() -> Path:
    return Path(get_package_share_directory("rmagine_gazebo_plugins"))


def resolved_world_path() -> Path:
    mesh_path = share_root() / "testdata" / "embree_harmonic" / MESH_FILE
    world_text = (share_root() / "worlds" / WORLD_FILE).read_text(encoding="utf-8")
    world_text = world_text.replace(MESH_CACHE_URI_PLACEHOLDER, str(mesh_path))
    resolved = Path("/tmp") / "rmagine_embree_mesh_cache_resolved.sdf"
    resolved.write_text(world_text, encoding="utf-8")
    return resolved, mesh_path


class TopicCollector(Node):
    def __init__(self) -> None:
        super().__init__("mesh_cache_fixture_collector")
        self.scan_count = 0
        self.points_count = 0
        self.latest_finite_count = 0
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.create_subscription(PointCloud2, "points", self._on_points, 10)

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan_count += 1
        self.latest_finite_count = sum(
            1 for r in msg.ranges if r == r and msg.range_min <= r <= msg.range_max)

    def _on_points(self, msg: PointCloud2) -> None:
        self.points_count += 1


def gz_sim_command() -> list:
    """`gz sim` on Harmonic+ (Jazzy); Fortress (Humble) only ships the `ign`
    CLI, invoked as `ign gazebo` -- same flags, same behavior."""
    if shutil.which("gz"):
        return ["gz", "sim"]
    return ["ign", "gazebo"]


def main() -> int:
    world_path, mesh_path = resolved_world_path()
    log_path = Path("/tmp/rmagine_embree_mesh_cache_gz.log")
    log_file = log_path.open("w", encoding="utf-8")

    process = subprocess.Popen(
        gz_sim_command() + ["-s", "-r", "--headless-rendering", "-v", "1", str(world_path)],
        stdout=log_file, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
    )

    # The sensor system only publishes gz-native messages now (see
    # embree_fixture_harness.py's own identical comment on
    # ros_gz_bridge_fixtures.yaml) -- bridge "scan"/"points" to /scan,
    # /points so this fixture's rclpy TopicCollector keeps working unchanged.
    bridge_log_path = Path("/tmp/rmagine_embree_mesh_cache_bridge.log")
    bridge_log_file = bridge_log_path.open("w", encoding="utf-8")
    bridge_config = (
        share_root() / "testdata" / "embree_harmonic" / "ros_gz_bridge_fixtures.yaml"
    )
    bridge_process = subprocess.Popen(
        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
         "--ros-args", "-p", f"config_file:={bridge_config}"],
        stdout=bridge_log_file, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
    )

    rclpy.init(args=None)
    collector = TopicCollector()
    failures = []

    def cache_miss_count() -> int:
        text = log_path.read_text(encoding="utf-8", errors="replace")
        return text.count(f"Loading mesh from '{mesh_path}' (cache miss).")

    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                print(f"gz sim exited early with code {process.returncode}. See {log_path}", file=sys.stderr)
                return 1
            if bridge_process.poll() is not None:
                print(f"ros_gz_bridge exited early with code {bridge_process.returncode}. "
                      f"See {bridge_log_path}", file=sys.stderr)
                return 1
            rclpy.spin_once(collector, timeout_sec=0.1)
            if collector.scan_count >= 3 and collector.points_count >= 3:
                break
        else:
            print(f"Timed out waiting for scan/points. See {log_path} and {bridge_log_path}", file=sys.stderr)
            return 1

        # Snapshot the count once the initial pass has settled, then let
        # the mover force several more seconds' worth of rebuild passes
        # (period 2.0s, so this window covers multiple full cycles) and
        # snapshot again -- the real check is that these two numbers
        # match, not just that the first one happens to be 3.
        early_count = cache_miss_count()
        late_deadline = time.monotonic() + 6.0
        while time.monotonic() < late_deadline:
            if process.poll() is not None:
                print(f"gz sim exited early with code {process.returncode}. See {log_path}", file=sys.stderr)
                return 1
            rclpy.spin_once(collector, timeout_sec=0.1)
    finally:
        collector.destroy_node()
        rclpy.shutdown()
        if bridge_process.poll() is None:
            os.killpg(bridge_process.pid, signal.SIGINT)
            try:
                bridge_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(bridge_process.pid, signal.SIGKILL)
                bridge_process.wait(timeout=5.0)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)

    late_count = cache_miss_count()

    if early_count != 1:
        failures.append(
            f"expected exactly 1 cache-miss load of '{mesh_path}' total, shared across all 3 "
            f"entities that reference it, but found {early_count} after the first settle "
            f"(see {log_path}) -- mesh-by-URI caching is not sharing the loaded mesh across "
            f"entities within the initial pass"
        )

    if late_count != early_count:
        failures.append(
            f"cache-miss count grew from {early_count} to {late_count} across ~6s of further "
            f"rebuild passes forced by the mover (see {log_path}) -- mesh-by-URI caching is not "
            f"being reused ACROSS rebuild passes, re-loading the same file from disk every rebuild"
        )

    if collector.latest_finite_count < 5:
        failures.append(
            f"latest scan finite_count {collector.latest_finite_count} < 5 -- "
            f"no real signal from the 3 target boxes"
        )

    if failures:
        print("mesh_cache fixture failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print(f"mesh_cache fixture passed: {early_count} cache-miss load(s) initially, still {late_count} "
          f"after further rebuild passes, for 3 entities sharing '{mesh_path}'. "
          f"{collector.scan_count} scan / {collector.points_count} points messages, "
          f"latest finite_count {collector.latest_finite_count}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
