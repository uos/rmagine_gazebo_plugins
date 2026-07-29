# Testing

This file documents the current manual validation flow for the Gazebo Harmonic (`gz-sim8`) Embree port.

## Scope

The steps below validate only the current gz-sim ported targets built with:

```console
colcon build --packages-select rmagine_gazebo_plugins --cmake-args -DRMAGINE_GZSIM_PORT=ON --merge-install --symlink-install
```

Validated components:

- `rmagine_embree_map_system`
- `rmagine_embree_sensor_system`
- ROS 2 `LaserScan` output on `scan`
- ROS 2 `PointCloud2` output on `points`

Not validated here:

- A robust TF integration for RViz in the `world` frame
- OptiX / GPU backend
- Gazebo Classic targets
- Real project integration beyond the synthetic baseline and dynamic validation worlds

Important boundary:

- Passing the checks in this file means the Harmonic Embree path works in the package's **toy validation worlds**.
- It does **not** by itself mean that `rmagine_gazebo_plugins` is fully migrated for the actual robot / vehicle / environment stack.

## Fixture Harness

The preferred evaluation path is now the fixture harness instead of ad-hoc `ros2 topic echo` dumps.

Committed fixtures:

- `testdata/embree_harmonic/baseline_fixture.json`
- `testdata/embree_harmonic/dynamic_fixture.json`

Installed entrypoints:

```console
ros2 run rmagine_gazebo_plugins capture_embree_fixture baseline
ros2 run rmagine_gazebo_plugins compare_embree_fixture baseline
ros2 run rmagine_gazebo_plugins capture_embree_fixture dynamic
ros2 run rmagine_gazebo_plugins compare_embree_fixture dynamic
```

Default capture outputs:

- `/tmp/rmagine_embree_baseline_capture.json`
- `/tmp/rmagine_embree_dynamic_capture.json`

Each capture result stores compact, reviewable metrics instead of full raw message dumps:

- `/scan` publish-rate estimate
- `/scan` finite-range summary
- center-beam time series
- center window sample
- `/points` finite / NaN counts
- `/points` bounding-box summary
- representative finite point sample

## Environment

From the workspace root:

```console
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Build

```console
colcon build --packages-select rmagine_gazebo_plugins --cmake-args -DRMAGINE_GZSIM_PORT=ON
```

Expected result:

- Build succeeds.
- Shared libraries are installed under `install/lib/rmagine_gazebo_plugins/`.

## Launch

### Baseline world

Run the baseline validation world first:

```console
gz sim -v 4 install/share/rmagine_gazebo_plugins/worlds/gz_embree_baseline.sdf
```

Expected result:

- Gazebo starts and remains alive.
- The GUI shows:
  - `target_box`
  - `sensor_model`
- The map system prints one initial rebuild.
- The sensor system prints a finite-range summary on each publish.

Preferred evaluation:

```console
ros2 run rmagine_gazebo_plugins capture_embree_fixture baseline
ros2 run rmagine_gazebo_plugins compare_embree_fixture baseline
```

### Dynamic world

Run the dynamic validation world after the baseline world behaves correctly:

```console
gz sim -v 4 install/share/rmagine_gazebo_plugins/worlds/gz_embree_dynamic.sdf
```

Expected result:

- Gazebo starts and remains alive.
- The GUI shows:
  - `target_box`
  - `sensor_model`
- `target_box` moves deterministically along x without GUI interaction.
- The map system reports pose-driven rebuilds.
- The sensor system reports simulator refreshes when map revisions change.

Compatibility note:

- `worlds/gz_embree_example.sdf` should behave identically to `worlds/gz_embree_dynamic.sdf`.

Preferred evaluation:

```console
ros2 run rmagine_gazebo_plugins capture_embree_fixture dynamic
ros2 run rmagine_gazebo_plugins compare_embree_fixture dynamic
```

## ROS 2 Topics

In a second terminal:

```console
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list | rg 'scan|points'
```

Expected result:

- `/scan`
- `/points`

Check rates:

```console
ros2 topic hz /scan
ros2 topic hz /points
```

Expected result:

- Both topics publish at about `5 Hz` with the current world configuration.

## Message Checks

### Baseline world

Inspect several scans:

```console
ros2 topic echo /scan --once
```

Expected result:

- `frame_id: sensor_link`
- `angle_min` near `-1.0472`
- `angle_max` near `1.0472`
- valid finite ranges in the center region
- center ranges should be close to the fixed box front face distance

Inspect one cloud:

```console
ros2 topic echo /points --once
```

Expected result:

- `frame_id: sensor_link`
- non-empty `data`
- finite points are present for the visible obstacle
- `height: 1`
- `width: 400` for the current example

### Dynamic world

Capture scans a few seconds apart:

```console
ros2 topic echo /scan --once
sleep 3
ros2 topic echo /scan --once
```

Expected result:

- The two scans are not identical.
- The sensor-system debug summary shows changing finite hit statistics as `target_box` moves.

Inspect one cloud:

```console
ros2 topic echo /points --once
```

Expected result:

- `frame_id: sensor_link`
- non-empty `data`
- finite points move consistently with the moving obstacle

## RViz2

Launch RViz2:

```console
rviz2
```

Add displays:

- `LaserScan` on `/scan`
- `PointCloud2` on `/points`

### Option A: Validate in sensor frame

- Set `Fixed Frame` to `sensor_link`.

Expected result:

- The scan / cloud are visible relative to the sensor frame.

### Option B: Validate in world frame

For the current validation worlds, start an external static TF publisher:

```console
ros2 run tf2_ros static_transform_publisher 0 0 1 0 0 0 world sensor_link
```

Then set RViz `Fixed Frame` to `world`.

Expected result:

- The scan / cloud are visible in the world frame.

Current known limitation:

- The plugin publishes sensor data correctly, but plugin-owned TF timing is not yet the recommended path for RViz validation.
- For the validation worlds, use `sensor_link` directly or an external static TF.

## Known Gaps

- RViz fixed frame support in `world` should not currently rely on plugin-owned TF timing.
- The current validation is manual only.
- The committed fixtures provide regression-oriented summary checks, not full raw message equivalence.
- Full parity against the original Gazebo Classic implementation is still in progress.
