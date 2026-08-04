[[Code](https://github.com/uos/rmagine_gazebo_plugins)] [[Wiki](https://github.com/uos/rmagine_gazebo_plugins/wiki)]

> [!IMPORTANT]
> Requires: Gazebo Harmonic (`gz-sim8`), ROS 2 Jazzy

# rmagine_gazebo_plugins

Range sensor plugins for Gazebo using the sensor simulation library [rmagine](https://github.com/uos/rmagine).
With rmagine's OptiX backend it is possible to simulate depth sensor data directly on your RTX graphics card. With the Embree backend you can simulate any provided sensor on your CPU.
Embree and OptiX are libraries for raytracing that build BVH acceleration structures over the scene for fast ray traversal. Once built, these structures let you simulate depth sensors on CPU or GPU without performance cliffs even in large Gazebo worlds -- and they're kept in sync with the live simulation incrementally (only what actually changed each tick is added/removed/moved), not rebuilt from scratch on every update.

<div align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=IOrBxiW0AmY
" target="_blank" >
  <img src="https://i.ytimg.com/vi/IOrBxiW0AmY/maxresdefault.jpg" 
  alt="Rmagine Gazebo Plugin YT Video" width="80%" style="max-width: 500px" height="auto" border="10" />
</a>
</div>

## Architecture

Two plugin roles per backend, mirroring the classic split between a scene-sync world plugin and a raycasting sensor plugin (gz-sim only has one plugin base type, `System`, so both are `System` plugins, but the responsibilities stay separate):

- **Map system** (`rmagine_embree_map_system` / `rmagine_optix_map_system`, attached to `<world>`): builds and incrementally maintains one persistent Embree/OptiX scene from the world's `<visual>` geometry. Publishes the current map through an in-process registry keyed by `map_key` (default `"default"`).
- **Sensor system** (`rmagine_embree_sensor_system` / `rmagine_optix_sensor_system`, attached once per `<world>`, like the map system): auto-discovers every `<sensor type="custom" gz:type="rmagine_embree|rmagine_optix">` anywhere in the world via gz-sim's `components::CustomSensor` (the closest available analogue to Gazebo Classic's `GZ_REGISTER_STATIC_SENSOR` -- gz-sensors' own plugin-loading mechanism for custom sensor types was removed upstream). For each discovered sensor it looks up the map by `map_key`, raycasts against it (Spherical/Pinhole/O1Dn/OnDn models), and publishes `gz.msgs.LaserScan` (Spherical, single-ring only) and `gz.msgs.PointCloudPacked` over plain **gz-transport** -- all sensors of one backend share a single `gz::transport::Node` owned by the factory system.

This plugin has **no ROS dependency at all** (map and sensor systems alike). If you want the data in ROS, bridge it with [`ros_gz_bridge`](https://github.com/gazebosim/ros_gz) -- see "Bridging to ROS" below. TF isn't published by this plugin either: attach gz-sim's own `gz::sim::systems::PosePublisher` to your robot and bridge its `gz.msgs.Pose_V` output to `tf2_msgs/msg/TFMessage`, exactly as shown in the mobile-robot example.

## Installation

#### rmagine

Follow rmagine's own build instructions, with the Embree and/or OptiX (CUDA) components enabled. Clone it into your workspace's `src` folder.

#### This package

```console
git clone git@github.com:uos/rmagine_gazebo_plugins.git
colcon build --packages-select rmagine_gazebo_plugins
```

Built targets depend on which rmagine components were found:

- `rmagine::embree` found: `rmagine_embree_map_system`, `rmagine_embree_sensor_system`
- `rmagine::optix` found: `rmagine_optix_map_system`, `rmagine_optix_sensor_system`

## Usage

### 1. Map system (one per world, per backend)

```xml
<world name="default">
  <plugin name="rmagine_embree_map_system" filename="librmagine_embree_map_system.so">
    <!-- optional -->
    <ignore_model>my_robot</ignore_model>
    <ignore_link>my_robot::lidar_link</ignore_link>
    <update>
      <rate_limit>200</rate_limit>
      <delta_trans>0.001</delta_trans>
      <delta_rot>0.001</delta_rot>
    </update>
    <debug>false</debug>
  </plugin>
  ...
</world>
```

`ignore_model`/`ignore_link` exclude a model (or a single link of one, `model::link`) from the raytracing scene entirely; useful for excluding a sensor's own housing, or a robot the sensor is mounted on. `update/rate_limit` caps how often the scene is re-synced (Hz); `delta_trans`/`delta_rot` are the minimum pose change (meters/radians) before a moved entity's transform is pushed into the scene.

### 2. Sensor system (one plugin per world, one `<sensor>` per actual sensor)

```xml
<world name="default">
  <plugin name="rmagine_embree_sensor_system" filename="librmagine_embree_sensor_system.so"/>
  ...

  <model name="my_robot">
    <link name="lidar_link">
      <sensor name="lidar" type="custom" gz:type="rmagine_embree">
        <map_key>default</map_key>
        <frame>lidar_link</frame>
        <topic_scan>/model/my_robot/scan</topic_scan>
        <topic_points>/model/my_robot/points</topic_points>
        <update_rate>20</update_rate>
        <range_min>0.2</range_min>
        <range_max>100.0</range_max>

        <model_type>spherical</model_type>
        <scan>
          <horizontal>
            <min_angle>-3.14159</min_angle>
            <increment>0.01745</increment>
            <samples>360</samples>
          </horizontal>
          <!-- optional: omit for a single-ring 2D scan -->
          <vertical>
            <min_angle>-0.2618</min_angle>
            <increment>0.008727</increment>
            <samples>60</samples>
          </vertical>
        </scan>
      </sensor>
    </link>
  </model>
</world>
```

`type="custom"` is required (sdformat validates the standard `type` attribute against its own known sensor type names); `gz:type` is the free-form identifier this package's factory plugin looks for (`rmagine_embree` or `rmagine_optix`) -- this is gz-sim's own documented convention for third-party sensor types, the same one used by its shipped `environmental_sensor.sdf` example.

`<topic_scan>`/`<topic_points>` are plain gz-transport topic names (this plugin does no automatic `/model/<name>/...` namespacing the way gz-sim's built-in sensors do) -- write the full path you want if you're bridging into a namespaced robot, or a bare name like `scan` if not.

`LaserScan` is only published for a single-ring (`<vertical>` omitted) Spherical model; a multi-ring 3D scan (or any other model type) publishes `PointCloudPacked` only. Extra output topics can be added with repeated `<output><topic>...</topic><type>scan|points</type></output>` elements.

### 2b. Bridging to ROS

Use [`ros_gz_bridge`](https://github.com/gazebosim/ros_gz)'s `parameter_bridge`, e.g. with a YAML config (see `config/ros_gz_bridge_robot_demo.yaml` for a complete example):

```yaml
- ros_topic_name: "/scan"
  gz_topic_name: "/model/my_robot/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/points"
  gz_topic_name: "/model/my_robot/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS
```

```console
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/path/to/bridge.yaml
```

### 2c. Migrating from `gpu_lidar`

Switching an existing robot from gz-sim's built-in `gpu_lidar` sensor to rmagine is a same-shape edit to the `<sensor>` block, not a rewrite. The only thing that never needs to change is your `ros_gz_bridge` config: keep `<topic_points>` set to `<topic_scan>/points`, matching `gpu_lidar`'s own convention of auto-publishing `PointCloudPacked` on `<topic>/points`, and the same bridge entries that worked for `gpu_lidar` keep working unchanged.

**Before** (`gpu_lidar`):

```xml
<sensor name="lidar" type="gpu_lidar">
  <topic>scan</topic>
  <update_rate>10</update_rate>
  <frame_id>lidar_link</frame_id>
  <lidar>
    <scan>
      <horizontal><samples>360</samples><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
      <vertical><samples>16</samples><min_angle>-0.261799</min_angle><max_angle>0.261799</max_angle></vertical>
    </scan>
    <range><min>0.2</min><max>30.0</max></range>
  </lidar>
</sensor>
```

**After** (rmagine):

```xml
<sensor name="lidar" type="custom" gz:type="rmagine_embree">
  <topic_scan>scan</topic_scan>
  <topic_points>scan/points</topic_points>
  <update_rate>10</update_rate>
  <frame>lidar_link</frame>
  <range_min>0.2</range_min>
  <range_max>30.0</range_max>
  <scan>
    <horizontal><min_angle>-3.14159</min_angle><increment>0.0175019</increment><samples>360</samples></horizontal>
    <vertical><min_angle>-0.261799</min_angle><increment>0.0349065</increment><samples>16</samples></vertical>
  </scan>
</sensor>
```

Field-by-field diff:

| `gpu_lidar` | rmagine | Notes |
|---|---|---|
| `type="gpu_lidar"` | `type="custom" gz:type="rmagine_embree"` (or `rmagine_optix` for the GPU/OptiX backend) | required |
| `<topic>` | `<topic_scan>` + `<topic_points>` | set `<topic_points>` to `<topic_scan>/points` to match `gpu_lidar`'s own auto-suffix and keep any existing bridge config unchanged |
| `<frame_id>` | `<frame>` | |
| `<lidar><range><min>/<max></lidar>` | `<range_min>`/`<range_max>` | now top-level |
| `<lidar><scan><horizontal>/<vertical>` | `<scan><horizontal>/<vertical>` | now top-level (not nested under `<lidar>`/`<ray>`) |
| `...<min_angle>`/`<max_angle>` | `...<min_angle>`/`<increment>` | rmagine reads the angular step directly rather than deriving it from `max_angle`; `increment = (max_angle - min_angle) / (samples - 1)` |
| `<update_rate>` | `<update_rate>` | unchanged |
| `<always_on>`, `<visualize>`, `<enable_metrics>` | (accepted, ignored) | harmless if left in place |

Everything else (`<map_key>`, `<model_type>`, `<debug>`, Pinhole/O1Dn/OnDn-specific tags, `<noise>`) is a rmagine-only addition with no `gpu_lidar` equivalent, and all of it is optional.

### 3. Non-spherical sensor models

Set `<model_type>` to `pinhole`, `o1dn`, or `ondn` (default `spherical`):

**Pinhole** (depth camera-style):

```xml
<model_type>pinhole</model_type>
<pinhole_width>640</pinhole_width>
<pinhole_height>480</pinhole_height>
<pinhole_hfov>1.0472</pinhole_hfov>
```

**O1Dn** (one shared ray origin, arbitrary ray directions) 

read a `<rays_file>` YAML file with a shared schema:

```yaml
width: 8
height: 4
rays:
  orig: [0, 0, 0]
  dirs:
    - [1, 0, 0]
    - [0.99, 0.01, 0]
  # ... width * height entries, row-major
```

```xml
<model_type>o1dn</model_type>
<rays_file>/path/to/rays.yaml</rays_file>
```

**OnDn** (arbitrary ray origins and directions)

read a `<rays_file>` YAML file with a shared schema:

```yaml
width: 8
height: 4
ray:
  origs:
    - [0, 0, 0]
    - [0, 0, 0]
  dirs: 
    - [1, 0, 0]
    - [0.99, 0.01, 0]
  # ... width * height entries, row-major
```

```xml
<model_type>o1dn</model_type>
<rays_file>/path/to/rays.yaml</rays_file>

### 4. Noise (OptiX/GPU sensor system only)

Repeatable `<noise>` elements, applied in order to the simulated ranges (in VRAM, before download):

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
<noise>
  <type>rel_gaussian</type>
  <stddev>0.002</stddev>
  <range_exp>1.0</range_exp>
</noise>
<noise>
  <type>uniform_dust</type>
  <hit_prob>0.0000001</hit_prob>
  <return_prob>0.5</return_prob>
</noise>
```

## Example: mobile robot

`launch/robot_demo.launch.py` spawns a small differential-drive robot
(`urdf/example_robot.urdf.xacro`) carrying one rmagine Embree spherical
lidar into `worlds/gz_embree_robot_demo.sdf` (a few static boxes/a cylinder
to drive around and scan), and bridges everything to ROS via
`config/ros_gz_bridge_robot_demo.yaml`:

```console
ros2 launch rmagine_gazebo_plugins robot_demo.launch.py
```

Then, in another terminal:

```console
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share rmagine_gazebo_plugins)/rviz/robot_demo.rviz
```

Drive it with any `geometry_msgs/msg/Twist` publisher on `/cmd_vel` (e.g.
`ros2 run teleop_twist_keyboard teleop_twist_keyboard`). `/scan`/`/points`
are the bridged lidar output; `/odom` and `/tf` come from gz-sim's own
`gz::sim::systems::DiffDrive` and `gz::sim::systems::PosePublisher` systems
(see the xacro's `<gazebo>` blocks) -- this package's own plugin publishes
neither, by design (see "Architecture" above).

## Testing

`worlds/gz_embree_*.sdf` / `gz_optix_*.sdf` are minimal fixture worlds, each paired with a captured reference in `testdata/embree_harmonic/*_fixture.json` and wired into `colcon test` via `scripts/embree_fixture_harness.py` (`capture_embree_fixture`/`compare_embree_fixture`), which also launches a `ros_gz_bridge parameter_bridge` (`testdata/embree_harmonic/ros_gz_bridge_fixtures.yaml`) alongside each world so the harness can keep subscribing to `/scan`/`/points` via `rclpy` exactly as before. Run with:

```console
colcon test --packages-select rmagine_gazebo_plugins
colcon test-result --all --verbose
```

Covers: static geometry (box/sphere/cylinder/plane/mesh/heightmap), a dynamically moving box (proving the incremental scene sync tracks pose changes correctly), Pinhole/O1Dn/OnDn models, `ignore_model`/`ignore_link`, mesh-by-URI caching, a same-tick spawn+delete regression (`zombie`), a multi-ring 3D scan (`vertical`), and OptiX/GPU parity for the baseline/dynamic/noise/multi-topic cases.
