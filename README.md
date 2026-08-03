[[Code](https://github.com/uos/rmagine_gazebo_plugins)] [[Wiki](https://github.com/uos/rmagine_gazebo_plugins/wiki)]

> [!IMPORTANT]
> Requires: Gazebo Harmonic (`gz-sim8`), ROS 2 Jazzy

# rmagine_gazebo_plugins

Range sensor plugins for Gazebo using the sensor simulation library [rmagine](https://github.com/uos/rmagine).
With rmagine's OptiX backend it is possible to simulate depth sensor data directly on your RTX graphics card. With the Embree backend you can simulate any provided sensor on your CPU.
Embree and OptiX are libraries for raytracing that build BVH acceleration structures over the scene for fast ray traversal. Once built, these structures let you simulate depth sensors on CPU or GPU without performance cliffs even in large Gazebo worlds -- and they're kept in sync with the live simulation incrementally (only what actually changed each tick is added/removed/moved), not rebuilt from scratch on every update.

## Architecture

Two plugin roles per backend, mirroring the classic split between a scene-sync world plugin and a raycasting sensor plugin (gz-sim only has one plugin base type, `System`, so both are `System` plugins, but the responsibilities stay separate):

- **Map system** (`rmagine_embree_map_system` / `rmagine_optix_map_system`, attached to `<world>`): builds and incrementally maintains one persistent Embree/OptiX scene from the world's `<visual>` geometry. Publishes the current map through an in-process registry keyed by `map_key` (default `"default"`).
- **Sensor system** (`rmagine_embree_sensor_system` / `rmagine_optix_sensor_system`, attached once per `<world>`, like the map system): auto-discovers every `<sensor type="custom" gz:type="rmagine_embree|rmagine_optix">` anywhere in the world via gz-sim's `components::CustomSensor` (the closest available analogue to Gazebo Classic's `GZ_REGISTER_STATIC_SENSOR` -- gz-sensors' own plugin-loading mechanism for custom sensor types was removed upstream). For each discovered sensor it looks up the map by `map_key`, raycasts against it (Spherical/Pinhole/O1Dn/OnDn models), and publishes `sensor_msgs/msg/LaserScan` (Spherical, single-ring only) and `sensor_msgs/msg/PointCloud2`, plus a TF broadcast -- all sensors of one backend share a single ROS node/TF broadcaster owned by the factory system.

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

`ignore_model`/`ignore_link` exclude a model (or a single link of one, `model::link`) from the raytracing scene entirely -- useful for excluding a sensor's own housing, or a robot the sensor is mounted on. `update/rate_limit` caps how often the scene is re-synced (Hz); `delta_trans`/`delta_rot` are the minimum pose change (meters/radians) before a moved entity's transform is pushed into the scene.

### 2. Sensor system (one plugin per world, one `<sensor>` per actual sensor)

```xml
<world name="default">
  <plugin name="rmagine_embree_sensor_system" filename="librmagine_embree_sensor_system.so">
    <!-- optional, default "rmagine_embree_sensor_system" -->
    <node_name>my_sensors_node</node_name>
  </plugin>
  ...

  <model name="my_robot">
    <link name="lidar_link">
      <sensor name="lidar" type="custom" gz:type="rmagine_embree">
        <map_key>default</map_key>
        <frame>lidar_link</frame>
        <parent_frame>world</parent_frame>
        <topic_scan>scan</topic_scan>
        <topic_points>points</topic_points>
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

`LaserScan` is only published for a single-ring (`<vertical>` omitted) Spherical model; a multi-ring 3D scan (or any other model type) publishes `PointCloud2` only. Extra output topics can be added with repeated `<output><topic>...</topic><type>scan|points</type></output>` elements.

### 3. Non-spherical sensor models

Set `<model_type>` to `pinhole`, `o1dn`, or `ondn` (default `spherical`):

**Pinhole** (depth camera-style):

```xml
<model_type>pinhole</model_type>
<pinhole_width>640</pinhole_width>
<pinhole_height>480</pinhole_height>
<pinhole_hfov>1.0472</pinhole_hfov>
```

**O1Dn** (one shared ray origin, arbitrary per-pixel directions) / **OnDn** (arbitrary per-pixel origins and directions): both read a `<rays_file>` YAML file with a shared schema:

```yaml
width: 8
height: 4
rays:
  - origin: [0, 0, 0]   # O1Dn: only rays[0].origin is used; OnDn: read per-ray
    dir: [1, 0, 0]
  - origin: [0, 0, 0]
    dir: [0.99, 0.01, 0]
  # ... width * height entries, row-major
```

```xml
<model_type>o1dn</model_type>
<rays_file>/path/to/rays.yaml</rays_file>
```

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

## Testing

`worlds/gz_embree_*.sdf` / `gz_optix_*.sdf` are minimal fixture worlds, each paired with a captured reference in `testdata/embree_harmonic/*_fixture.json` and wired into `colcon test` via `scripts/embree_fixture_harness.py` (`capture_embree_fixture`/`compare_embree_fixture`). Run with:

```console
colcon test --packages-select rmagine_gazebo_plugins
colcon test-result --all --verbose
```

Covers: static geometry (box/sphere/cylinder/plane/mesh/heightmap), a dynamically moving box (proving the incremental scene sync tracks pose changes correctly), Pinhole/O1Dn/OnDn models, `ignore_model`/`ignore_link`, mesh-by-URI caching, a same-tick spawn+delete regression (`zombie`), a multi-ring 3D scan (`vertical`), and OptiX/GPU parity for the baseline/dynamic/noise/multi-topic cases.
