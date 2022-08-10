# rmagine_gazebo_plugins

## Usage

### 1. Compilation

Requirements: Rmagine - library


### 2. Sensor Registration

Use command line Arguments of Gazebo to load additional shared libraries:

```bash
gazebo -s librmagine_embree_sensors_gzregister.so
```

or in launch file

```xml
<node name="gazebo" pkg="gazebo_ros" type="gzserver" 
    args="-s librmagine_embree_sensors_gzregister.so ...">
...
</node>
```

### 3. Integrating Map Plugins

See example world files in worlds folder

### 4. Integrating Sensors

TODO