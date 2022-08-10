# rmagine_gazebo_plugins



## Example

After compilation, run

```bash
user@pc:~$ roslaunch rmagine_gazebo_plugins example.launch
```

To execute sensor plugins on different Rmagine backends, for example optix, run

```bash
user@pc:~$ roslaunch rmagine_gazebo_plugins example.launch rmagine:=optix
```

## Usage

### 1. Compilation

Dependency: Rmagine - library. 

Compile with Embree or OptiX support to enable different rmagine_gazebo_plugins:

1. Embree
    - World-Plugins: `rmagine_embree_map_gzplugin`
    - Sensor-Plugins: `rmagine_embree_spherical`
2. OptiX
    - World-Plugins: `rmagine_optix_map_gzplugin`
    - Sensor-Plugins: `rmagine_optix_spherical`

### 2. Sensor Registration

The rmagine sensors are implemented as new gazebo sensors. They need to be registered first. To do that, you need to add `librmagine_embree_sensors_gzregister.so` or `librmagine_optix_sensors_gzregister.so` to the arguments of the gazebo execution call.



**Embree Example**

1. Command line Arguments:

```bash
user@pc:~$ gazebo -s librmagine_embree_sensors_gzregister.so
```

2. ROS launch file

```xml
<node name="gazebo" pkg="gazebo_ros" type="gzserver" 
    args="-s librmagine_embree_sensors_gzregister.so ...">
...
</node>
```

### 3. Integrating Map Plugins

Embree sensor plugins require one Embree map plugin running.
OptiX sensor plugins require one OptiX map plugin running.
In world-files the map plugins can be enabled as follows:

```xml
<world>
...

<!-- Embree Map Plugin -->
<plugin name='rmagine_embree_map' filename='librmagine_embree_map_gzplugin.so'>
</plugin>

<!-- Optix Map Plugin -->
<plugin name='rmagine_optix_map' filename='librmagine_optix_map_gzplugin.so'>
</plugin>

</world>
```

The map plugins construct a acceleration structure over the Gazebo scene.
As soon as the gazebo scene changes, the acceleration structure is updated accordingly.
Some other examples are located in the worlds folder.

To increase the performance sdf entities can be marked to be ignored by the map plugins.
For example, if you know that your 3D lidar never scans the robot it is attached to, you may consider ignoring the entire robot in the map plugins.

To achieve that in world-files just add an `rmagine_ignore` tag to the model:

```xml
<world>

<model name='plane1_model'>
    <rmagine_ignore/>
    ...
</model>

</world>
```

How to add ignores in urdf-files will be explained in the next section.

### 4. Integrating Sensors

**2D Laser**

```xml
<gazebo reference="laser2d">
    <sensor type="rmagine_embree_spherical" name="laser2d">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>60</update_rate>
      <visualize>false</visualize>
      <pre_alloc_mem>true</pre_alloc_mem>

      <ray>
        <scan>
          <horizontal>
            <min_angle>${-M_PI}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>360</samples>
          </horizontal>
        </scan>

        <range>
          <min>0.0</min>
          <max>10.0</max>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
        
      </ray>
    </sensor>
</gazebo>
```

**3D Laser**

```xml
<gazebo reference="laser3d">
    <sensor type="rmagine_embree_spherical" name="laser3d">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>60</update_rate>
      <visualize>false</visualize>
      <pre_alloc_mem>true</pre_alloc_mem>

      <ray>
        <scan>
          <horizontal>
            <min_angle>${-M_PI}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>360</samples>
          </horizontal>
          <vertical>
            <min_angle>${-60.0 * M_PI / 180.0}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>120</samples>
          </vertical>
        </scan>

        <range>
          <min>0.0</min>
          <max>80.0</max>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
    </sensor>
</gazebo>
```

As in world-files, ignores can be added to URDF files:

1. Ignore the entire robot:

```xml
<robot>
...

<gazebo>
    <rmagine_ignore/>
</gazebo>
</robot>
```

2. Ignore a link (TODO)

### 5. ROS Adapter

Supported `output` messages are:
- `sensor_msgs/LaserScan`
- `sensor_msgs/PointCloud`
- `sensor_msgs/PointCloud2`

Examples - this time using OptiX.

**2D Laser**

```xml
<gazebo reference="laser2d">
    <sensor type="rmagine_optix_spherical" name="laser2d">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>60</update_rate>
      <visualize>false</visualize>
      <pre_alloc_mem>true</pre_alloc_mem>

      <ray>
        <scan>
          <horizontal>
            <min_angle>${-M_PI}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>360</samples>
          </horizontal>
        </scan>

        <range>
          <min>0.0</min>
          <max>10.0</max>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
        
      </ray>

      <plugin name="rmagine_ros_laser2d" filename="librmagine_optix_ros_gzplugin.so">
          <frame>laser2d</frame>
          <outputs>
            <output name="scan">
              <msg>sensor_msgs/LaserScan</msg>
              <topic>laser2d/scan</topic>
            </output>

            <output name="pcl">
              <msg>sensor_msgs/PointCloud</msg>
              <topic>laser2d/pcl</topic>
            </output>
          </outputs>
      </plugin>
    </sensor>
</gazebo>
```

**3D Laser**

```xml
<gazebo reference="laser3d">
    <sensor type="rmagine_optix_spherical" name="laser3d">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>60</update_rate>
      <visualize>false</visualize>
      <pre_alloc_mem>true</pre_alloc_mem>

      <ray>
        <scan>
          <horizontal>
            <min_angle>${-M_PI}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>360</samples>
          </horizontal>
          <vertical>
            <min_angle>${-60.0 * M_PI / 180.0}</min_angle>
            <increment>${1.0 * M_PI / 180.0}</increment>
            <samples>120</samples>
          </vertical>
        </scan>

        <range>
          <min>0.0</min>
          <max>80.0</max>
        </range>

        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>

      <plugin name="rmagine_ros_laser3d" filename="librmagine_optix_ros_gzplugin.so">
          <frame>laser3d</frame>
          <outputs>
            <output name="pcl">
              <msg>sensor_msgs/PointCloud</msg>
              <topic>laser3d/pcl</topic>
            </output>

            <output name="pcl2">
              <msg>sensor_msgs/PointCloud2</msg>
              <topic>laser3d/pcl2</topic>
            </output>
          </outputs>
      </plugin>
    </sensor>
</gazebo>
```
