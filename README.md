[[Code](https://github.com/uos/rmagine_gazebo_plugins)] [[Wiki](https://github.com/uos/rmagine_gazebo_plugins/wiki)]

# rmagine_gazebo_plugins (WIP)

Range sensor plugins for Gazebo using the sensor simulation library [rmagine](https://github.com/uos/rmagine). 
With rmagine's OptiX backend it is possible to simulate depth sensor data directly on your RTX graphics card. With Embree backend you can simulate any provided sensor online on your CPU.
Embree and OptiX are libraries for raytracing and build BVH acceleration structures on the scene for faster ray traversals.
After building these acceleration structures, you can simulate depth sensors on CPU or GPU without getting perfomance issues even in large Gazebo worlds.

Youtube-Video:

<div align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=IOrBxiW0AmY
" target="_blank" >
  <img src="https://i.ytimg.com/vi/IOrBxiW0AmY/maxresdefault.jpg" 
  alt="Rmagine Gazebo Plugin YT Video" width="80%" style="max-width: 500px" height="auto" border="10" />
</a>
</div>



## Examples

After compiling 

### `example.launch`

```bash
user@pc:~$ roslaunch rmagine_gazebo_plugins example.launch
```

Simulates a 3d lidar at 20hz on Embree backend.  
To use OptiX backend, run

```bash
user@pc:~$ roslaunch rmagine_gazebo_plugins example.launch rmagine:=optix
```

Open RViz set fixed frame to `base_footprint` and visualize topic `laser3d/pcl`.

### `rotating_scanner.launch`

```bash
user@pc:~$ roslaunch rmagine_gazebo_plugins rotating_laser.launch
```

or with OptiX backend

```bash
user@pc:~$ roslaunch rmagine_gazebo_plugins rotating_laser.launch rmagine:=optix
```

Open RViz set fixed frame to `base_footprint` and visualize topic `laser2d/scan`.
In Gazebo-GUI find the `laser2d` link at model `robot_sensor`.
To let the scanner rotate go to Gazebo-GUI:
1. Right-click on the `laser2d` link at model `robot_sensor`
2. Click "Apply Force/Torque"
3. Set Torque to y=0.5 
4. Click "Apply Torque"

Now the scanner cylinder should rotate in Gazebo as well as in RViz.


## Usage

### 1. Installation


#### Rmagine

Follow instructions of Rmagine library installation. Compile with Embree or OptiX backends for CPU or GPU support respectively.

#### Compilation
Clone this repository to your ROS-workspace (src folder).

```bash
user@pc:~/catkin_ws/src$ git clone [this-repo-link]
```

Then compile with

```bash
user@pc:~/catkin_ws$ catkin_make
```

Depending on which backends were installed during Rmagine installation the following plugins are built:

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

### 3. Map Plugins

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
For example, if you know that your 3D lidar never scans the robot it is attached to, you may consider excluding the entire robot of the map plugins.

To achieve that in world-files just add an `rmagine_ignore` tag to the model:

```xml
<world>

<!-- Exclude single model from map -->
<model name='plane1_model'>
    <rmagine_ignore/>
    ...
</model>

<!-- Or exclude single link from map -->
<model name="plane2_model">
  ...
  <link name="plane2_link">
    ...
    <rmagine_ignore/>
  </link>

</model>

</world>
```

How to add ignores in urdf-files will be explained in the next section.

### 4. Sensors

**2D Laser**

```xml
<gazebo reference="laser2d">
    <sensor type="rmagine_embree_spherical" name="laser2d">
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <update_rate>60</update_rate>

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

```xml
<robot>
...

<!-- Ignore the entire robote-->
<gazebo>
    <rmagine_ignore/>
</gazebo>

<!-- Ignore a link. Useful if you want to ignore the scanner visual -->
<gazebo reference="my_scanner_link">
    <rmagine_ignore/>
</gazebo>
</robot>
```

### 5. Noise

Currently noise models are implemented as preprocessing steps directly on the simulated ranges data. Any of the following noise models can be chained to generate complex combined noise models.

1. Gaussian Noise

Apply gaussian noise $N(\mu, \sigma)$ to simulated ranges.

| Parameter |  Description  |
|:---------:|:-------------:|
| `mean` | Mean $\mu$ of normal distributed noise |
| `stddev` | standard deviation $\sigma$ of normal distributed noise |

Example:

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```

2. Relative Gaussian Noise

Apply gaussian noise $N(\mu, \sigma_r)$ to simulated ranges. Here, the standard deviation varies depending on distance.


| Parameter |  Description  |
|:---------:|:-------------:|
| `mean` | Mean $\mu$ of normal distributed noise |
| `stddev` | standard deviation $\sigma$ of normal distributed noise |
| `range_exp` | range exponent $c$ to compute range based stddev: $ \sigma_r = \sigma \cdot r^{c} $ |


Example:

```xml
<noise>
  <type>rel_gaussian</type>
  <mean>0.0</mean>
  <stddev>0.002</stddev>
  <range_exp>1.0</range_exp>
</noise>
```

3. Uniform Dust Noise

Apply uniform dust noise to simulated ranges. Assuming some small particles could be hit by the range sensor that are not modeled by the scene, use this noise type. 

Parameters:

| Parameter |  Description  |
|:---------:|:-------------:|
| `hit_prob` | Probability of a ray hitting a particle in one meter free space. |
| `return_prob` | Probability of a ray hitting dust returns to sender depending on particle distance |

Example:

```xml
<noise>
  <type>uniform_dust</type>
  <hit_prob>0.0000001</hit_prob>
  <return_prob>0.5</return_prob>
</noise> 
```


**Noise Chaining**

Example of using the gaussian model first and the uniform dust model second:

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.002</stddev>
</noise>

<noise>
  <type>uniform_dust</type>
  <hit_prob>0.0000001</hit_prob>
  <return_prob>0.5</return_prob>
</noise> 
```


## ROS Plugin

This plugin generates ROS-messages of the simulated data and writes them to specified ROS-topics.
The following ROS-Adapter are available dependend on your sensor type:


`librmagine_optix_ros_gzplugin.so`
- sensor types: `rmagine_optix_spherical`

`librmagine_embree_ros_gzplugin.so`
- sensor types: `rmagine_embree_spherical`



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

## Work in Progress

This is a pre-release. There is still some work to do for the first stable release:

- Implemented: SphericalModel. TODO: PinholeModel, O1DnModel, OnDnModel
- Tests: More tests on different devices. Let me know, if you had problems integrating the rmagine_gazebo_plugins into your project.

Nice-to-Have:
- Add segmenting functionallity: Store labeled sensor data from a list of poses in a commonly used file format

Known Issues:

- Sometimes the Gazebo simulation needs to be started twice in order to get everything started (blocking threads?)
- "Core dumped" on exit:
```bash
[Dbg] [rmagine_embree_map_gzplugin.cpp:52] [RmagineEmbreeMap] Destroyed.
terminate called after throwing an instance of 'boost::wrapexcept<boost::lock_error>'
terminate called recursively
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
Aborted (core dumped)
```


### Known Bug-Fixes:

On my system, Gazebo finds all rmagine libraries automatically. If that is not the case for you, try appending your ROS workspace `your_ws` to the Gazebo search pathes:

```console
export GAZEBO_PLUGIN_PATH=~/your_ws/devel/lib:$GAZEBO_PLUGIN_PATH
```
