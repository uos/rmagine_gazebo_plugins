"""Spawns a small differential-drive robot equipped with an rmagine Embree
spherical lidar into gz_embree_robot_demo.sdf, and bridges its gz-native
topics (see rmagine_embree_sensor_system.hpp -- the plugin is gz-transport
only, no ROS dependency at all) to ROS via ros_gz_bridge.

Structurally mirrors amock/rmcl_examples's rmcl_examples_sim/launch/
start_robot_launch.py: robot_description via xacro, robot_state_publisher,
gz-sim launched through ros_gz_sim's own launch include, `ros_gz_sim create`
to spawn from the robot_description topic, and a single `parameter_bridge`
node driven by a YAML config file (config/ros_gz_bridge_robot_demo.yaml)
instead of one bridge argument per topic.
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def generate_launch_description():
    pkg_share = get_package_share_directory("rmagine_gazebo_plugins")

    world_path = PathJoinSubstitution(
        [pkg_share, "worlds", "gz_embree_robot_demo.sdf"]
    )

    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([pkg_share, "urdf", "example_robot.urdf.xacro"]),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "publish_frequency": 50.0,
                "robot_description": robot_description,
            }
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r ", world_path]}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            # Must match <ignore_model> in gz_embree_robot_demo.sdf and the
            # /model/rmagine_example_robot/... topic names in
            # example_robot.urdf.xacro/ros_gz_bridge_robot_demo.yaml --
            # `create -name` sets the spawned entity's name regardless of
            # the URDF's own <robot name="...">.
            "-name", "rmagine_example_robot",
            "-z", "0.2",
        ],
        parameters=[{"use_sim_time": True}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [pkg_share, "config", "ros_gz_bridge_robot_demo.yaml"]
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription([gz_sim, robot_state_publisher, spawn_robot, bridge])
