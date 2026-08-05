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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_share = get_package_share_directory("rmagine_gazebo_plugins")


    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            "rmagine",
            description="Rmagine backend used for simulation",
            default_value="embree",
            choices=["embree", "optix"],
        ),
        DeclareLaunchArgument(
            "enable_map_transform",
            description="Enable map transform for the robot",
            default_value="True",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            description="Start RViz2 with the robot demo configuration",
            default_value="True",
        ),
        DeclareLaunchArgument(
            "enable_gazebo_scan",
            description="Enable Gazebo scan plugin for the robot",
            default_value="True",
        ),
    ]

    rmagine_backend = LaunchConfiguration("rmagine")
    enable_map_transform = LaunchConfiguration("enable_map_transform")
    start_rviz = LaunchConfiguration("start_rviz")
    enable_gazebo_scan = LaunchConfiguration("enable_gazebo_scan")

    world_path = PathJoinSubstitution(
        [pkg_share, "worlds", PythonExpression(["'gz_' + '", rmagine_backend, "' + '_robot_demo.sdf'"])]
    )

    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([pkg_share, "urdf/example_robot.urdf.xacro"]),
                " ",
                PythonExpression(['" enable_gazebo_scan:=true" if ', enable_gazebo_scan, ' else " enable_gazebo_scan:=false"']),
                " ",
                PythonExpression(['" rmagine_backend:=" + "', rmagine_backend, '"']),
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
            "-name", "robot",
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
    
    gt_localization_node = Node(
        package="rmagine_gazebo_plugins",
        executable="gt_localization_node",
        name="gt_localization",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "gt_localization.yaml"])
        ],
        condition=IfCondition(enable_map_transform),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_share, "rviz", "robot_demo.rviz"]),
        ],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(launch_args + [gz_sim, robot_state_publisher, spawn_robot, bridge, gt_localization_node, rviz])
