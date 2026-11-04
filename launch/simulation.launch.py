import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """
    Generate the launch description.

    Returns:
        LaunchDescription -- The launch description.
    """
    python_executable = os.getenv("PYTHON_EXECUTABLE", "/usr/bin/python3")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_simulation = get_package_share_directory("simulation")

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz.",
            ),
            DeclareLaunchArgument(
                "keyboard_control",
                default_value="false",
                description="Activate keyboard control.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    get_package_share_directory("simulation"),
                    "config",
                    "ros_params.yaml",
                ),
                description="Path to the ROS parameters file.",
            ),
            DeclareLaunchArgument(
                "bridge_config_file",
                default_value=os.path.join(
                    get_package_share_directory("simulation"),
                    "config",
                    "ros_gz_bridge.yaml",
                ),
                description="Path to the Gazebo-ROS bridge configuration.",
            ),
            DeclareLaunchArgument(
                "road_render_mode",
                default_value="cached",
                description=(
                    "Road texture rendering strategy: cached or legacy "
                    "(always render)."
                ),
            ),
            DeclareLaunchArgument(
                "entity_spawn_mode",
                default_value="service",
                description=(
                    "Entity spawning strategy: service (batched persistent "
                    "client) or legacy (one launch process per entity)."
                ),
            ),
            DeclareLaunchArgument(
                "spawn_concurrency",
                default_value="4",
                description="Maximum number of concurrent Gazebo spawn requests.",
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                PathJoinSubstitution([pkg_simulation]),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py"),
                ),
                launch_arguments={
                    "gz_args": [
                        PathJoinSubstitution([pkg_simulation, "models", "world.sdf"]),
                        " --gui-config ",
                        PathJoinSubstitution(
                            [pkg_simulation, "config", "gazebo_gui.config"]
                        ),
                        " -r ",
                    ],
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/world/smartrollerz/create@ros_gz_interfaces/srv/SpawnEntity",
                ],
                parameters=[
                    {
                        "config_file": LaunchConfiguration("bridge_config_file"),
                        "qos_overrides./tf_static.publisher.durability": "transient_local",
                        "qos_overrides./tf.publisher.durability": "transient_local",
                    }
                ],
                output="screen",
            ),
            Node(
                package="simulation",
                executable="robot_description_publisher",
                name="robot_description_publisher",
                parameters=[
                    params_file,
                ],
            ),
            Node(
                package="simulation",
                executable="car_control_bridge",
                name="car_control_bridge",
                parameters=[
                    params_file,
                ],
            ),
            Node(
                package="simulation",
                executable="car_sensor_bridge",
                name="car_sensor_bridge",
                parameters=[
                    params_file,
                ],
            ),
            Node(
                package="simulation",
                executable="world_spawner",
                name="world_spawner",
                parameters=[
                    params_file,
                    {
                        "road_render_mode": LaunchConfiguration("road_render_mode"),
                        "entity_spawn_mode": LaunchConfiguration("entity_spawn_mode"),
                        "spawn_concurrency": LaunchConfiguration("spawn_concurrency"),
                    },
                ],
                prefix=[python_executable],
            ),
            Node(
                package="simulation",
                executable="keyboard_control",
                name="keyboard_control",
                parameters=[
                    params_file,
                ],
                condition=IfCondition(LaunchConfiguration("keyboard_control")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(pkg_simulation, "config", "smarty.rviz"),
                ],
                parameters=[
                    {"use_sim_time": True},
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
