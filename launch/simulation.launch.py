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
                "params_file",
                default_value=os.path.join(
                    get_package_share_directory("simulation"),
                    "config",
                    "ros_params.yaml",
                ),
                description="Path to the ROS parameters file.",
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
                    "gz_args": PathJoinSubstitution(
                        [pkg_simulation, "models", "world.sdf"]
                    ),
                    "on_exit_shutdown": "True",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": os.path.join(
                            pkg_simulation, "config", "ros_gz_bridge.yaml"
                        ),
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
