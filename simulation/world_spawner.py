import importlib
import itertools
import os

import launch
import numpy as np
import rclpy
import rclpy.node
import rclpy.qos
import rclpy.wait_for_message
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from simulation.groundtruth.road import renderer, road


class WorldSpawner(rclpy.node.Node):
    """World spawner node."""

    def __init__(self):
        """Initialize the WorldSpawner node."""
        super().__init__("world_spawner")

        # Get the package path
        self.package_share_path = get_package_share_directory("simulation")

        # Load the parameters from the ROS parameter server and initialize
        # the publishers and subscribers
        self.load_ros_params()
        self.get_logger().info("WorldSpawner node initialized")

        self.world_name = "smartrollerz"  # TODO

        self.road = self.load_road(self.road_path)
        self.spawn_road_signs()

    def load_ros_params(self):
        """Gets the parameters from the ROS parameter server."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("road", "models/roads/default_road"),
            ],
        )

        self.road_path = os.path.join(
            self.package_share_path,
            self.get_parameter("road").value,
            "road.py",
        )

    def load_road(self, road_path: str) -> road.Road:
        """
        Load a road from a python file.

        Args:
            road_path: Path to the python file containing the road.
        """
        try:
            spec = importlib.util.spec_from_file_location("road", road_path)
            road_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(road_module)
        except ImportError as e:
            raise ImportError(f"Could not import road from {road_path}: {e}")

        try:
            road = getattr(road_module, "road")
        except AttributeError:
            raise AttributeError("The provided road does not contain a 'road' object.")

        return road

    def spawn_road_signs(self):
        # Get signs from the road sections and flatten the list with the sum() function
        signs = sum([section.traffic_signs for section in self.road.sections], [])

        # Group the signs by their id
        signs.sort(key=lambda sign: sign.kind.id_)
        signs_grouped = itertools.groupby(signs, key=lambda sign: sign.kind.id_)

        # Iterate through each group of signs and prepend a incrementing number to the 
        # name to avoid name collisions
        for _, sign_group in signs_grouped:
            for idx, sign in enumerate(sign_group):
                name = f"{sign.kind.mesh}_{idx}"
                xml = renderer.traffic_sign.generate_sdf_model(
                    mesh=sign.kind.mesh,
                    collision_box_position=sign.kind.collision_box_position,
                    collision_box_size=sign.kind.collision_box_size,
                )
                self.spawn_model(
                    model=xml,
                    name=name,
                    position=np.asarray([sign.center.x, sign.center.y, 0.0]),
                    orientation=np.asarray([0.0, 0.0, sign.orientation]),
                )

    def spawn_model(
        self,
        model: str,
        name: str,
        position: np.ndarray = None,
        orientation: np.ndarray = None,
    ):
        """
        Spawn a model in Gazebo.

        Args:
            model: Path to the URDF/SDF model file or xml string of the model.
            name: Name of the entity in Gazebo. Duplicates will not be spawned.
            position: A numpy array [x, y, z] representing the position.
            orientation: A numpy array [roll, pitch, yaw] representing the orientation.
        """

        self.get_logger().info(f"Spawning {name}")
        gz_spawn_model_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("ros_gz_sim"),
                            "launch",
                            "gz_spawn_model.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "world": self.world_name,
                "model_string": model,
                "entity_name": name,
                "x": str(position[0]),
                "y": str(position[1]),
                "z": str(position[2]),
                "R": str(orientation[0]),
                "P": str(orientation[1]),
                "Y": str(orientation[2]),
            }.items(),
        )

        ls = launch.LaunchService()
        ls.include_launch_description(LaunchDescription([gz_spawn_model_launch]))
        ls.run()


def main(args=None):
    """
    Main function to start the WorldSpawner.

    Keyword Arguments:
        args -- Launch arguments (default: {None})
    """
    rclpy.init(args=args)
    node = WorldSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        # Shutdown if not already done by the ROS2 launch system
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
