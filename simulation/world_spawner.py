import itertools
import math
import os
import time

import numpy as np
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from ros_gz_interfaces.msg import EntityFactory
from ros_gz_interfaces.srv import SpawnEntity

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from simulation.groundtruth.road import renderer, road
from simulation.groundtruth.road.render_cache import render_to_file_cached


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
        self._spawn_client = None
        self._queued_spawns = []
        if self.entity_spawn_mode == "service":
            self._spawn_client = self.create_client(
                SpawnEntity,
                f"/world/{self.world_name}/create",
            )

        road_load_started = time.perf_counter()
        self.road = road.load(self.road_path)
        self.get_logger().info(
            "PERF road_load_s="
            f"{time.perf_counter() - road_load_started:.3f}"
        )

        spawn_started = time.perf_counter()
        self.spawn_road()
        if self.entity_spawn_mode == "service":
            self.spawn_queued_models()
        self.spawn_road_signs()
        self.spawn_obstacles()
        if self.entity_spawn_mode == "service":
            self.spawn_queued_models()
        self.get_logger().info(
            "PERF all_road_entities_spawned_s="
            f"{time.perf_counter() - spawn_started:.3f}"
        )

    def load_ros_params(self):
        """Gets the parameters from the ROS parameter server."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("road", "models/roads/default_road"),
                ("road_render_mode", "cached"),
                ("entity_spawn_mode", "service"),
                ("spawn_concurrency", 4),
            ],
        )

        self.road_path = os.path.join(
            self.package_share_path,
            self.get_parameter("road").value,
            "road.py",
        )
        self.road_render_mode = self.get_parameter("road_render_mode").value
        if self.road_render_mode not in ("cached", "legacy"):
            raise ValueError(
                "road_render_mode must be either 'cached' or 'legacy', "
                f"got {self.road_render_mode!r}"
            )
        self.entity_spawn_mode = self.get_parameter("entity_spawn_mode").value
        if self.entity_spawn_mode not in ("service", "legacy"):
            raise ValueError(
                "entity_spawn_mode must be either 'service' or 'legacy', "
                f"got {self.entity_spawn_mode!r}"
            )
        self.spawn_concurrency = self.get_parameter("spawn_concurrency").value
        if self.spawn_concurrency < 1:
            raise ValueError(
                "spawn_concurrency must be at least 1, "
                f"got {self.spawn_concurrency!r}"
            )

    def spawn_road(self):
        """Spawns the road in the gazebo simulation."""
        dir = os.path.dirname(self.road_path)
        render_started = time.perf_counter()
        if self.road_render_mode == "legacy":
            size = self.road.render_to_file(dir)
            cache_hit = False
        else:
            size, cache_hit = render_to_file_cached(
                road=self.road,
                output_directory=dir,
                package_source=os.path.dirname(__file__),
                road_source=dir,
                seed=self.road._seed,
            )
        self.get_logger().info(
            f"PERF road_render_mode={self.road_render_mode} "
            f"road_render_cache_hit={str(cache_hit).lower()} "
            "road_render_s="
            f"{time.perf_counter() - render_started:.3f}"
        )
        image_path = os.path.join(
            dir.split("/simulation/")[2],
            f"{self.road._name}.png",
        )

        xml = f"""
            <sdf version="1.8">
            <model name="tile">
                <static>true</static>
                <link name="link">
                <visual name="visual">
                    <cast_shadows>false</cast_shadows>
                    <geometry>
                    <plane>
                        <normal>0 0 1</normal>
                        <size>{size.x} {size.y}</size>
                    </plane>
                    </geometry>
                    <material>
                    <ambient>0.8 0.8 0.8 1</ambient>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.2 0.2 0.2 1</specular>
                    <pbr>
                        <metal>
                            <albedo_map>package://{image_path}</albedo_map>
                            <roughness>0.8</roughness>
                        </metal>
                    </pbr>
                    </material>
                </visual>
                </link>
            </model>
            </sdf>
            """

        self.spawn_model(
            model=xml,
            name="road",
            position=[0.0, 0.0, 0.0],
            orientation=np.asarray([0.0, 0.0, 0.0]),
        )

    def spawn_road_signs(self):
        """Spawns the road signs in the gazebo simulation."""
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

    def spawn_obstacles(self):
        """Spawns the obstacles in the gazebo simulation."""
        # Get obstacles from the road sections and flatten the list with sum() function
        obstacles = sum([section.obstacles for section in self.road.sections], [])

        for idx, obstacle in enumerate(obstacles):
            name = f"obstacle_{idx}"
            xml = renderer.obstacle.generate_sdf_model(
                obstacle_size=np.array(
                    [obstacle.depth, obstacle.width, obstacle.height]
                ),
            )
            self.spawn_model(
                model=xml,
                name=name,
                position=np.asarray(
                    [obstacle.center.x, obstacle.center.y, obstacle.height / 2]
                ),
                orientation=np.asarray([0.0, 0.0, obstacle.orientation]),
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
        if self.entity_spawn_mode == "service":
            self._queued_spawns.append(
                (
                    name,
                    self.make_spawn_request(model, name, position, orientation),
                )
            )
            return

        self.get_logger().info(f"Spawning {name}")
        spawn_started = time.perf_counter()
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
        self.get_logger().info(
            f"PERF spawn_entity={name} "
            f"launch_service_s={time.perf_counter() - spawn_started:.3f}"
        )

    @staticmethod
    def make_spawn_request(
        model: str,
        name: str,
        position: np.ndarray,
        orientation: np.ndarray,
    ) -> SpawnEntity.Request:
        """Create a Gazebo spawn request from an SDF model and pose."""
        roll, pitch, yaw = orientation
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose.orientation.z = cr * cp * sy - sr * sp * cy
        pose.orientation.w = cr * cp * cy + sr * sp * sy

        factory = EntityFactory()
        factory.name = name
        factory.sdf = model
        factory.pose = pose

        request = SpawnEntity.Request()
        request.entity_factory = factory
        return request

    def spawn_queued_models(self):
        """Spawn queued models with bounded concurrency and check every result."""
        if not self._queued_spawns:
            return

        self.get_logger().info(
            "PERF spawn_strategy=service "
            f"spawn_concurrency={self.spawn_concurrency} "
            f"queued_entities={len(self._queued_spawns)}"
        )
        if not self._spawn_client.wait_for_service(timeout_sec=30.0):
            raise TimeoutError(
                "Timed out waiting for Gazebo spawn service "
                f"/world/{self.world_name}/create"
            )

        for batch_start in range(0, len(self._queued_spawns), self.spawn_concurrency):
            batch = self._queued_spawns[
                batch_start : batch_start + self.spawn_concurrency
            ]
            submitted_at = time.perf_counter()
            futures = [
                (name, self._spawn_client.call_async(request))
                for name, request in batch
            ]

            for name, future in futures:
                rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
                if not future.done():
                    raise TimeoutError(
                        f"Timed out spawning Gazebo entity {name!r}"
                    )

                response = future.result()
                if response is None or not response.success:
                    raise RuntimeError(
                        f"Gazebo failed to spawn entity {name!r}: {response!r}"
                    )

                self.get_logger().info(
                    f"PERF spawn_entity={name} "
                    "spawn_service_s="
                    f"{time.perf_counter() - submitted_at:.3f}"
                )

        self._queued_spawns.clear()


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
