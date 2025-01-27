import itertools
import os
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, Tuple

import cairo

from simulation.groundtruth.geometry import Polygon, Transform, Vector
from simulation.groundtruth.road.renderer import surface_markings, utils
from simulation.groundtruth.road.road import Road
from simulation.groundtruth.road.sections.road_section import RoadSection


@dataclass
class Tile:
    """Piece of the groundplane with lines used to display road lines on the ground.

    The groundplane in simulation is made out of many rectangular tiles. Each tile displays
    an image on the ground.
    """

    COUNTER = 0
    """Static variable used to generate unique model names in Gazebo.

    To ensure that Gazebo doesn't get confused with multiple tiles that have
    the same name, e.g. when reloading, the counter is increased every time a model
    name is generated.
    """

    index: Tuple[int, int]
    """Position within the lattice of tiles on the groundplane."""
    size: Vector
    """Size of the tile."""
    resolution: Vector
    """Resolution of the tile's image."""
    road_folder_name: str = field(default=None, repr=False)
    """Name of the folder in which all tiles of the current road are.

    (Not the complete path, just the name of the folder!)
    """
    sections: Dict[int, RoadSection] = field(default_factory=set)
    """All sections that are (atleast partly) on this tile."""

    """Automatically generated when rendering."""
    already_rendered: bool = False
    """Indicate whether the tile has been rendered before."""

    @property
    def name(self) -> str:
        """str: Name of the tile's model when spawned in Gazebo."""
        return f"tile_{self.index[0]}x{self.index[1]}"

    @property
    def transform(self) -> Transform:
        """Transform: Transform to the center of the tile."""
        return Transform(
            [(self.index[0]) * self.size.x, (self.index[1]) * self.size.y], 0
        )

    @property
    def frame(self) -> Polygon:
        """Polygon: Frame of the tile."""
        return (
            self.transform
            * Transform([-self.size.x / 2, -self.size.y / 2], 0)
            * Polygon([[0, 0], [self.size.x, 0], self.size, [0, self.size.y]])
        )

    def get_model_string(self) -> str:
        """Get a model string that can be spawned in Gazebo."""
        Tile.COUNTER += 1
        return f"""
          <sdf version="1.8">
          <model name="tile">
            <static>true</static>
            <link name="link">
              <visual name="visual">
                <cast_shadows>false</cast_shadows>
                <geometry>
                  <plane>
                    <normal>0 0 1</normal>
                    <size>{self.size.x} {self.size.y}</size>
                  </plane>
                </geometry>
                <material>
                  <ambient>0.8 0.8 0.8 1</ambient>
                  <diffuse>0.8 0.8 0.8 1</diffuse>
                  <specular>0.2 0.2 0.2 1</specular>
                  <pbr>
                    <metal>
                      <albedo_map>model://models/roads/default_road/tiles/{self.name}.png</albedo_map>
                      <roughness>0.8</roughness>
                    </metal>
                  </pbr>
                </material>
              </visual>
            </link>
          </model>
          </sdf>
          """
        return f"""
          <sdf version="1.8">
          <model name="tile">
            <static>true</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <plane>
                    <normal>0 0 1</normal>
                    <size>{self.size.x} {self.size.y}</size>
                  </plane>
                </geometry>
                <surface>
                  <friction>
                    <ode>
                      <mu>50</mu>
                    </ode>
                    <bullet>
                      <friction>1</friction>
                      <rolling_friction>0.1</rolling_friction>
                    </bullet>
                  </friction>
                </surface>
              </collision>
              <visual name="visual">
                <geometry>
                  <plane>
                    <normal>0 0 1</normal>
                    <size>{self.size.x} {self.size.y}</size>
                  </plane>
                </geometry>
                <material>
                  <ambient>0.8 0.8 0.8 1</ambient>
                  <diffuse>0.8 0.8 0.8 1</diffuse>
                  <specular>0.2 0.2 0.2 1</specular>
                  <pbr>
                    <metal>
                      <albedo_map>model://models/roads/default_road/tiles/{self.name}.png</albedo_map>
                      <roughness>0.8</roughness>
                    </metal>
                  </pbr>
                </material>
              </visual>
            </link>
          </model>
          </sdf>
          """

    def render_to_file(self, roads_path: str):
        """Render an image of the tile and save it to a file.

        Args:
            roads_path: Directory in which all roads are located.
        """
        surface = cairo.ImageSurface(
            cairo.FORMAT_RGB24, int(self.resolution.x), int(self.resolution.y)
        )
        ctx = cairo.Context(surface)

        # Adjust scale
        ctx.scale(self.resolution.x / self.size.x, self.resolution.y / self.size.y)
        # Inverse y-axis
        ctx.translate(0, self.size.y / 2)
        ctx.scale(1, -1)
        ctx.translate(0, -self.size.y / 2)

        # Move to center of the tile
        ctx.translate(self.size.x / 2, self.size.y / 2)

        # Create black background
        ctx.set_source_rgb(0, 0, 0)
        ctx.rectangle(0, 0, self.size.x, self.size.y)
        ctx.fill()

        # Invert the render transform
        ctx.translate(-self.transform.translation.x, -self.transform.translation.y)

        # Draw lines for all sections
        for sec in self.sections.values():
            for line in sec.lines:
                utils.draw_line(ctx, line)
            for marking in sec.surface_markings:
                surface_markings.draw(ctx, marking)

        dir = os.path.join(roads_path)
        os.makedirs(dir, exist_ok=True)

        surface.write_to_png(os.path.join(dir, f"{self.name}.png"))

    @staticmethod
    def load_tiles_from_folder(
        folder: str,
        tile_size: Vector,
        tile_resolution: Vector,
    ) -> list["Tile"]:
        """Load all tiles from a folder.

        Args:
            folder: Folder in which the tiles are located.
        """
        tiles = []
        for file in os.listdir(folder):
            if file.endswith(".png"):
                name = file.split(".")[0]
                x, y = name.split("_")[1].split("x")
                tiles.append(
                    Tile(
                        index=(int(x), int(y)),
                        size=tile_size,
                        resolution=tile_resolution,
                        road_folder_name=folder,
                        already_rendered=True,
                    )
                )
        return tiles

    @staticmethod
    def create_new_tiles(
        road: Road,
        tile_size: Vector,
        tile_resolution: Vector,
    ) -> list["Tile"]:
        sections = road.sections

        active_tiles: defaultdict[tuple[int, int], set[int]] = defaultdict(set)
        


        for sec in sections:
            box = sec.get_bounding_box()
            minx, miny, maxx, maxy = box.bounds

            tiles_x = range(
                int(minx / tile_size.x) - 2,
                int(maxx / tile_size.x) + 2,
            )
            tiles_y = range(
                int(miny / tile_size.y) - 2,
                int(maxy / tile_size.y) + 2,
            )

            tile_keys = itertools.product(tiles_x, tiles_y)

            for key in tile_keys:
                active_tiles[key].add(sec.id)

        tiles = [
            Tile(
                key,
                sections={sec_id: sections[sec_id] for sec_id in secs},
                size=tile_size,
                resolution=tile_resolution,
                road_folder_name=f".{road._name}",
            )
            for key, secs in active_tiles.items()
        ]
        return tiles
