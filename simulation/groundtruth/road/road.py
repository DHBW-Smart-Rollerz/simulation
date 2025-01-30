"""Road class used to define roads as a python class.

A simulated road can be defined through an object of type Road. It contains all sections of
that road as a list.
"""

import importlib
import os
import random
from dataclasses import dataclass, field
from typing import List, Tuple

import cairo

from simulation.groundtruth.geometry import Pose, Transform
from simulation.groundtruth.geometry.vector import Vector
from simulation.groundtruth.road.renderer import surface_markings, utils
from simulation.groundtruth.road.sections.road_section import RoadSection


@dataclass
class Road:
    """Container object for roads.

    A road consists of multiple road sections that are concatenated. The sections attribute
    contains these sections in the correct order.
    """

    _name: str = field(default=None, init=False)
    """Name of the road.

    The name attribute is determined by the name of the file.
    It is filled in when the road is generated.
    """

    _file_path: str = field(default=None, init=False)
    """The file path of the road definition.    """

    _seed: str = field(default=None, init=False)
    """Seed used when generating the road.

    Determined when generating the road.
    """

    use_seed: bool = True
    """Use a default seed if none is provided.

    By default the `use_seed` attribute is true and a seed is set before creating \
    the road. If `use_seed` is set to False, the seed is invalidated and there will \
    be different random values every time the road is created.
    """

    sections: List[RoadSection] = field(default_factory=list)
    """All sections of the road."""

    length: float = 0
    """Length of road."""

    def __post_init__(self):
        """Set random seed if specified."""
        if not self.use_seed:
            # Overwrite seed
            random.seed()

    def append(self, section: RoadSection):
        """Append a road section. Determine id of the section.

        Args:
            section: New road section.
        """
        section.id = len(self.sections)
        if section.id == 0:
            section._is_start = True
        if section.id > 0:
            # Pass ending of last section as the transformation to next section
            ending: Tuple[Pose, float] = self.sections[-1].get_ending()
            section.set_transform(Transform(ending[0], ending[0].orientation))
            section.prev_length = self.length
        self.length = self.length + section.middle_line.length
        self.sections.append(section)

    def close_loop(self, p_curvature: float = 2):
        """Append a road section that connects the last section to the beginning.

        The road's beginning and it's end are connected using a cubic bezier curve.

        Args:
            p_curvature: Scale the curvature of the resulting loop.
        """

        # Global position of the end of the road
        end_pose_global, _ = self.sections[-1].get_ending()

        # Distance to start / p_curvature
        approximate_radius = abs(end_pose_global.position) / p_curvature

        section = RoadSection.fit_ending(
            end_pose_global, Pose([0, 0], 0), approximate_radius
        )
        self.append(section)

    def render_to_file(self, path: str, padding: float = 2.0) -> Vector:
        """Render an image of the road and save it to a file.

        Args:
            path: Path to the folder where the road should be rendered.
        """
        min_x, min_y, max_x, max_y = 0.0, 0.0, 0.0, 0.0
        for sec in self.sections:
            minx, miny, maxx, maxy = sec.get_bounding_box().bounds
            min_x, min_y = min(min_x, minx), min(min_y, miny)
            max_x, max_y = max(max_x, maxx), max(max_y, maxy)

        dim_x = max(abs(min_x), abs(max_x))
        dim_y = max(abs(min_y), abs(max_y))

        print(f"Road: {dim_x:5.2f}/{dim_y:5.2f}")
        print(f"Road: {min_x:5.2f}/{min_y:5.2f}  {max_x:5.2f}/{max_y:5.2f}")

        size = Vector((dim_x + padding) * 2, (dim_y + padding) * 2)
        resolution_per_meter = Vector(512, 512)
        resolution = Vector(
            resolution_per_meter.x * size.x,
            resolution_per_meter.y * size.y,
        )

        surface = cairo.ImageSurface(
            cairo.FORMAT_RGB24, int(resolution.x), int(resolution.y)
        )
        ctx = cairo.Context(surface)

        # Adjust scale
        ctx.scale(resolution.x / size.x, resolution.y / size.y)
        # Inverse y-axis
        ctx.translate(0, size.y / 2)
        ctx.scale(1, -1)
        ctx.translate(0, -size.y / 2)

        # Move to center of the tile
        ctx.translate(size.x / 2, size.y / 2)

        # Create black background
        ctx.set_source_rgb(0, 0, 0)
        ctx.rectangle(0, 0, size.x, size.y)
        ctx.fill()

        # Draw lines for all sections
        for sec in self.sections:
            for line in sec.lines:
                utils.draw_line(ctx, line)
            for marking in sec.surface_markings:
                surface_markings.draw(ctx, marking)

        dir = os.path.join(path)
        os.makedirs(dir, exist_ok=True)

        surface.write_to_png(os.path.join(dir, f"{self._name}.png"))

        return size


def _import_road_module(path: str) -> Tuple[Road, str, str]:
    module_name = os.path.basename(path.rstrip(".py"))

    try:
        spec = importlib.util.spec_from_file_location(module_name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except ImportError as e:
        raise ImportError(f"Could not import road from {path}: {e}")

    try:
        road = getattr(module, "road")
    except AttributeError:
        raise AttributeError("The provided road does not contain a 'road' object.")

    return road, path, module_name


def load(road_path: str, seed: str = "smartrollerz") -> Road:
    """
    Load a road from a python file.

    Args:
        road_path: Path to the python file containing the road.
        seed: Predetermine random values.
    """

    # Set random seed
    # set it at this point because the module is reloaded afterwards
    # the above import is just to ensure that the road is in the module cache
    random.seed(seed)

    # Ensure that the road is up to date
    road, file_path, road_name = _import_road_module(road_path)

    road._file_path = file_path
    road._name = road_name
    road._seed = seed

    return road
