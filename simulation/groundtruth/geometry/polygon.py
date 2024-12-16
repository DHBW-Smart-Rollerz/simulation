"""Polygon."""

from contextlib import suppress

import geometry_msgs.msg as geometry_msgs
import numpy as np
import shapely.affinity as affinity
import shapely.geometry

from simulation.groundtruth.geometry.point import Point
from simulation.groundtruth.geometry.transform import Transform

from .frame import validate_and_maintain_frames


class Polygon:
    """Polygon class using shapely's Polygon class.

    Composition with shapely enables to use their powerful operations in combination
    with other objects, e.g. polygon intersections.

    Initialization can be done in one of the following ways.

    Args:
        1 ([Point]): List of points or anything that can be initialized as a point,
                     e.g. Vector, geometry_msgs.Point,np.array
        2 (geometry_msgs.Polygon)
        3 ((Line,Line)): Two lines which are interpreted as the left
                         and right boundary of the polygon, e.g.
                         a road lane
    """

    def __init__(self, *args, frame=None):
        """Polygon initialization."""

        # Due to recursive calling of the init function, the frame should be set
        # in the first call within the recursion only.
        if not hasattr(self, "_frame"):
            self._frame = frame

        # Create points out of arguments:
        with suppress(NotImplementedError, IndexError, TypeError):
            args = ([Point(p) for p in args[0]], 0)

        # Try to initialize directly
        with suppress(Exception):
            self._polygon = shapely.geometry.polygon.Polygon(*args)
            return

        # Try to initialize from list of points
        with suppress(Exception):
            args = [[p.x, p.y, p.z] for p in args[0]]
            self._polygon = shapely.geometry.polygon.Polygon(args)
            return

        # Try to initialize from geometry_msgs/Polygon
        with suppress(Exception):
            self._polygon = shapely.geometry.polygon.Polygon(
                [[p.x, p.y, p.z] for p in args[0].points]
            )
            return

        # Try to initialize from two lines
        with suppress(AttributeError):
            points = args[1].get_points()
            points.extend(reversed(args[0].get_points()))

            self.__init__(points)
            return

        # None of the initializations worked
        raise NotImplementedError(
            f"Polygon initialization not implemented for {type(args[0])}"
        )

    def get_points(self) -> list[Point]:
        """Points of polygon.

        Returns:
            list of points on the polygon.
        """
        return [
            Point(x, y, z, frame=self._frame)
            for x, y, z in self._polygon.exterior.coords
        ]

    def to_geometry_msg(self):
        """To ROS geometry_msg.

        Returns:
            This polygon as a geometry_msgs.Polygon.
        """
        msg = geometry_msgs.Polygon()
        msg.points = [geometry_msgs.Point32(*p.to_numpy()) for p in self.get_points()]
        return msg

    def to_numpy(self):
        """To numpy array.

        Returns:
            Polygon as a numpy array of np.arrays.
        """
        return np.array([p.to_numpy() for p in self.get_points()])

    @validate_and_maintain_frames
    def __rmul__(self, tf: Transform):
        """Transform this polygon.

        Args:
            tf (Transform): Transformation to apply

        Rotate the polygon tf.rotation around (0,0,0) and translate by tf.xyz

        Returns:
            Transformed polygon.
        """
        if type(tf) is not Transform:
            return NotImplemented

        # Get affine matrix, turn into list and restructure how shapely expects the input
        flat = list(tf.to_affine_matrix().flatten())
        flat = flat[0:3] + flat[4:7] + flat[8:11] + [flat[3], flat[7], flat[11]]

        transformed = affinity.affine_transform(self._polygon, flat)

        return self.__class__(transformed.exterior.coords)

    @validate_and_maintain_frames
    def __eq__(self, polygon: "Polygon"):
        """Compare two polygons using shapely's equals_exact.

        Also allow the points to be provided in the reversed order.
        """
        return self._polygon.equals_exact(
            polygon._polygon
        ) or self._polygon.equals_exact(
            Polygon(reversed(polygon.get_points()))._polygon
        )

    def __repr__(self) -> str:
        return f"{self.__class__.__qualname__}({self.get_points()})" + (
            f",frame={self._frame.name}" if self._frame is not None else ""
        )

    def __getattr__(self, name):
        """Delegate attribute access to the shapely object."""

        # Avoid recursion for attributes starting with '_'
        if name.startswith("_"):
            raise AttributeError(
                f"'{self.__class__.__name__}' object has no attribute '{name}'"
            )

        # Check if the attribute exists in the shapely object
        if hasattr(self._polygon, name):
            return getattr(self._polygon, name)
        else:
            # Raise an AttributeError if the attribute is not found
            raise AttributeError(
                f"'{self.__class__.__name__}' object has no attribute '{name}'"
            )
