"""Line."""

from contextlib import suppress

import numpy as np
import shapely.affinity as affinity
import shapely.geometry
from scipy.ndimage import filters

from simulation.groundtruth.geometry.point import Point
from simulation.groundtruth.geometry.pose import Pose
from simulation.groundtruth.geometry.transform import Transform
from simulation.groundtruth.geometry.vector import Vector

from .frame import validate_and_maintain_frames

APPROXIMATION_DISTANCE = 0.00005
CURVATURE_APPROX_DISTANCE = 0.005


def ensure_valid_arc_length(*, approx_distance=APPROXIMATION_DISTANCE) -> callable:
    """Check if an arc length is on the line and can be used for approximation.

    If the arc_length is too close to the end points of the line,
    it is moved further away from the edges.

    Args:
        approx_distance(float): Approximation step length to be used
                in further calculations.
        Arc length will be at least that far away from the end of the line.
    """

    def wrapper(func):
        def decorator(self, arc_length: float):
            # Ensure that arc_length is not too close to the end points of the road.
            if not (arc_length >= 0 and arc_length <= self.length):
                raise ValueError(
                    "The provided arc length is less than 0 \
                            or greater than the line's length."
                )
            elif self.length < 2 * approx_distance:
                raise ValueError(
                    f"The line must be at least {2*approx_distance} long but is only"
                    f"{self.length} long."
                )

            arc_length = max(arc_length, approx_distance)
            arc_length = min(arc_length, self.length - approx_distance)

            return func(self, arc_length=arc_length)

        decorator.__doc__ = func.__doc__
        decorator.__annotations__ = func.__annotations__

        return decorator

    return wrapper


class Line:
    """List of points as a Line class using shapely's LineString class.

    Composition with shapely enables to use their powerful operations in combination
    with other objects, e.g. polygon intersections.

    Initialization can be done in one of the following ways.

    Args:
        1 ([Point]): List of points or anything that can be initialized as a point,
                     e.g. Vector, geometry_msgs.Point,np.array)
        2 ([]): Empty list creates an empty line.
    """

    @classmethod
    def cut(cls, line: "Line", arc_length: float) -> tuple["Line", "Line"]:
        """Cuts a line in two at a arc_length from its starting point.

        See:
        https://shapely.readthedocs.io/en/latest/manual.html?highlight=cut#linear-referencing-methods
        """
        coords = list(line._linestring.coords)  # Accessing the composed object

        if arc_length == 0:
            return Line(), Line(coords)
        elif arc_length == line.length:
            return Line(coords), Line()
        else:
            assert (
                arc_length > 0.0 and arc_length < line.length
            ), "Invalid arc length given."

        for i, p in enumerate(coords):
            pd = line.project(Point(p))
            if pd == arc_length:
                return (Line(coords[: i + 1]), Line(coords[i:]))
            if pd > arc_length:
                cp = line.interpolate(arc_length)
                return (
                    Line(coords[:i] + [(cp.x, cp.y)], frame=line._frame),
                    Line([(cp.x, cp.y)] + coords[i:], frame=line._frame),
                )

    def __init__(self, *args, frame=None):
        """Line initialization."""

        # Due to recursive calling of the init function, the frame should be set
        # in the first call within the recursion only.
        if not hasattr(self, "_frame"):
            self._frame = frame

        if len(args) == 0:
            args = ([], None)

        # Catch missing z coordinate by converting to point
        with suppress(NotImplementedError, IndexError):
            args = ([Point(arg) for arg in args[0]], None)

        # Try to initialize from list of Point or geometry_msgs/Point
        with suppress(NotImplementedError, AttributeError):
            self._linestring = shapely.geometry.linestring.LineString(
                [[p.x, p.y, p.z] for p in args[0]]
            )
            return

        # None of the initializations worked
        raise NotImplementedError(
            f"Line initialization not implemented for {type(args[0])}"
        )

    def get_points(self) -> list[Point]:
        """Points of line.

        Returns:
            list of points on the line.
        """
        return [
            Point(x, y, z, frame=self._frame) for x, y, z in self._linestring.coords
        ]

    @validate_and_maintain_frames
    def parallel_offset(self, offset: float, side: str) -> "Line":
        """Shift line.

        Args:
            offset (float): distance to shift
            side (str): either `left` or `right` shift

        Returns:
            Line shifted by an offset into the left or right direction.
        """
        assert (
            side == "right" or side == "left"
        ), "Parallel offset is only possible to the right or left!"

        offset_line = self._linestring.parallel_offset(offset, side)
        try:
            coords = offset_line.coords
        except NotImplementedError:
            # If offset_line is a multi part geometry!
            coords = sum([list(line.coords) for line in offset_line], [])

        if side == "right":
            # Because shapely orders right hand offset lines in reverse
            coords = reversed(coords)
        return Line(coords)

    @validate_and_maintain_frames
    def simplify(self, tolerance=0.001):
        coords = self._linestring.simplify(tolerance).coords
        return self.__class__(coords)

    @validate_and_maintain_frames
    def smooth(self, smooth_sigma=0.01):
        """Use a gauss filter to smooth out the LineString coordinates.

        https://programtalk.com/vs2/python/13539/label_centerlines/src_create_centerlines.py/
        """
        smooth_x = np.array(filters.gaussian_filter1d(self.xy[0], smooth_sigma))
        smooth_y = np.array(filters.gaussian_filter1d(self.xy[1], smooth_sigma))
        smoothed_coords = np.hstack((smooth_x, smooth_y))
        smoothed_coords = zip(smooth_x, smooth_y)
        linestring_smoothed = Line(smoothed_coords)
        return linestring_smoothed

    @ensure_valid_arc_length()
    @validate_and_maintain_frames
    def interpolate_direction(self, *, arc_length: float) -> Vector:
        """Interpolate the direction of the line as a vector.

        Approximate by calculating difference vector of a point slightly further
        and a point slightly before along the line.

        Args:
            arc_length (float): Length along the line starting from the first point

        Raises:
            ValueError: If the arc_length is <0 or more than the length of the line.

        Returns:
            Corresponding direction as a normalised vector.
        """
        n = Vector(self.interpolate(arc_length + APPROXIMATION_DISTANCE))
        p = Vector(self.interpolate(arc_length - APPROXIMATION_DISTANCE))

        d = n - p

        return 1 / abs(d) * d

    @ensure_valid_arc_length(approx_distance=CURVATURE_APPROX_DISTANCE)
    def interpolate_curvature(self, *, arc_length: float) -> float:
        """Interpolate the curvature at a given arc_length.

        The curvature is approximated by calculating the Menger curvature as defined
        and described here:
        https://en.wikipedia.org/wiki/Menger_curvature#Definition

        Args:
            arc_length (float): Length along the line starting from the first point

        Raises:
            ValueError: If the arc_length is <0 or more than the length of the line.

        Returns:
            Corresponding curvature.
        """
        p = Vector(
            self.interpolate(arc_length - CURVATURE_APPROX_DISTANCE)
        )  # Previous point
        c = Vector(self.interpolate(arc_length))  # Point at current arc_length
        n = Vector(
            self.interpolate(arc_length + CURVATURE_APPROX_DISTANCE)
        )  # Next point

        # Area of the triangle spanned by p, c, and n.
        # The triangle's area can be computed by the cross product of the vectors.
        cross = (n - c).cross(p - c)

        sign = 1 - 2 * (
            cross.z < 0
        )  # The z-coord sign determines whether the curvature is positive or negative

        return sign * 2 * abs(cross) / (abs(p - c) * abs(n - c) * abs(p - n))

    @ensure_valid_arc_length(approx_distance=0)
    def interpolate_pose(self, *, arc_length: float) -> Pose:
        """Interpolate the pose a model travelling along this line has.

        Args:
            arc_length (float): Length along the line starting from the first point

        Raises:
            ValueError: If the arc_length is <0 or more than the length of the line.

        Returns:
            Corresponding pose.
        """
        point = self.interpolate(arc_length)
        orientation = self.interpolate_direction(arc_length=arc_length)

        return Pose(Point(point), orientation)

    def to_geometry_msg(self):
        """To ROS geometry_msg.

        Returns:
            This line as a geometry_msgs/Polygon.
        """
        return [p.to_geometry_msg() for p in self.get_points()]

    def to_numpy(self):
        """To numpy array.

        Returns:
            Line as a numpy array of np.arrays.
        """
        return np.array([p.to_numpy() for p in self.get_points()])

    @validate_and_maintain_frames
    def __add__(self, line: "Line"):
        """Concatenate lines.

        Returns:
            Lines concatenated behind another.
        """
        coords = list(self._linestring.coords)
        coords.extend(line._linestring.coords)

        return self.__class__(coords)

    @validate_and_maintain_frames
    def __rmul__(self, tf: Transform):
        """Transform this line.

        Args:
            tf (Transform): Transformation to apply

        Rotate the line tf.rotation around (0,0,0) and translate by tf.xyz

        Returns:
            Transformed line.
        """
        if type(tf) is not Transform:
            return NotImplemented

        # Get affine matrix, turn into list and restructure how shapely expects the input
        flat = list(tf.to_affine_matrix().flatten())
        flat = flat[0:3] + flat[4:7] + flat[8:11] + [flat[3], flat[7], flat[11]]

        transformed = affinity.affine_transform(self._linestring, flat)
        return self.__class__(transformed.coords)

    def __eq__(self, line: "Line") -> bool:
        if not self.__class__ == line.__class__:
            return NotImplemented

        # simplify lines (1 mm tolerance) before checking equality
        return self.simplify()._linestring.equals_exact(line.simplify()._linestring)

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
        if hasattr(self._linestring, name):
            return getattr(self._linestring, name)
        else:
            # Raise an AttributeError if the attribute is not found
            raise AttributeError(
                f"'{self.__class__.__name__}' object has no attribute '{name}'"
            )
