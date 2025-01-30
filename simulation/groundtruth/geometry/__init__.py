"""Definition of the geometry module.

Collect classes and functions which should be included in the geometry module.
"""

from simulation.groundtruth.geometry.line import Line  # noqa: 402
from simulation.groundtruth.geometry.point import (  # noqa: 402
    InvalidPointOperationError,
    Point,
)
from simulation.groundtruth.geometry.polygon import Polygon  # noqa: 402
from simulation.groundtruth.geometry.pose import Pose  # noqa: 402
from simulation.groundtruth.geometry.transform import Transform  # noqa: 402
from simulation.groundtruth.geometry.vector import Vector  # noqa: 402

__all__ = ["Vector", "Point", "Transform", "Pose", "Line", "Polygon"]
