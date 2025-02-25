"""Definition of the road.sections module.

Collect classes and functions which should be included in the road.sections module.

If these imports are rearranged cyclic imports may occur. To prevent this:
isort:skip_file
"""

from collections import defaultdict


class ID:
    """Container of all class ids.

    Every kind of traffic sign, road surface marking, and obstacle has a unique class id and
    description. These ids can be used to later provide labels for elements within the
    simulated world.
    """

    _counters = defaultdict(lambda: 0)

    @classmethod
    def register(cls, ns: int = 0) -> int:
        cls._counters[ns] += 1
        id_ = ns + cls._counters[ns]
        return id_


from simulation.groundtruth.road.sections.obstacle import (  # noqa: 402
    StaticObstacle,
    ParkingObstacle,
)
from simulation.groundtruth.road.sections.traffic_sign import TrafficSign  # noqa: 402
from simulation.groundtruth.road.sections.surface_marking import (
    SurfaceMarking,
)  # noqa: 402
from simulation.groundtruth.road.sections.surface_marking import (
    SurfaceMarkingPoly,
)  # noqa: 402
from simulation.groundtruth.road.sections.surface_marking import (
    SurfaceMarkingRect,
)  # noqa: 402
from simulation.groundtruth.road.sections.road_section import RoadSection  # noqa: 402
from simulation.groundtruth.road.sections.straight_road import StraightRoad  # noqa: 402
from simulation.groundtruth.road.sections.bezier_curve import (
    QuadBezier,
    CubicBezier,
)  # noqa: 402
from simulation.groundtruth.road.sections.circular_arc import (  # noqa: 402
    LeftCircularArc,
    RightCircularArc,
)
from simulation.groundtruth.road.sections.intersection import Intersection  # noqa: 402
from simulation.groundtruth.road.sections.parking_area import (  # noqa: 402
    ParkingArea,
    ParkingLot,
    ParkingSpot,
)
from simulation.groundtruth.road.sections.zebra_crossing import (
    ZebraCrossing,
)  # noqa: 402
from simulation.groundtruth.road.sections.blocked_area import BlockedArea  # noqa: 402
from simulation.groundtruth.road.sections.traffic_island import (
    TrafficIsland,
)  # noqa: 402
from simulation.groundtruth.road.sections.speed_limit import SpeedLimit  # noqa: 402
from simulation.groundtruth.road.sections.custom_section import (
    CustomSection,
)  # noqa: 402

__all__ = [
    "QuadBezier",
    "CubicBezier",
    "LeftCircularArc",
    "RightCircularArc",
    "Intersection",
    "StaticObstacle",
    "ParkingObstacle",
    "TrafficSign",
    "SurfaceMarkingPoly",
    "SurfaceMarkingRect",
    "ParkingArea",
    "ParkingLot",
    "ParkingSpot",
    "StraightRoad",
    "ZebraCrossing",
    "RoadSection",
    "BlockedArea",
    "TrafficIsland",
    "SpeedLimit",
    "CustomSection",
]
