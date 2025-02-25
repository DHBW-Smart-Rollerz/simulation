"""ZebraCrossing."""

from dataclasses import dataclass

import simulation.groundtruth.road.sections.type as road_section_type
from simulation.groundtruth.geometry import Point, Polygon
from simulation.groundtruth.road.config import Config
from simulation.groundtruth.road.sections import (
    StraightRoad,
    SurfaceMarkingRect,
    TrafficSign,
)


@dataclass
class ZebraCrossing(StraightRoad):
    """Road section representing a zebra crossing.

    Args:
        length (float) = 0.45: length of the crossing and thus the section.
    """

    TYPE = road_section_type.ZEBRA_CROSSING

    length: float = 0.45

    def __post_init__(self):
        self.surface_markings.append(
            SurfaceMarkingRect(
                arc_length=self.length / 2,
                y=0,
                depth=self.length,
                width=2 * Config.road_width,
                kind=SurfaceMarkingRect.ZEBRA_CROSSING,
            )
        )
        self.traffic_signs.append(
            TrafficSign(
                kind=TrafficSign.ZEBRA_CROSSING,
                arc_length=-0.4,
                y=-Config.road_width - 0.1,
                angle=0,
                normalize_x=False,
            )
        )
        self.middle_line_marking = self.MISSING_LINE_MARKING
        super().__post_init__()

    @property
    def frame(self) -> Polygon:
        """Polygon : Frame for the zebra crossing surface marking."""
        poly = Polygon(
            [
                Point(0, -Config.road_width),
                Point(0, Config.road_width),
                Point(self.length, Config.road_width),
                Point(self.length, -Config.road_width),
            ]
        )
        return self.transform * poly
