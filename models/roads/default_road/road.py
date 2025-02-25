import math

from simulation.groundtruth.road.road import Road
from simulation.groundtruth.road.sections import (
    Intersection,
    LeftCircularArc,
    ParkingArea,
    ParkingLot,
    ParkingObstacle,
    ParkingSpot,
    StaticObstacle,
    StraightRoad,
    ZebraCrossing,
)
from simulation.groundtruth.road.sections.road_section import RoadSection

road = Road()
road.append(StraightRoad(length=1))

# Create a parking area with different kinds of parking spots
road.append(
    ParkingArea(
        length=4,
        start_line=True,
        left_lots=[
            ParkingLot(
                start=1,
                spots=[ParkingSpot(), ParkingSpot(kind=ParkingSpot.BLOCKED)],
            )
        ],
        right_lots=[
            ParkingLot(
                start=0.2,
                spots=[
                    ParkingSpot(
                        kind=ParkingSpot.OCCUPIED,
                        width=0.7,
                        obstacle=ParkingObstacle(x=0.15, y=-0.2, width=0.3, depth=0.25),
                    ),
                    ParkingSpot(kind=ParkingSpot.BLOCKED),
                ],
            )
        ],
    )
)
road.append(
    LeftCircularArc(
        radius=2,
        angle=math.pi / 2,
        right_line_marking=RoadSection.MISSING_LINE_MARKING,
    )
)
road.append(StraightRoad(length=0.45))
road.append(LeftCircularArc(radius=2, angle=math.pi / 2))
road.append(Intersection(size=3, turn=Intersection.RIGHT))
road.append(LeftCircularArc(radius=1.5, angle=math.pi, obstacles=[StaticObstacle()]))
road.append(Intersection(size=3, turn=Intersection.RIGHT))

left_arc = LeftCircularArc(
    radius=1.5,
    angle=math.pi / 2,
    left_line_marking=RoadSection.MISSING_LINE_MARKING,
)
left_arc.add_speed_limit(arc_length=0, speed=30)
road.append(left_arc)

road.append(ZebraCrossing())
road.append(StraightRoad(length=1))

left_arc = LeftCircularArc(radius=1.5, angle=math.pi / 2)
left_arc.add_speed_limit(0, -30)
road.append(left_arc)

road.append(StraightRoad(length=0.95))
