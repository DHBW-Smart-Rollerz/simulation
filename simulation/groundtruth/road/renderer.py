import itertools
from collections import defaultdict

from simulation.groundtruth.geometry import Vector
from simulation.groundtruth.road.renderer.tile import Tile
from simulation.groundtruth.road.road import Road


def main():
    pass

    road = default_road()
    tile_size = Vector(15, 15)
    tile_resolution = Vector(7 * 512, 7 * 512)

    tiles: list[Tile] = create_new_tiles(road, tile_size, tile_resolution)

    for tile in tiles:
        tile.render_to_file(
            "/home/smartrollerz/Smartrollerz/smarty_workspace/src/simulation/config/default_road"
        )


def create_new_tiles(
    road: Road,
    tile_size: Vector,
    tile_resolution: Vector,
) -> list[Tile]:
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


def default_road() -> Road:
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
                            obstacle=ParkingObstacle(
                                x=0.15, y=-0.2, width=0.3, depth=0.25
                            ),
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
    road.append(
        LeftCircularArc(radius=1.5, angle=math.pi, obstacles=[StaticObstacle()])
    )
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

    return road


if __name__ == "__main__":
    main()
