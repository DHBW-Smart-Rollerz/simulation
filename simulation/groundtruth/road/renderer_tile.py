import numpy as np

from simulation.groundtruth.geometry import Vector
from simulation.groundtruth.road.renderer.tile import Tile
from simulation.groundtruth.road.road import Road

# def render_to_file(self, roads_path: str):
#     """Render an image of the tile and save it to a file.

#     Args:
#         roads_path: Directory in which all roads are located.
#     """
#     # Resolution of the image per meter
#     resolution = (512, 512)
#     size = (15, 15)

#     surface = cairo.ImageSurface(
#         cairo.FORMAT_RGB24, int(resolution[0]), int(resolution[1])
#     )
#     ctx = cairo.Context(surface)

#     # Adjust scale
#     ctx.scale(resolution[0] / size[0], resolution[1] / size[1])
#     # Inverse y-axis
#     ctx.translate(0, size[1] / 2)
#     ctx.scale(1, -1)
#     ctx.translate(0, -size[1] / 2)

#     # Move to center of the tile
#     ctx.translate(size[0] / 2, size[1] / 2)

#     # Create black background
#     ctx.set_source_rgb(0, 0, 0)
#     ctx.rectangle(0, 0, size[0], size[1])
#     ctx.fill()

#     # Invert the render transform
#     ctx.translate(-self.transform.translation.x, -self.transform.translation.y)

#     # Draw lines for all sections
#     for sec in self.sections.values():
#         for line in sec.lines:
#             # print("LINE1234: ", line)
#             utils.draw_line(ctx, line)
#         for marking in sec.surface_markings:
#             render_surface_markings.draw(ctx, marking)

#     sha_256 = hashlib.sha256()
#     sha_256.update(surface.get_data())
#     hash = sha_256.hexdigest()

#     self.id = f"tile-{hash}"

#     dir = os.path.join(roads_path, self.road_folder_name, self.id)
#     if not os.path.exists(dir):
#         try:
#             os.makedirs(dir)
#         except OSError as exc:  # Guard against race condition
#             if exc.errno != errno.EEXIST:
#                 raise

#     surface.write_to_png(os.path.join(dir, self.id + ".png"))

#     with open(os.path.join(dir, self.id + ".material"), "w+") as file:
#         file.write(self.get_material_string())


def main():
    pass

    road = default_road()
    tile_size = Vector(2, 2)
    tile_resolution = Vector(512, 512)

    tiles: list[Tile] = create_new_tiles(road, tile_size, tile_resolution)

    # for tile in tiles:
    #     tile.render_to_file(
    #         "/home/smartrollerz/Smartrollerz/smarty_workspace/src/simulation/config/default_road"
    #     )


def create_new_tiles(
    road: Road,
    tile_size: Vector,
    tile_resolution: Vector,
) -> list[Tile]:
    sections = road.sections

    min_x = 0.0
    min_y = 0.0
    max_x = 0.0
    max_y = 0.0

    for sec in sections:
        box = sec.get_bounding_box()
        minx, miny, maxx, maxy = box.bounds

        if minx < min_x:
            min_x = minx
        if miny < min_y:
            min_y = miny
        if maxx > max_x:
            max_x = maxx
        if maxy > max_y:
            max_y = maxy

    print(f"{sec.TYPE}: {min_x:5.2f}/{min_y:5.2f}  {max_x:5.2f}/{max_y:5.2f}")

    dim_x = max_x - min_x
    dim_y = max_y - min_y

    size = (np.ceil(dim_x) * 2, np.ceil(dim_y) * 2)
    resolution_meter = (128, 128)
    tile = Tile(
        index=(0, 0),
        size=Vector(size[0], size[1]),
        resolution=Vector(resolution_meter[0] * size[0], resolution_meter[1] * size[1]),
        road_folder_name=f".{road._name}",
        sections={section.id: section for section in sections},
    )

    tile.render_to_file(
        "/home/smartrollerz/Smartrollerz/smarty_workspace/src/simulation/simulation/groundtruth/roadv2"
    )


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
