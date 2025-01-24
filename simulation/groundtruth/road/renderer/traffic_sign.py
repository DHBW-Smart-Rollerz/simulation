import numpy as np


def generate_sdf_model(
    mesh: str,
    collision_box_position: np.ndarray,
    collision_box_size: np.ndarray,
):
    """Generates an SDF string for a traffic signe model.

    Args:
        mesh (str): The name of the mesh file (without the .dae extension).
        collision_box_position (np.ndarray): The position of the collision box as a
            numpy array [x, y, z].
        collision_box_size (np.ndarray): The size of the collision box as a nump array
            [x, y, z].

    Returns:
        str: The generated SDF string.
    """
    return """
        <sdf version="1.8">
        <model name='model'>
            <static>1</static>
            <link name='link'>
                <visual name='visual'>
                <cast_shadows>1</cast_shadows>
                <geometry>
                    <mesh>
                        <uri>model://models/meshes/{mesh}.dae</uri>
                        <scale>1 1 1</scale>
                    </mesh>
                </geometry>
                </visual>
                <collision name='collision'>
                    <pose>{box_position_x} {box_position_y} {box_position_z} 0 0 0</pose>
                    <geometry>
                        <box>
                        <size>{box_size_x} {box_size_y} {box_size_z}</size>
                        </box>
                    </geometry>
                </collision>
                <self_collide>0</self_collide>
            </link>
        </model>
        </sdf>
        """.format(
        mesh=mesh,
        box_position_x=collision_box_position[0],
        box_position_y=collision_box_position[1],
        box_position_z=collision_box_position[2],
        box_size_x=collision_box_size[0],
        box_size_y=collision_box_size[1],
        box_size_z=collision_box_size[2],
    )
