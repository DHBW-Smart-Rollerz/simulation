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
    return f"""
        <sdf version="1.8">
        <model name='model'>
            <static>1</static>
            <link name='link'>
                <visual name='visual'>
                    <cast_shadows>1</cast_shadows>
                    <geometry>
                        <mesh>
                            <uri>package://models/meshes/{mesh}.dae</uri>
                            <scale>1 1 1</scale>
                        </mesh>
                    </geometry>
                </visual>
                <collision name='collision'>
                    <pose>{collision_box_position[0]} {collision_box_position[1]} {collision_box_position[2]} 0 0 0</pose>
                    <geometry>
                        <box>
                        <size>{collision_box_size[0]} {collision_box_size[1]} {collision_box_size[2]}</size>
                        </box>
                    </geometry>
                </collision>
                <self_collide>0</self_collide>
            </link>
        </model>
        </sdf>
        """
