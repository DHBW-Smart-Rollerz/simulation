import numpy as np


def generate_sdf_model(obstacle_size: np.ndarray):
    """Generates an SDF string for a obstacle model.

    Args:
        obstacle_size (np.ndarray): The size of the obstacle as a numpy array [x, y, z].

    Returns:
        str: The generated SDF string.
    """
    return """
        <sdf version="1.8">
        <model name='model'>
            <link name='link'>
                <inertial>
                    <mass>1</mass>
                    <inertia>
                        <ixx>0.166667</ixx>
                        <ixy>0</ixy>
                        <ixz>0</ixz>
                        <iyy>0.166667</iyy>
                        <iyz>0</iyz>
                        <izz>0.166667</izz>
                    </inertia>
                </inertial>
                <visual name='visual'>
                    <cast_shadows>1</cast_shadows>
                    <geometry>
                        <box>
                            <size>{size_x} {size_y} {size_z}</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.95 0.95 0.95 1</ambient>
                        <diffuse>0.95 0.95 0.95 1</diffuse>
                        <specular>0.95 0.95 0.95 1</specular>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <box>
                          <size>{size_x} {size_y} {size_z}</size>
                        </box>
                    </geometry>
                </collision>
            </link>
        </model>
        </sdf>
        """.format(
        size_x=obstacle_size[0],
        size_y=obstacle_size[1],
        size_z=obstacle_size[2],
    )
