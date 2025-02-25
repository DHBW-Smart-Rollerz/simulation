"""Transformation."""

import math
import numbers
from contextlib import suppress

# Compatible formats
import geometry_msgs.msg as geometry_msgs
import numpy as np
from pyquaternion import Quaternion

from simulation.groundtruth.geometry.vector import Vector

from .frame import validate_and_maintain_frames


class Transform:
    """Transformation class consisting of a translation and a rotation.

    A Transform object can be used to easily interchange between multiple coordinate systems
    (which can be transformed in one another via a rotation and a translation.

    Initialization can be done in one of the following ways:

    Args:
        1 (geometry_msgs/Transformation): initialize from geometry_msgs.
        2 (Vector, float): Second argument is the transformation's rotation angle in radian.
        3 (Vector, pyquaternion.Quaternion): Vector and quaternion.


    Attributes:
        tranlation (Vector)
        rotation (pyquaternion.Quaternion)
    """

    def __init__(self, *args, frame=None):
        """Transform initialization."""

        # Due to recursive calling of the init function, the frame should be set
        # in the first call within the recursion only.
        if not hasattr(self, "_frame"):
            self._frame = frame

        # Attempt initialization from Vector like and Quaternion like objects
        with suppress(Exception):
            args = (args[0], Quaternion(*args[1]))

        with suppress(Exception):
            if isinstance(args[1], numbers.Number):
                args = (args[0], Quaternion(axis=[0, 0, 1], radians=args[1]))
            pass

        # Attempt default init
        with suppress(IndexError, NotImplementedError, TypeError):
            if isinstance(args[1], Quaternion):
                self.rotation = args[1]
                self.translation = Vector(args[0])
                return

        # Try to initialize geometry pose
        with suppress(Exception):
            # Call this function with values extracted
            t = Vector(args[0].position)
            g_quaternion = args[0].orientation
            q = Quaternion(
                g_quaternion.w, g_quaternion.x, g_quaternion.y, g_quaternion.z
            )
            self.__init__(t, q)
            return

        # Try to initialize geometry transform
        with suppress(Exception):
            # Call this function with values extracted
            t = Vector(args[0].translation)
            g_quaternion = args[0].rotation
            q = Quaternion(
                g_quaternion.w, g_quaternion.x, g_quaternion.y, g_quaternion.z
            )
            self.__init__(t, q)
            return

        with suppress(Exception):
            # Try to initialize with two vectors translation+rotation
            t = args[0]
            rotation_vec = args[1].to_numpy()
            angle = (-1 if rotation_vec[1] < 0 else 1) * math.acos(
                np.dot([1, 0, 0], rotation_vec) / np.linalg.norm(rotation_vec)
            )

            self.__init__(t, angle)
            return

        # None of the initializations worked
        raise NotImplementedError(
            f"Transform initialization not implemented for {type(args[0])}"
        )

    @property
    @validate_and_maintain_frames
    def inverse(self) -> "Transform":
        """Inverse transformation."""
        return Transform(
            -1 * Vector(self.translation).rotated(self.rotation.inverse),
            self.rotation.inverse,
        )

    def get_angle(self, axis=Vector(0, 0, 1)) -> float:
        """Angle of rotation.

        Args:
            axis: Axis the rotation is projected onto.

        Returns:
            The angle that a vector is rotated, when this transformation is applied.
        """

        # Project the rotation axis onto the rotation axis to get the amount of the rotation
        # that is in the axis' direction!
        # Also the quaternions rotation axis is sometimes flipped at which point
        # the angles flip their sign,
        # taking the scalar product with the axis fixes that as well
        return Vector(self.rotation.axis) * axis * self.rotation.radians

    def to_geometry_msg(self) -> geometry_msgs.Transform:
        """Convert transform to ROS geometry_msg.

        Returns:
            This transformation as a geometry_msgs/Transform.
        """
        vector = self.translation.to_geometry_msg()
        rotation = geometry_msgs.Quaternion(
            self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w
        )

        tf = geometry_msgs.Transform()
        tf.translation = vector
        tf.rotation = rotation

        return tf

    def to_affine_matrix(self) -> np.ndarray:
        """Get transformation as an affine matrix."""
        return np.column_stack(
            (
                self.rotation.rotation_matrix,
                self.translation.to_numpy(),
            )
        )

    @validate_and_maintain_frames
    def __mul__(self, tf: "Transform") -> "Transform":
        """Multiplication of transformations.

        The product has to be understood as a single transformation consisting of
        the right hand transformation applied first and then the left hand transformation.

        Example:
            Easily modify a vector multiple times:

            :math:`(\\text{Tf}_1*\\text{Tf}_2)*\\vec{v} =
                \\text{Tf}_1*( \\text{Tf}_2*\\vec{v})`

        Returns:
            The product transformation.
        """
        if tf.__class__ == self.__class__:
            return self.__class__(
                Vector(self.translation)
                + Vector(tf.translation).rotated(self.rotation),
                self.rotation * tf.rotation,
                frame=self._frame,
            )

        return NotImplemented

    @validate_and_maintain_frames
    def __eq__(self, tf) -> bool:
        if self.__class__ != tf.__class__:
            return NotImplemented
        return tf.rotation.normalised == self.rotation.normalised and Vector(
            self.translation
        ) == Vector(tf.translation)

    def __repr__(self) -> str:
        return (
            f"Transform(translation={repr(self.translation)},"
            + f"rotation={repr(self.rotation)}"
            + (f",frame={self._frame.name}" if self._frame is not None else "")
            + ")"
        )

    def __hash__(self):
        return NotImplemented
