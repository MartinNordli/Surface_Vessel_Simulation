"""ROS-independent rigid transforms; quaternion order is x, y, z, w.

These helpers are shared by the perception and mapping nodes and their pure
Python cores. They take plain numbers, NumPy arrays or duck-typed ROS messages
(anything with the right attribute names), so they can be unit tested without
a ROS installation.
"""
import math
import numpy as np


def yaw_from_quaternion(q):
    """Return the heading (yaw) in radians of a quaternion message.

    ``q`` is any object with ``x``, ``y``, ``z`` and ``w`` attributes, such as
    ``geometry_msgs/Quaternion``. Yaw is the rotation about +z (Z-Y-X Euler
    convention), counter-clockwise positive; in the ENU ``map`` frame 0 means
    facing east. The result lies in [-pi, pi]. The quaternion is assumed to be
    (close to) unit length and is not normalized here.
    """
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def rotation_matrix(quaternion):
    """Return the 3x3 rotation matrix of an (x, y, z, w) quaternion.

    The quaternion is normalized first, so a slightly non-unit input from a
    message still gives a proper rotation. Raises ``ValueError`` for a zero or
    non-finite quaternion instead of silently returning garbage.
    """
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("rotation quaternion must be finite and nonzero")
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def transform_points(points, translation, quaternion):
    """Apply a rigid transform ``p' = R p + t`` to a set of 3D points.

    Args:
        points: array-like reshaped to (N, 3), in the source frame [m].
        translation: (x, y, z) of the source origin in the target frame [m].
        quaternion: (x, y, z, w) orientation of the source frame in the target.

    Returns:
        (N, 3) float array of the same points expressed in the target frame.
    """
    return (np.asarray(points, dtype=float).reshape(-1, 3) @ rotation_matrix(quaternion).T
            + np.asarray(translation))


def transform_from_ros(points, transform):
    """Accept a TransformStamped without importing ROS in this utility.

    ``transform`` is the result of ``tf2_ros.Buffer.lookup_transform(target,
    source, time)``: it maps points given in ``source`` into ``target``. Callers
    look it up at the acquisition stamp of the data being transformed, so a
    moving vessel is handled correctly.
    """
    t, q = transform.transform.translation, transform.transform.rotation
    return transform_points(points, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def stamp_seconds(stamp):
    """Convert a ``builtin_interfaces/Time`` stamp to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9
