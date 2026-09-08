"""ROS-independent rigid transforms; quaternion order is x, y, z, w."""
import math
import numpy as np


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def rotation_matrix(quaternion):
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
    return np.asarray(points, dtype=float).reshape(-1, 3) @ rotation_matrix(quaternion).T + np.asarray(translation)


def transform_from_ros(points, transform):
    """Accept a TransformStamped without importing ROS in this utility."""
    t, q = transform.transform.translation, transform.transform.rotation
    return transform_points(points, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9
