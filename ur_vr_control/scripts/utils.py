"""Utility functions for GRR"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


def quat_multiply(q1, q2):
    """
    Multiplies two quaternions in [x, y, z, w] format.
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ])


def quat_conjugate(q):
    """
    Computes the conjugate of a quaternion in [x, y, z, w] format.
    """
    q = np.array(q)
    return np.array([-q[0], -q[1], -q[2], q[3]])


def interpolate_quat(quat1, quat2, u):
    """Interpolate between two rotation vectors given a ratio"""
    quat1 = R.from_quat(quat1).as_quat()
    quat2 = R.from_quat(quat2).as_quat()

    # Spherical linear interpolation (SLERP)
    rotations = R.from_quat([quat1, quat2])
    slerp = Slerp([0, 1], rotations)
    interpolated_quat = slerp([u])[0].as_quat()

    return interpolated_quat


def rotvec_to_quat(rotvec):
    """Convert rotation vector to a quaternion"""
    return R.from_rotvec(rotvec).as_quat()


def quat_to_rotvec(quat):
    """Convert a quaternion to a rotation vector"""
    return R.from_quat(quat).as_rotvec()
