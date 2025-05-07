import math

def quaternion_multiply(q1, q0):
    """Multiply two quaternions."""
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    return [
        w1*x0 + x1*w0 + y1*z0 - z1*y0,
        w1*y0 - x1*z0 + y1*w0 + z1*x0,
        w1*z0 + x1*y0 - y1*x0 + z1*w0,
        w1*w0 - x1*x0 - y1*y0 - z1*z0
    ]

def quaternion_inverse(q):
    """Compute the inverse of a quaternion."""
    x, y, z, w = q
    norm_sq = x*x + y*y + z*z + w*w
    if norm_sq == 0:
        raise ValueError("Cannot invert zero quaternion")
    return [-x/norm_sq, -y/norm_sq, -z/norm_sq, w/norm_sq]

def quaternion_to_axis_angle(x, y, z, w):
    """Convert quaternion to axis-angle representation."""
    norm = math.sqrt(x*x + y*y + z*z)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.acos(w)
    if abs(angle) < 1e-6:
        return 0.0, 0.0, 0.0
    s = math.sin(angle / 2.0)
    if abs(s) < 1e-6:
        return 0.0, 0.0, 0.0
    return (x / s) * angle, (y / s) * angle, (z / s) * angle

def axis_angle_to_quaternion(rx, ry, rz):
    """Convert axis-angle to quaternion."""
    angle = math.sqrt(rx*rx + ry*ry + rz*rz)
    if angle < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    s = math.sin(angle / 2.0) / angle
    w = math.cos(angle / 2.0)
    x = rx * s
    y = ry * s
    z = rz * s
    return x, y, z, w