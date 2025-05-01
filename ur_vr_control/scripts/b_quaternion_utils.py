#!/usr/bin/env python3
import math

def quaternion_to_axis_angle(qx, qy, qz, qw):
    # คำนวณ theta และ axis-angle
    theta = 2.0 * math.acos(qw)
    sin_half_theta = math.sqrt(1 - qw*qw)
    if sin_half_theta < 0.001:
        return (0.0, 0.0, 0.0)  # แทน angular velocity เป็นศูนย์
    else:
        ax = qx / sin_half_theta * theta
        ay = qy / sin_half_theta * theta
        az = qz / sin_half_theta * theta
        return (ax, ay, az)
