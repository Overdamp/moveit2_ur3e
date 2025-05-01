import math

def quaternion_to_axis_angle(x, y, z, w):
    """Convert quaternion to axis-angle representation."""
    # Compute norm
    norm = math.sqrt(x**2 + y**2 + z**2 + w**2)
    
    # Normalize quaternion if norm is not 1.0
    if abs(norm - 1.0) > 1e-4:
        if norm < 1e-6:
            raise ValueError("Quaternion norm is too small")
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        norm = 1.0
    
    # Convert to axis-angle
    angle = 2 * math.acos(w)
    if abs(angle) < 1e-6:
        return 0.0, 0.0, 0.0
    
    s = math.sqrt(1 - w**2)
    if s < 1e-6:
        return 0.0, 0.0, 0.0
    
    rx = x/s * angle
    ry = y/s * angle
    rz = z/s * angle
    
    # Check for invalid values
    if not all(math.isfinite(v) for v in [rx, ry, rz]):
        raise ValueError(f"Invalid axis-angle values: rx={rx}, ry={ry}, rz={rz}")
    
    # Clamp axis-angle to ±π
    max_angle = math.pi
    rx = max(-max_angle, min(max_angle, rx))
    ry = max(-max_angle, min(max_angle, ry))
    rz = max(-max_angle, min(max_angle, rz))
    
    return rx, ry, rz