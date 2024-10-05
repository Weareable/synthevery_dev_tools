import math

def quaternion_to_euler(w, x, y, z):
    """
    Computes roll, pitch, and yaw from the current quaternion.
    """
    roll = math.atan2(w * x + y * z,
                            0.5 - x * x - y * y)
    pitch = math.asin(-2.0 * (x * z - w * y))
    yaw = math.atan2(x * y + w * z,
                            0.5 - y * y - z * z)

    return roll, pitch, yaw