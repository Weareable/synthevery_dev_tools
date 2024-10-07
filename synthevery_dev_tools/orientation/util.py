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

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts roll, pitch, and yaw to a quaternion.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz


def normalize_angle_to_range(angle, min_angle, max_angle):
    # 角度を指定した範囲に正規化
    range_size = max_angle - min_angle
    return (angle - min_angle) % range_size + min_angle