import math
from typing import List

class GlobalAccel:
    @staticmethod
    def multiply_quaternion(q: List[float], p: List[float]) -> List[float]:
        """
        Multiplies two quaternions.

        :param q: First quaternion [w, x, y, z].
        :param p: Second quaternion [w, x, y, z].
        :return: Product quaternion [w, x, y, z].
        """
        if len(q) != 4 or len(p) != 4:
            raise ValueError("Both quaternions must have exactly 4 elements [w, x, y, z].")
        
        qw, qx, qy, qz = q
        pw, px, py, pz = p

        rw = qw * pw - qx * px - qy * py - qz * pz
        rx = qw * px + qx * pw + qy * pz - qz * py
        ry = qw * py - qx * pz + qy * pw + qz * px
        rz = qw * pz + qx * py - qy * px + qz * pw

        return [rw, rx, ry, rz]

    @staticmethod
    def rotate_vector(v: List[float], q: List[float]) -> List[float]:
        """
        Rotates a vector by a quaternion.

        :param v: Vector to rotate [x, y, z].
        :param q: Quaternion [w, x, y, z].
        :return: Rotated vector [x, y, z].
        """
        if len(v) != 3:
            raise ValueError("Vector must have exactly 3 elements [x, y, z].")
        if len(q) != 4:
            raise ValueError("Quaternion must have exactly 4 elements [w, x, y, z].")

        # Calculate the conjugate of the quaternion
        conjugate = [q[0], -q[1], -q[2], -q[3]]

        # Convert the vector into a quaternion with w=0
        vec_quat = [0.0] + v

        # Multiply q * vec_quat
        qp = GlobalAccel.multiply_quaternion(q, vec_quat)

        # Multiply the result by the conjugate: (q * v) * q_conj
        qpq_conj = GlobalAccel.multiply_quaternion(qp, conjugate)

        # The rotated vector is the vector part of the resulting quaternion
        return qpq_conj[1:]

    @staticmethod
    def inv_sqrt(x: float) -> float:
        """
        Computes the inverse square root of a number.

        :param x: The number to compute the inverse square root of.
        :return: The inverse square root of x.
        """
        if x <= 0.0:
            raise ValueError("Cannot compute inverse square root of non-positive number.")
        return 1.0 / math.sqrt(x)

    @staticmethod
    def get_quaternion_from_rp(roll: float, pitch: float) -> List[float]:
        """
        Computes a normalized quaternion from roll and pitch angles.

        :param roll: Roll angle in radians.
        :param pitch: Pitch angle in radians.
        :return: Normalized quaternion [w, x, y, z].
        """
        cos_roll = math.cos(roll * 0.5)
        sin_roll = math.sin(roll * 0.5)
        cos_pitch = math.cos(pitch * 0.5)
        sin_pitch = math.sin(pitch * 0.5)

        qw = cos_roll * cos_pitch
        qx = sin_roll * cos_pitch
        qy = cos_roll * sin_pitch
        qz = -sin_roll * sin_pitch

        norm = GlobalAccel.inv_sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        normalized_quat = [qw * norm, qx * norm, qy * norm, qz * norm]
        
        # Debug: Print quaternion before and after normalization
        # print(f"Quaternion before normalization: [{qw}, {qx}, {qy}, {qz}]")
        # print(f"Normalized Quaternion: {normalized_quat}")

        return normalized_quat

    @staticmethod
    def update(accel: List[float], roll: float, pitch: float) -> List[float]:
        """
        Updates the global acceleration by rotating the local acceleration vector
        using the provided roll and pitch angles. Assumes yaw is not used.
        Adds 1.0 to the z-component to account for gravity.

        :param accel: Local acceleration vector [x, y, z].
        :param roll: Roll angle in degrees.
        :param pitch: Pitch angle in degrees.
        :return: Global acceleration vector [x, y, z].
        """
        if len(accel) != 3:
            raise ValueError("Acceleration vector must have exactly 3 elements [x, y, z].")

        # Convert roll and pitch from degrees to radians and negate
        try:
            roll_rad = -math.radians(roll)
            pitch_rad = -math.radians(pitch)
        except Exception as e:
            raise ValueError(f"Error converting angles to radians: {e}")

        # Get the quaternion from roll and pitch
        try:
            quat = GlobalAccel.get_quaternion_from_rp(roll_rad, pitch_rad)
        except Exception as e:
            raise ValueError(f"Error computing quaternion: {e}")

        # Rotate the local acceleration to global frame
        try:
            rotated_accel = GlobalAccel.rotate_vector(accel, quat)
        except Exception as e:
            raise ValueError(f"Error rotating vector: {e}")

        # Add gravity (assuming gravity is along positive Z in global frame)
        rotated_accel[2] += 1.0  # Adjust this value based on your units (e.g., 9.81 m/s²)

        return rotated_accel
