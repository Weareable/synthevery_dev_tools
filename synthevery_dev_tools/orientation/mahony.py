import math

class MahonyFilter:
    def __init__(self, prop_gain=0.5, int_gain=0.0, sample_freq=512.0):
        """
        Initializes the Mahony AHRS filter.

        :param prop_gain: Proportional gain (Kp)
        :param int_gain: Integral gain (Ki)
        :param sample_freq: Sample frequency in Hz
        """
        self.two_kp = 2.0 * prop_gain  # 2 * proportional gain
        self.two_ki = 2.0 * int_gain   # 2 * integral gain

        # Quaternion representing the orientation
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        # Integral feedback terms
        self.integral_fb_x = 0.0
        self.integral_fb_y = 0.0
        self.integral_fb_z = 0.0

        self.inv_sample_freq = 1.0 / sample_freq

        # Orientation angles
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.grav = [0.0, 0.0, 0.0]
        self.angles_computed = False

    def inv_sqrt(self, x):
        """
        Fast inverse square root approximation.
        In Python, it's more accurate and efficient to use 1/math.sqrt(x).
        """
        return 1.0 / math.sqrt(x) if x != 0 else 0.0

    def compute_angles(self):
        """
        Computes roll, pitch, and yaw from the current quaternion.
        """
        self.roll = math.atan2(self.q0 * self.q1 + self.q2 * self.q3,
                               0.5 - self.q1 * self.q1 - self.q2 * self.q2)
        self.pitch = math.asin(-2.0 * (self.q1 * self.q3 - self.q0 * self.q2))
        self.yaw = math.atan2(self.q1 * self.q2 + self.q0 * self.q3,
                              0.5 - self.q2 * self.q2 - self.q3 * self.q3)

        self.grav[0] = 2.0 * (self.q1 * self.q3 - self.q0 * self.q2)
        self.grav[1] = 2.0 * (self.q0 * self.q1 + self.q2 * self.q3)
        self.grav[2] = 2.0 * (self.q0 * self.q0 - 0.5 + self.q3 * self.q3)

        self.angles_computed = True

    def update_imu(self, gx, gy, gz, ax, ay, az, dt=None):
        """
        Updates the AHRS filter with new gyroscope and accelerometer data.

        :param gx: Gyroscope X-axis data in degrees/sec
        :param gy: Gyroscope Y-axis data in degrees/sec
        :param gz: Gyroscope Z-axis data in degrees/sec
        :param ax: Accelerometer X-axis data
        :param ay: Accelerometer Y-axis data
        :param az: Accelerometer Z-axis data
        :param dt: Time step in seconds. If None, uses the inverse sample frequency.
        """
        if dt is None:
            dt = self.inv_sample_freq

        # Convert gyroscope degrees/sec to radians/sec
        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        gz_rad = math.radians(gz)

        # Compute feedback only if accelerometer measurement is valid
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            # Normalize accelerometer measurement
            recip_norm = self.inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm

            # Estimated direction of gravity
            half_vx = self.q1 * self.q3 - self.q0 * self.q2
            half_vy = self.q0 * self.q1 + self.q2 * self.q3
            half_vz = self.q0 * self.q0 - 0.5 + self.q3 * self.q3

            # Error is cross product between estimated and measured direction of gravity
            error_x = (ay * half_vz - az * half_vy)
            error_y = (az * half_vx - ax * half_vz)
            error_z = (ax * half_vy - ay * half_vx)

            # Apply integral feedback if enabled
            if self.two_ki > 0.0:
                self.integral_fb_x += self.two_ki * error_x * dt
                self.integral_fb_y += self.two_ki * error_y * dt
                self.integral_fb_z += self.two_ki * error_z * dt

                gx_rad += self.integral_fb_x
                gy_rad += self.integral_fb_y
                gz_rad += self.integral_fb_z
            else:
                self.integral_fb_x = 0.0
                self.integral_fb_y = 0.0
                self.integral_fb_z = 0.0

            # Apply proportional feedback
            gx_rad += self.two_kp * error_x
            gy_rad += self.two_kp * error_y
            gz_rad += self.two_kp * error_z

        # Integrate rate of change of quaternion
        gx_rad *= 0.5 * dt
        gy_rad *= 0.5 * dt
        gz_rad *= 0.5 * dt

        qa = self.q0
        qb = self.q1
        qc = self.q2

        self.q0 += (-qb * gx_rad - qc * gy_rad - self.q3 * gz_rad)
        self.q1 += (qa * gx_rad + qc * gz_rad - self.q3 * gy_rad)
        self.q2 += (qa * gy_rad - qb * gz_rad + self.q3 * gx_rad)
        self.q3 += (qa * gz_rad + qb * gy_rad - qc * gx_rad)

        # Normalize quaternion
        recip_norm = self.inv_sqrt(self.q0 * self.q0 + self.q1 * self.q1 +
                                   self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 *= recip_norm
        self.q1 *= recip_norm
        self.q2 *= recip_norm
        self.q3 *= recip_norm

        self.angles_computed = False

    def get_kp(self):
        """Returns the proportional gain Kp."""
        return self.two_kp / 2.0

    def set_kp(self, kp):
        """Sets the proportional gain Kp."""
        self.two_kp = 2.0 * kp

    def get_ki(self):
        """Returns the integral gain Ki."""
        return self.two_ki / 2.0

    def set_ki(self, ki):
        """Sets the integral gain Ki."""
        self.two_ki = 2.0 * ki

    def get_roll(self, degrees=True):
        """
        Returns the roll angle.

        :param degrees: If True, returns degrees. Otherwise, returns radians.
        """
        if not self.angles_computed:
            self.compute_angles()
        return math.degrees(self.roll) if degrees else self.roll

    def get_pitch(self, degrees=True):
        """
        Returns the pitch angle.

        :param degrees: If True, returns degrees. Otherwise, returns radians.
        """
        if not self.angles_computed:
            self.compute_angles()
        return math.degrees(self.pitch) if degrees else self.pitch

    def get_yaw(self, degrees=True):
        """
        Returns the yaw angle.

        :param degrees: If True, returns degrees. Otherwise, returns radians.
        """
        if not self.angles_computed:
            self.compute_angles()
        yaw = math.degrees(self.yaw) + 180.0  # Adjust as in C++ code
        return yaw if degrees else self.yaw

    def get_roll_radians(self):
        """Returns the roll angle in radians."""
        return self.get_roll(degrees=False)

    def get_pitch_radians(self):
        """Returns the pitch angle in radians."""
        return self.get_pitch(degrees=False)

    def get_yaw_radians(self):
        """Returns the yaw angle in radians."""
        return self.get_yaw(degrees=False)

    def get_quaternion(self):
        """
        Returns the current quaternion as a tuple (w, x, y, z).
        """
        return (self.q0, self.q1, self.q2, self.q3)

    def set_quaternion(self, w, x, y, z):
        """
        Sets the quaternion to the provided values.

        :param w: Quaternion w component
        :param x: Quaternion x component
        :param y: Quaternion y component
        :param z: Quaternion z component
        """
        self.q0 = w
        self.q1 = x
        self.q2 = y
        self.q3 = z
        # Normalize the quaternion
        recip_norm = self.inv_sqrt(self.q0 * self.q0 + self.q1 * self.q1 +
                                   self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 *= recip_norm
        self.q1 *= recip_norm
        self.q2 *= recip_norm
        self.q3 *= recip_norm
        self.angles_computed = False

    def get_gravity_vector(self):
        """
        Returns the gravity vector as a tuple (x, y, z).
        """
        if not self.angles_computed:
            self.compute_angles()
        return tuple(self.grav)

