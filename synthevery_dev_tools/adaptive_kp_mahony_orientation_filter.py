import math
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped, TransformStamped
from tf2_ros import TransformBroadcaster

import numpy as np

from synthevery_dev_tools.orientation.mahony import MahonyFilter
from synthevery_dev_tools.feature.signal_features import BilinearTransformLPF, MovingWindow
from synthevery_dev_tools.orientation.util import euler_to_quaternion, normalize_angle_to_range


class AdaptiveKpMahonyOrientationFilter(Node):
    def __init__(self):
        super().__init__('akp_mahony')

        self.default_kp = self.declare_parameter("kp", 10.0).get_parameter_value().double_value
        self.default_ki = self.declare_parameter("ki", 0.0).get_parameter_value().double_value
        self.filter_rate = self.declare_parameter("filter_rate", 200.0).get_parameter_value().double_value

        self.orientation_filter = MahonyFilter(self.default_kp, self.default_ki, self.filter_rate)
        self.latest_orientation = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.latest_yaw = 0.0
        self.yaw_bias_average = MovingWindow(100)
        self.kp_prev = 0.0
        self.kp = 0.0
        self.high_accel_count = 0
        self.accel_norm = 0.0
        self.kp_lpf_coef = 0.0

        self.accel_lpf_cutoff_frequency = 2.0
        self.accel_lpf = [BilinearTransformLPF.from_cutoff_frequency(self.accel_lpf_cutoff_frequency, 1.0 / self.filter_rate) for _ in range(3)]

        self.accel_moving_window_size = 100
        self.accel_moving_window = [MovingWindow(self.accel_moving_window_size) for _ in range(3)]

        self.filtered_accel = [0.0, 0.0, 0.0]

        self.accel_subscriber = Subscriber(self, Vector3Stamped, "~/accel")
        self.gyro_subscriber = Subscriber(self, Vector3Stamped, "~/gyro")
        self.time_synchronizer = ApproximateTimeSynchronizer(
            [self.accel_subscriber, self.gyro_subscriber], 
            queue_size=10, 
            slop=1.0 / self.filter_rate
        )
        self.time_synchronizer.registerCallback(self.sensor_callback)

        self.frame_id = self.declare_parameter("frame_id", "body").get_parameter_value().string_value
        self.world_frame_id = self.declare_parameter("world_frame_id", "world").get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.orientation_publisher = self.create_publisher(QuaternionStamped, "~/orientation", 10)
        self.orientation_rpy_publisher = self.create_publisher(Vector3Stamped, "~/orientation_rpy", 10)
        self.raw_orientation_publisher = self.create_publisher(QuaternionStamped, "~/raw_orientation", 10)
        self.raw_orientation_rpy_publisher = self.create_publisher(Vector3Stamped, "~/raw_orientation_rpy", 10)

        self.kp_publisher = self.create_publisher(Float32, "~/kp", 10)
        self.accel_norm_publisher = self.create_publisher(Float32, "~/accel_norm", 10)
        self.high_accel_count_publisher = self.create_publisher(Float32, "~/high_accel_count", 10)
        self.kp_lpf_coef_publisher = self.create_publisher(Float32, "~/kp_lpf_coef", 10)
        self.accel_lpf_publisher = self.create_publisher(Vector3Stamped, "~/accel_lpf", 10)
        self.accel_moving_window_publisher = self.create_publisher(Vector3Stamped, "~/accel_moving_window", 10)
        self.filtered_accel_publisher = self.create_publisher(Vector3Stamped, "~/filtered_accel", 10)

    def sensor_callback(self, accel, gyro):
        self.update_filter(accel, gyro)

        latest_time = max(accel.header.stamp, gyro.header.stamp, key=lambda t: (t.sec, t.nanosec))
        self.publish_orientation(latest_time)
        self.publish_filter_params(latest_time)

    def update_filter(self, accel, gyro):
        # 姿勢推定フィルタの更新
        self.accel_norm = np.linalg.norm(np.array([accel.vector.x, accel.vector.y, accel.vector.z]))

        self.accel_lpf[0].update(min(max(accel.vector.x, -1.0), 1.0))
        self.accel_lpf[1].update(min(max(accel.vector.y, -1.0), 1.0))
        self.accel_lpf[2].update(min(max(accel.vector.z, -1.0), 1.0))

        self.accel_moving_window[0].update(accel.vector.x)
        self.accel_moving_window[1].update(accel.vector.y)
        self.accel_moving_window[2].update(accel.vector.z)
        
        self.kp = self.default_kp - abs(1 - self.accel_norm) * self.default_kp * 3

        self.kp_lpf_coef = min(max(self.accel_norm / 2.0 * 0.1, 0.0), 0.09) + 0.9
        
        if (self.kp_prev < self.kp):
            self.kp = self.kp_prev * self.kp_lpf_coef + self.kp * (1 - self.kp_lpf_coef)
        else:
            self.kp = self.kp_prev * 0.5 + self.kp * 0.5

        self.kp = min(max(self.kp, self.default_kp * 0.2), self.default_kp)
        self.kp_prev = self.kp

        self.orientation_filter.set_kp(self.kp)

        """
        if (self.kp < self.default_kp * 0.5):
            self.orientation_filter.set_kp(0.0)

        if (self.kp < self.default_kp * 0.15):
            self.high_accel_count = min(max(self.high_accel_count + 1, 0), 25)
        else:
            self.high_accel_count = max(self.high_accel_count - 1, 0)
        """

        accel_norm_coef = min(max(abs(1 - self.accel_norm), 0.0), 1.0) ** 2

        self.filtered_accel = [
            self.accel_moving_window[0].average() * accel_norm_coef + (1 - accel_norm_coef) * self.accel_lpf[0].value(),
            self.accel_moving_window[1].average() * accel_norm_coef + (1 - accel_norm_coef) * self.accel_lpf[1].value(),
            self.accel_moving_window[2].average() * accel_norm_coef + (1 - accel_norm_coef) * self.accel_lpf[2].value(),
        ]

        self.orientation_filter.update_imu(
                -gyro.vector.x,
                -gyro.vector.y,
                -gyro.vector.z,
                -self.filtered_accel[0],
                -self.filtered_accel[1],
                -self.filtered_accel[2],
            )
        
        self.latest_orientation[4:6] = [
            self.orientation_filter.get_roll(),
            self.orientation_filter.get_pitch()
        ]

        yaw_delta = normalize_angle_to_range(self.orientation_filter.get_yaw() - self.latest_yaw, -180.0, 180.0)
        self.latest_yaw = self.orientation_filter.get_yaw()

        if abs(yaw_delta) > 0.008:
            self.latest_orientation[6] = self.latest_orientation[6] + yaw_delta
        else:
            self.yaw_bias_average.update(yaw_delta)

        self.get_logger().info(f"yaw_bias_average: {self.yaw_bias_average.average()}", throttle_duration_sec=1.0)

        q = euler_to_quaternion(
            -self.latest_orientation[4] / 180.0 * math.pi, 
            -self.latest_orientation[5] / 180.0 * math.pi, 
            self.latest_orientation[6] / 180.0 * math.pi
        )

        self.latest_orientation[0] = q[0]
        self.latest_orientation[1] = q[1]
        self.latest_orientation[2] = q[2]
        self.latest_orientation[3] = q[3]


    def publish_orientation(self, time):
        msg = QuaternionStamped()
        msg.header.stamp = time
        msg.header.frame_id = self.world_frame_id
        msg.quaternion.w = self.latest_orientation[0]
        msg.quaternion.x = self.latest_orientation[1]
        msg.quaternion.y = self.latest_orientation[2]
        msg.quaternion.z = self.latest_orientation[3]
        self.orientation_publisher.publish(msg)

        msg = Vector3Stamped()
        msg.header.stamp = time
        msg.header.frame_id = self.world_frame_id
        msg.vector.x = self.latest_orientation[4]
        msg.vector.y = self.latest_orientation[5]
        msg.vector.z = self.latest_orientation[6]
        self.orientation_rpy_publisher.publish(msg)

        msg = QuaternionStamped()
        msg.header.stamp = time
        msg.header.frame_id = self.world_frame_id
        msg.quaternion.w = self.orientation_filter.get_quaternion()[0]
        msg.quaternion.x = self.orientation_filter.get_quaternion()[1]
        msg.quaternion.y = self.orientation_filter.get_quaternion()[2]
        msg.quaternion.z = self.orientation_filter.get_quaternion()[3]
        self.raw_orientation_publisher.publish(msg)

        msg = Vector3Stamped()
        msg.header.stamp = time
        msg.header.frame_id = self.world_frame_id
        msg.vector.x = self.orientation_filter.get_roll()
        msg.vector.y = self.orientation_filter.get_pitch()
        msg.vector.z = self.orientation_filter.get_yaw()
        self.raw_orientation_rpy_publisher.publish(msg)

        t = TransformStamped()
        t.header.stamp = time
        t.header.frame_id = self.world_frame_id
        t.child_frame_id = self.frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = self.latest_orientation[0]
        t.transform.rotation.x = self.latest_orientation[1]
        t.transform.rotation.y = self.latest_orientation[2]
        t.transform.rotation.z = self.latest_orientation[3]
        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = time
        t.header.frame_id = self.world_frame_id
        t.child_frame_id = self.frame_id + "_raw"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = self.orientation_filter.get_quaternion()[0]
        t.transform.rotation.x = self.orientation_filter.get_quaternion()[1]
        t.transform.rotation.y = self.orientation_filter.get_quaternion()[2]
        t.transform.rotation.z = self.orientation_filter.get_quaternion()[3]
        self.tf_broadcaster.sendTransform(t)

    def publish_filter_params(self, time):
        msg = Float32()
        msg.data = self.kp
        self.kp_publisher.publish(msg)

        msg = Float32()
        msg.data = self.accel_norm
        self.accel_norm_publisher.publish(msg)

        msg = Float32()
        msg.data = float(self.high_accel_count)
        self.high_accel_count_publisher.publish(msg)

        msg = Float32()
        msg.data = self.kp_lpf_coef
        self.kp_lpf_coef_publisher.publish(msg)

        msg = Vector3Stamped()
        msg.header.stamp = time
        msg.header.frame_id = self.frame_id
        msg.vector.x = self.accel_lpf[0].value()
        msg.vector.y = self.accel_lpf[1].value()
        msg.vector.z = self.accel_lpf[2].value()
        self.accel_lpf_publisher.publish(msg)

        msg = Vector3Stamped()
        msg.header.stamp = time
        msg.header.frame_id = self.frame_id
        msg.vector.x = self.accel_moving_window[0].average()
        msg.vector.y = self.accel_moving_window[1].average()
        msg.vector.z = self.accel_moving_window[2].average()
        self.accel_moving_window_publisher.publish(msg)

        msg = Vector3Stamped()
        msg.header.stamp = time
        msg.header.frame_id = self.frame_id
        msg.vector.x = self.filtered_accel[0]
        msg.vector.y = self.filtered_accel[1]
        msg.vector.z = self.filtered_accel[2]
        self.filtered_accel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveKpMahonyOrientationFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received KeyboardInterrupt. Stopping the node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
