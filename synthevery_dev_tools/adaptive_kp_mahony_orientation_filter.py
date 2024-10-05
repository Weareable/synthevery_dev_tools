import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped

import numpy as np

from synthevery_dev_tools.orientation.mahony import MahonyFilter



class AdaptiveKpMahonyOrientationFilter(Node):
    def __init__(self):
        super().__init__('akp_mahony')

        self.default_kp = self.declare_parameter("kp", 20.0).get_parameter_value().double_value
        self.default_ki = self.declare_parameter("ki", 0.0).get_parameter_value().double_value
        self.filter_rate = self.declare_parameter("filter_rate", 200.0).get_parameter_value().double_value

        self.orientation_filter = MahonyFilter(self.default_kp, self.default_ki, self.filter_rate)
        self.latest_orientation = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.kp_prev = 0.0
        self.kp = 0.0
        self.high_accel_count = 0
        self.accel_norm = 0.0
        self.kp_lpf_coef = 0.0

        self.accel_subscriber = Subscriber(self, Vector3Stamped, "~/accel")
        self.gyro_subscriber = Subscriber(self, Vector3Stamped, "~/gyro")
        self.time_synchronizer = ApproximateTimeSynchronizer(
            [self.accel_subscriber, self.gyro_subscriber], 
            queue_size=10, 
            slop=1.0 / self.filter_rate
        )
        self.time_synchronizer.registerCallback(self.sensor_callback)

        self.frame_id = self.declare_parameter("frame_id", "world").get_parameter_value().string_value

        self.orientation_publisher = self.create_publisher(QuaternionStamped, "~/orientation", 10)
        self.orientation_rpy_publisher = self.create_publisher(Vector3Stamped, "~/orientation_rpy", 10)
        self.raw_orientation_publisher = self.create_publisher(QuaternionStamped, "~/raw_orientation", 10)
        self.raw_orientation_rpy_publisher = self.create_publisher(Vector3Stamped, "~/raw_orientation_rpy", 10)

        self.kp_publisher = self.create_publisher(Float32, "~/kp", 10)
        self.accel_norm_publisher = self.create_publisher(Float32, "~/accel_norm", 10)
        self.high_accel_count_publisher = self.create_publisher(Float32, "~/high_accel_count", 10)
        self.kp_lpf_coef_publisher = self.create_publisher(Float32, "~/kp_lpf_coef", 10)

    def sensor_callback(self, accel, gyro):
        self.update_filter(accel, gyro)

        latest_time = max(accel.header.stamp, gyro.header.stamp, key=lambda t: (t.sec, t.nanosec))
        self.publish_orientation(latest_time)
        self.publish_filter_params()

    def update_filter(self, accel, gyro):
        # 姿勢推定フィルタの更新
        self.accel_norm = np.linalg.norm(np.array([accel.vector.x, accel.vector.y, accel.vector.z]))
        self.kp = self.default_kp - abs(1 - self.accel_norm) * self.default_kp * 3

        self.kp_lpf_coef = min(max(self.accel_norm / 2.0 * 0.1, 0.0), 0.09) + 0.9
        
        if (self.kp_prev < self.kp):
            self.kp = self.kp_prev * self.kp_lpf_coef + self.kp * (1 - self.kp_lpf_coef)
        else:
            self.kp = self.kp_prev * 0.5 + self.kp * 0.5

        self.kp = min(max(self.kp, 0.0), self.default_kp)
        self.kp_prev = self.kp

        self.orientation_filter.set_kp(self.kp)

        if (self.kp < 10.0):
            self.orientation_filter.set_kp(0.0)

        if (self.kp < 3.0):
            self.high_accel_count = min(max(self.high_accel_count + 1, 0), 25)
        else:
            self.high_accel_count = max(self.high_accel_count - 1, 0)

        self.orientation_filter.update_imu(
                gyro.vector.x,
                gyro.vector.y,
                -gyro.vector.z,
                accel.vector.x,
                accel.vector.y,
                -accel.vector.z,
            )
        
        if (self.high_accel_count < 20 and self.kp > 1.5):
            self.latest_orientation = [
                self.orientation_filter.get_quaternion()[0],
                self.orientation_filter.get_quaternion()[1],
                self.orientation_filter.get_quaternion()[2],
                self.orientation_filter.get_quaternion()[3],
                self.orientation_filter.get_roll(),
                self.orientation_filter.get_pitch(),
                self.orientation_filter.get_yaw(),
            ]

    def publish_orientation(self, time):
        msg = QuaternionStamped()
        msg.header.stamp = time
        msg.header.frame_id = self.frame_id
        msg.quaternion.w = self.latest_orientation[0]
        msg.quaternion.x = self.latest_orientation[1]
        msg.quaternion.y = self.latest_orientation[2]
        msg.quaternion.z = self.latest_orientation[3]
        self.orientation_publisher.publish(msg)

        msg = Vector3Stamped()
        msg.header.stamp = time
        msg.header.frame_id = self.frame_id
        msg.vector.x = self.latest_orientation[4]
        msg.vector.y = self.latest_orientation[5]
        msg.vector.z = self.latest_orientation[6]
        self.orientation_rpy_publisher.publish(msg)

        msg = QuaternionStamped()
        msg.header.stamp = time
        msg.header.frame_id = self.frame_id
        msg.quaternion.w = self.orientation_filter.get_quaternion()[0]
        msg.quaternion.x = self.orientation_filter.get_quaternion()[1]
        msg.quaternion.y = self.orientation_filter.get_quaternion()[2]
        msg.quaternion.z = self.orientation_filter.get_quaternion()[3]
        self.raw_orientation_publisher.publish(msg)

        msg = Vector3Stamped()
        msg.header.stamp = time
        msg.header.frame_id = self.frame_id
        msg.vector.x = self.orientation_filter.get_roll()
        msg.vector.y = self.orientation_filter.get_pitch()
        msg.vector.z = self.orientation_filter.get_yaw()
        self.raw_orientation_rpy_publisher.publish(msg)

    def publish_filter_params(self):
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
