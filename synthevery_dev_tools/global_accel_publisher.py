import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped

from synthevery_dev_tools.orientation.global_accel import GlobalAccel
from synthevery_dev_tools.orientation.util import quaternion_to_euler

class GlobalAccelPublisher(Node):
    def __init__(self):
        super().__init__("global_accel_publisher")

        self.frame_id = self.declare_parameter("frame_id", "world").get_parameter_value().string_value

        self.global_accel_publisher = self.create_publisher(Vector3Stamped, "~/global_accel", 10)

        self.accel_subscriber = Subscriber(self, Vector3Stamped, "~/accel")
        self.orientation_subscriber = Subscriber(self, QuaternionStamped, "~/orientation")
        
        self.time_synchronizer = ApproximateTimeSynchronizer(
            [self.accel_subscriber, self.orientation_subscriber], 
            queue_size=10, 
            slop=self.declare_parameter("slop", 0.05).get_parameter_value().double_value
        )
        self.time_synchronizer.registerCallback(self.sensor_callback)

    def sensor_callback(self, accel, orientation):
        self.update_global_accel(accel, orientation)

        latest_time = max(accel.header.stamp, orientation.header.stamp, key=lambda t: (t.sec, t.nanosec))

        self.publish_global_accel(latest_time)

    def update_global_accel(self, accel, orientation):
        self.global_accel = self.calc_global_accel(accel, orientation)

    def calc_global_accel(self, accel, orientation):
        roll, pitch, yaw = quaternion_to_euler(
            orientation.quaternion.w, 
            orientation.quaternion.x, 
            orientation.quaternion.y, 
            orientation.quaternion.z
        )
        return GlobalAccel.update([accel.vector.x, accel.vector.y, accel.vector.z], roll, pitch)

    def publish_global_accel(self, time):
        msg = Vector3Stamped()
        msg.header.stamp = time
        msg.header.frame_id = self.frame_id
        msg.vector.x = self.global_accel[0]
        msg.vector.y = self.global_accel[1]
        msg.vector.z = self.global_accel[2]
        self.global_accel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalAccelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("received KeyboardInterrupt, stopping node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()