import datetime

from blinker import Signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped, Vector3
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from synthevery_dev_tools.feature.signal_features import BackwardDifferenceLPF, Difference, Peak
from synthevery_dev_tools.fsm.fsm import FSM, State, Transition, FSMStopwatch ,FSMTimer
from synthevery_dev_tools.sensor.sensor_data import SensorData


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.init_features()
        self.init_fsm()

        self.accel = Vector3()
        self.gyro = Vector3()
        self.orientation_rpy = Vector3()
        self.global_accel = Vector3()

        self.accel_subscriber = Subscriber(self, Vector3Stamped, "~/accel")
        self.gyro_subscriber = Subscriber(self, Vector3Stamped, "~/gyro")
        self.orientation_rpy_subscriber = Subscriber(self, Vector3Stamped, "~/orientation_rpy")
        self.global_accel_subscriber = Subscriber(self, Vector3Stamped, "~/global_accel")
        self.time_synchronizer = ApproximateTimeSynchronizer(
            [self.accel_subscriber, self.gyro_subscriber, self.orientation_rpy_subscriber, self.global_accel_subscriber], 
            queue_size=10, 
            slop=0.05
        )
        self.time_synchronizer.registerCallback(self.sensor_callback)
        

        self.feature_publisher = self.create_publisher(Float32MultiArray, "~/feature", 10)
        self.tap_publisher = self.create_publisher(Float32MultiArray, "~/tap", 10)

    def sensor_callback(self, accel, gyro, orientation_rpy, global_accel):
        self.accel = accel.vector
        self.gyro = gyro.vector
        self.orientation_rpy = orientation_rpy.vector
        self.global_accel = global_accel.vector

        self.update_features()
        self.fsm.update()


    def init_features(self):
        self.global_accel_z_peak = Peak(15)
        self.global_accel_z_lpf = BackwardDifferenceLPF(0.7)
        self.global_accel_z_difference = Difference()
        self.global_accel_z_min_peak_difference = Difference()
        self.gyro_x_peak = Peak(15)
        self.gyro_y_peak = Peak(8)
        self.gyro_z_peak = Peak(8)

    def update_features(self):
        self.global_accel_z_peak.update(self.global_accel.z)
        self.global_accel_z_lpf.update(self.global_accel.z)
        self.global_accel_z_difference.update(self.global_accel_z_lpf.value(), datetime.datetime.now().timestamp())
        self.global_accel_z_min_peak_difference.update(self.global_accel_z_peak.min_peak(), datetime.datetime.now().timestamp())
        self.gyro_x_peak.update(self.gyro.x)
        self.gyro_y_peak.update(self.gyro.y)
        self.gyro_z_peak.update(self.gyro.z)

        feature_msg = Float32MultiArray()
        feature_msg.data = [
            self.global_accel_z_peak.max_peak(),
            self.global_accel_z_peak.min_peak(),
            self.global_accel_z_peak.peak_to_peak(),
            self.global_accel_z_min_peak_difference.difference(),
            self.global_accel_z_lpf.value(),
            self.global_accel_z_difference.difference()
        ]
        self.feature_publisher.publish(feature_msg)

    def init_fsm(self):
        def logger(message) -> None:
            self.get_logger().info(message)

        self.fsm = FSM(logger=logger)

        self.tap_state_idle = State("Idle")
        self.tap_state_tapping = State("Tapping")
        self.tap_state_tapped = State("Tapped")
        self.tap_state_measuring = State("Measuring")

        self.tap_state_idle_stopwatch = FSMStopwatch()
        self.tap_state_idle_stopwatch.bind(self.tap_state_idle)

        self.tap_state_tapping_stopwatch = FSMStopwatch()
        self.tap_state_tapping_stopwatch.bind(self.tap_state_tapping)

        self.tap_state_tapped_stopwatch = FSMStopwatch()
        self.tap_state_tapped_stopwatch.bind(self.tap_state_tapped)

        self.tap_idle_to_tapping_transition = Transition(lambda: (
            self.global_accel.z > 0.4 and 
            self.global_accel_z_difference.difference() < -0.01
        ))

        self.tap_tapping_to_tapped_transition = Transition(lambda: (
            self.global_accel.z < -0.6 and
            self.global_accel_z_difference.difference() > 0.0
        ))
        self.tap_tapping_to_tapped_by_shock_transition = Transition(lambda: self.global_accel_z_difference.difference() < -2)

        self.fsm.add_state(self.tap_state_idle)
        self.fsm.add_state(self.tap_state_tapping)
        self.fsm.add_state(self.tap_state_tapped)
        self.fsm.add_transition(self.tap_state_idle, self.tap_state_tapping, self.tap_idle_to_tapping_transition)
        self.fsm.add_transition(self.tap_state_tapping, self.tap_state_tapping, Transition(lambda: self.global_accel.z > 0.6))
        self.fsm.add_transition(self.tap_state_tapping, self.tap_state_tapped, self.tap_tapping_to_tapped_transition)
        self.fsm.add_transition(self.tap_state_tapping, self.tap_state_tapped, self.tap_tapping_to_tapped_by_shock_transition)

        self.tap_measuring_timer = FSMTimer(5)
        self.tap_measuring_timer.bind(self.tap_state_tapped)
        self.tap_measuring_timer.on_timeout().connect(self.on_tap, weak=False)

        self.tap_state_tapped.on_enter().connect(self.on_tap, weak=False)

        # timeout: tapping -> idle
        self.fsm.add_transition(self.tap_state_tapping, self.tap_state_idle, Transition(lambda: self.tap_state_tapping_stopwatch.get_elapsed_ms() > 100 and self.global_accel.z < 0.3 and self.global_accel_z_difference.difference() > -0.1))
        self.fsm.add_transition(self.tap_state_tapping, self.tap_state_idle, Transition(lambda: self.tap_state_tapping_stopwatch.get_elapsed_ms() > 350))

        # timeout: tapped -> idle
        self.fsm.add_transition(self.tap_state_tapped, self.tap_state_idle, Transition(lambda: self.tap_state_tapped_stopwatch.get_elapsed_ms() > 50))

        self.fsm.transition_to(self.tap_state_idle)
        
    def on_tap(self, sender):
        self.get_logger().warn("TAP")
        tap_msg = Float32MultiArray()
        tap_msg.data = [self.global_accel_z_peak.peak_to_peak(), self.orientation_rpy.x, self.orientation_rpy.y, self.orientation_rpy.z]
        self.tap_publisher.publish(tap_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

