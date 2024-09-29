import datetime

from blinker import Signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from synthevery_dev_tools.feature.signal_features import LPF, Difference, Peak
from synthevery_dev_tools.fsm.fsm import FSM, State, Transition, FSMStopwatch ,FSMTimer
from synthevery_dev_tools.sensor.sensor_data import SensorData


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.init_features()
        self.init_fsm()
        self.sensor_subscriber = self.create_subscription(Float32MultiArray, "/sensor_data", self.sensor_callback, 10)
        self.sensor_data = SensorData()

    def init_features(self):
        self.global_accel_z_peak = Peak(8)
        self.global_accel_z_lpf = LPF(0.7)
        self.global_accel_z_difference = Difference()
        self.gyro_x_peak = Peak(15)
        self.gyro_y_peak = Peak(8)
        self.gyro_z_peak = Peak(8)

    def update_features(self):
        self.global_accel_z_peak.update(self.sensor_data.accel.z)
        self.global_accel_z_lpf.update(self.sensor_data.accel.z)
        self.global_accel_z_difference.update(self.global_accel_z_lpf.value(), datetime.datetime.now().timestamp())
        self.gyro_x_peak.update(self.sensor_data.gyro.x)
        self.gyro_y_peak.update(self.sensor_data.gyro.y)
        self.gyro_z_peak.update(self.sensor_data.gyro.z)

    def init_fsm(self):
        self.fsm = FSM()

        self.tap_state_idle = State()
        self.tap_state_tapping = State()

        self.tap_state_idle_stopwatch = FSMStopwatch()
        self.tap_state_idle_stopwatch.bind(self.tap_state_idle)

        self.tap_state_tapping_timer = FSMTimer(160)
        self.tap_state_tapping_timer.bind(self.tap_state_tapping)

        self.tr1 = lambda: (
            self.sensor_data.accel.z < -0.8 and 
            self.tap_state_idle_stopwatch.get_elapsed_ms() > 90
        )

        self.tr2 = lambda: (
            self.global_accel_z_difference.difference() > 0
        )

        self.tap_idle_to_tapping_transition = Transition(self.tr1)
        self.tap_tapping_to_idle_transition = Transition(self.tr2)

        self.tap_idle_to_tapping_transition.on_transition().connect(lambda sender: self.get_logger().info("idle >> tapping"), weak=False)

        self.tap_tapping_to_idle_transition.on_transition().connect(self.on_tap, weak=False)
        self.tap_tapping_to_idle_transition.on_transition().connect(lambda sender: self.get_logger().info("tapping >> idle"), weak=False)

        self.fsm.add_state(self.tap_state_idle)
        self.fsm.add_state(self.tap_state_tapping)

        self.fsm.add_transition(self.tap_state_idle, self.tap_state_tapping, self.tap_idle_to_tapping_transition)
        self.fsm.add_transition(self.tap_state_tapping, self.tap_state_idle, self.tap_tapping_to_idle_transition)


        self.fsm.transition_to(self.tap_state_idle)
        
    def on_tap(self, sender):
        self.get_logger().info("TAP")

    def sensor_callback(self, msg: Float32MultiArray):
        self.sensor_data.update(msg)
        self.update_features()
        self.fsm.update()

def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

