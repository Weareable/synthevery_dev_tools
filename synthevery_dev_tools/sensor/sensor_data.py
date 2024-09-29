from std_msgs.msg import Float32MultiArray
import numpy as np

class AccelData:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class GyroData:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class OrientationData:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

class SensorData:
    def __init__(self):
        self.accel = AccelData()
        self.gyro = GyroData()
        self.orientation = OrientationData()

    def update(self, msg: Float32MultiArray):
        self.accel.x = float(msg.data[0])
        self.accel.y = float(msg.data[1])
        self.accel.z = float(msg.data[2])
        self.gyro.x = float(msg.data[3])
        self.gyro.y = float(msg.data[4])
        self.gyro.z = float(msg.data[5])
        self.orientation.roll = float(msg.data[6])
        self.orientation.pitch = float(msg.data[7])
        self.orientation.yaw = float(msg.data[8])
        