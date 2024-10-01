import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import serial
from cobs import cobs
import struct
import sys
import threading
import time
import numpy as np
from synthevery_dev_tools.orientation import mahony
from synthevery_dev_tools.orientation.mahony import MahonyFilter
from synthevery_dev_tools.sensor.global_accel import GlobalAccel
from synthevery_dev_tools.feature.signal_features import LPF, Difference

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')

        # パラメータの宣言と取得
        self.declare_parameter('device', '/dev/ttyACM0')
        device = self.get_parameter('device').get_parameter_value().string_value

        # シリアルポートの初期化
        try:
            self.serial = serial.Serial(device, baudrate=460800)
            self.get_logger().info(f'シリアルデバイスをオープン: {device}')
        except serial.SerialException as e:
            self.get_logger().error(f'シリアルデバイス {device} のオープンに失敗: {e}')
            sys.exit(1)

        # パブリッシャーの作成
        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensor_data', 10)
        self.orientation_publisher_ = self.create_publisher(Float32MultiArray, 'orientation', 10)
        self.global_accel_publisher_ = self.create_publisher(Float32MultiArray, 'global_accel', 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, "~/pose", 10)

        # 姿勢推定フィルタの初期化 
        self.default_kp = 20.0
        self.high_accel_count = 0
        self.orientation_filter = MahonyFilter(self.default_kp, 0, 200)

        # スレッド制御用のイベント
        self._stop_event = threading.Event()

        self.mahony_kp_prev = self.default_kp

        self.latest_orientation = [0.0] * 7

        # 読み取りスレッドの開始
        self.read_thread = threading.Thread(target=self.read_serial_thread, daemon=True)
        self.read_thread.start()
        self.get_logger().info('シリアル読み取りスレッドを開始しました')

    def read_serial_thread(self):
        while not self._stop_event.is_set():
            try:
                # COBSフレームはゼロバイトで区切られていると仮定
                data = self.serial.read_until(b'\x00')
                if not data:
                    continue  # タイムアウトやデータなしの場合

                if data[-1] != 0:
                    self.get_logger().warn('ゼロバイトで終了していないデータを受信')
                    continue

                try:
                    cobs_data = cobs.decode(data[:-1])
                except cobs.DecodeError as e:
                    self.get_logger().error(f'COBSデコードエラー: {e}')
                    continue

                self.process_packet(cobs_data)
            except serial.SerialException as e:
                self.get_logger().error(f'シリアル読み取りエラー: {e}')
                time.sleep(1)  # エラー時に少し待機

    def process_packet(self, packet):
        expected_length = 6 + 12  # MACアドレス6バイト + センサーデータ18バイト (9フィールド × 2バイト)
        if len(packet) < expected_length:
            self.get_logger().warn(f'不完全なパケットを受信: {len(packet)} バイト')
            return

        # MACアドレスの抽出
        mac = packet[:6]
        mac_str = ':'.join(f'{b:02X}' for b in mac)

        # センサーデータの抽出
        sensor_bytes = packet[6:18]  # 12バイト
        # self.get_logger().info(f'Received sensor bytes: {" ".join(f"{b:02X}" for b in sensor_bytes)}')

        # センサーデータをリトルエンディアンの2バイト整数としてデシリアライズ
        if len(sensor_bytes) != 12:
            self.get_logger().warn(f'センサーデータの長さが不正です: {len(sensor_bytes)} バイト')
            return

        # フォーマット文字列：リトルエンディアンの9つのshort
        format_str = '<' + 'h' * 6  # '<'はリトルエンディアン, 'h'はshort (2バイト)
        try:
            values = list(struct.unpack(format_str, sensor_bytes))
        except struct.error as e:
            self.get_logger().error(f'センサーデータのデシリアライズに失敗: {e}')
            return

        # メッセージの準備とパブリッシュ
        msg = Float32MultiArray()

        # センサーデータのスケーリング
        # accel: 8, gyro: 2000
        arr = np.array(values, dtype=np.float64)
        arr[0:3] = arr[0:3] / 32768.0 * 8.0
        arr[3:6] = arr[3:6] / 32768.0 * 2000.0
        msg.data = arr.tolist()

        # 姿勢推定フィルタの更新
        mahony_kp = self.default_kp - abs(1 - np.linalg.norm(arr[0:3])) * self.default_kp * 3

        filter_coef = min(max(np.linalg.norm(arr[0:3]) / 2.0 * 0.1, 0.0), 0.09) + 0.9
        
        if (self.mahony_kp_prev < mahony_kp):
            mahony_kp = self.mahony_kp_prev * filter_coef + mahony_kp * (1 - filter_coef)
        else:
            mahony_kp = self.mahony_kp_prev * 0.5 + mahony_kp * 0.5

        mahony_kp = min(max(mahony_kp, 0.0), self.default_kp)
        self.mahony_kp_prev = mahony_kp

        self.orientation_filter.set_kp(mahony_kp)

        if (mahony_kp < 10.0):
            self.orientation_filter.set_kp(0.0)

        if (mahony_kp < 3.0):
            self.high_accel_count = min(max(self.high_accel_count + 1, 0), 25)
        else:
            self.high_accel_count = max(self.high_accel_count - 1, 0)

        self.orientation_filter.update_imu(
                arr[3],
                arr[4],
                -arr[5],
                arr[0],
                arr[1],
                -arr[2],
            )

        if (self.high_accel_count < 20 and mahony_kp > 1.5):
            self.latest_orientation = [
                self.orientation_filter.get_quaternion()[0],
                self.orientation_filter.get_quaternion()[1],
                self.orientation_filter.get_quaternion()[2],
                self.orientation_filter.get_quaternion()[3],
                self.orientation_filter.get_roll(),
                self.orientation_filter.get_pitch(),
                self.orientation_filter.get_yaw(),
            ]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.orientation.w = self.latest_orientation[0]
        pose_msg.pose.orientation.x = self.latest_orientation[1]
        pose_msg.pose.orientation.y = self.latest_orientation[2]
        pose_msg.pose.orientation.z = self.latest_orientation[3]
        self.pose_publisher_.publish(pose_msg)            

        # 姿勢推定フィルタの結果をパブリッシュ
        orientation_msg = Float32MultiArray()
        orientation_msg.data = [
            self.latest_orientation[4],
            self.latest_orientation[5],
            self.latest_orientation[6],
            mahony_kp,
            np.linalg.norm(arr[0:3]),
            float(self.high_accel_count)
        ]
        
        self.orientation_publisher_.publish(orientation_msg)

        global_accel = GlobalAccel.update(arr[0:3].tolist(), self.latest_orientation[4], self.latest_orientation[5])

        msg.data.extend(global_accel)
        msg.data.extend(orientation_msg.data)

        self.publisher_.publish(msg)
        self.get_logger().debug(f'MAC {mac_str} からデータをパブリッシュ: {msg.data}')

        global_accel_msg = Float32MultiArray()
        global_accel_msg.data = global_accel
        self.global_accel_publisher_.publish(global_accel_msg)

    def close_serial(self):
        self._stop_event.set()
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
            if self.read_thread.is_alive():
                self.get_logger().warn('シリアル読み取りスレッドが終了しませんでした')

        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
            self.get_logger().info('シリアルデバイスをクローズしました')

def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterruptを受信しました。ノードを停止します。')
    finally:
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
