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
        self.orientation_publisher_ = self.create_publisher(Float32MultiArray, 'orientation_data', 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, "~/pose", 10)

        # 姿勢推定フィルタの初期化 
        self.default_kp = 30
        self.orientation_filter = MahonyFilter(self.default_kp, 0, 200)

        # スレッド制御用のイベント
        self._stop_event = threading.Event()

        self.accel_norm_lpf = LPF(0.1)
        self.accel_norm_lpf_diff = Difference()

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
        expected_length = 6 + 18  # MACアドレス6バイト + センサーデータ18バイト (9フィールド × 2バイト)
        if len(packet) < expected_length:
            self.get_logger().warn(f'不完全なパケットを受信: {len(packet)} バイト')
            return

        # MACアドレスの抽出
        mac = packet[:6]
        mac_str = ':'.join(f'{b:02X}' for b in mac)

        # センサーデータの抽出
        sensor_bytes = packet[6:24]  # 18バイト
        # self.get_logger().info(f'Received sensor bytes: {" ".join(f"{b:02X}" for b in sensor_bytes)}')

        # センサーデータをリトルエンディアンの2バイト整数としてデシリアライズ
        if len(sensor_bytes) != 18:
            self.get_logger().warn(f'センサーデータの長さが不正です: {len(sensor_bytes)} バイト')
            return

        # フォーマット文字列：リトルエンディアンの9つのshort
        format_str = '<' + 'h' * 9  # '<'はリトルエンディアン, 'h'はshort (2バイト)
        try:
            values = list(struct.unpack(format_str, sensor_bytes))
        except struct.error as e:
            self.get_logger().error(f'センサーデータのデシリアライズに失敗: {e}')
            return

        # メッセージの準備とパブリッシュ
        msg = Float32MultiArray()

        # センサーデータのスケーリング
        # accel: 10, gyro: 2000, angle: 360
        arr = np.array(values, dtype=np.float64)
        arr[0:3] = arr[0:3] / 32768.0 * 10.0
        arr[3:6] = arr[3:6] / 32768.0 * 2000.0
        arr[6:9] = arr[6:9] / 32768.0 * 360.0
        msg.data = arr.tolist()

        # 姿勢推定フィルタの更新

        self.accel_norm_lpf.update(np.linalg.norm(arr[0:3]))
        self.accel_norm_lpf_diff.update(self.accel_norm_lpf.value(), datetime.datetime.now().timestamp())

        mahony_kp = self.default_kp - min(max(abs(1 - np.linalg.norm(arr[0:3])) * self.default_kp * 3, 0), self.default_kp * 0.99)
        #mahony_kp = self.default_kp - min(max(abs(self.accel_norm_lpf_diff.difference()) * self.default_kp * 10, 0), self.default_kp * 0.99)

        self.orientation_filter.set_kp(mahony_kp)

        self.orientation_filter.update_imu(
            arr[3],
            arr[4],
            -arr[5],
            arr[0],
            arr[1],
            -arr[2],
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.orientation.w = self.orientation_filter.get_quaternion()[0]
        pose_msg.pose.orientation.x = self.orientation_filter.get_quaternion()[1]
        pose_msg.pose.orientation.y = self.orientation_filter.get_quaternion()[2]
        pose_msg.pose.orientation.z = self.orientation_filter.get_quaternion()[3]
        self.pose_publisher_.publish(pose_msg)

        global_accel = GlobalAccel.update(arr[0:3].tolist(), self.orientation_filter.get_roll(), self.orientation_filter.get_pitch())
        msg.data.extend(global_accel)

        self.publisher_.publish(msg)
        self.get_logger().debug(f'MAC {mac_str} からデータをパブリッシュ: {msg.data}')

        # 姿勢推定フィルタの結果をパブリッシュ
        orientation_msg = Float32MultiArray()
        orientation_msg.data = [
            self.orientation_filter.get_roll(),
            self.orientation_filter.get_pitch(),
            self.orientation_filter.get_yaw(),
            mahony_kp,
        ]
        self.orientation_publisher_.publish(orientation_msg)

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
