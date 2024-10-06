import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import serial
from cobs import cobs
import struct
import sys
import threading
import time
import numpy as np
import json


class SyntheverySerialBridge(Node):
    def __init__(self):
        super().__init__('synthevery_serial_bridge')

        # パラメータの宣言と取得
        self.declare_parameter('device', '/dev/ttyACM0')
        device = self.get_parameter('device').get_parameter_value().string_value
        self.config_file = self.declare_parameter('config_file', '').get_parameter_value().string_value

        # シリアルポートの初期化
        try:
            self.serial = serial.Serial(device, baudrate=460800)
            self.get_logger().info(f'シリアルデバイスをオープン: {device}')
        except serial.SerialException as e:
            self.get_logger().error(f'シリアルデバイス {device} のオープンに失敗: {e}')
            sys.exit(1)

        # デバイスリストの読み込み
        self.device_list = {}
        try:
            with open(self.config_file, 'r') as f:
                self.device_list = json.load(f)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'デバイスリストの読み込みに失敗: {e}')
        except FileNotFoundError as e:
            self.get_logger().error(f'デバイスリストのファイルが見つかりません: {e}')

        self.accel_publishers = {}
        self.gyro_publishers = {}
        self.orientation_publishers = {}
        self.filtered_accel_publishers = {}

        self.get_logger().info(f'デバイスリスト:')
        for mac_str, device_info in self.device_list.items():
            self.get_logger().info(f'- {mac_str}')
            # パブリッシャーの作成
            self.accel_publishers[mac_str] = self.create_publisher(Vector3Stamped, f'~/{device_info["device_name"]}/accel', 10)
            self.gyro_publishers[mac_str] = self.create_publisher(Vector3Stamped, f'~/{device_info["device_name"]}/gyro', 10)
            self.orientation_publishers[mac_str] = self.create_publisher(Vector3Stamped, f'~/{device_info["device_name"]}/orientation_rpy', 10)
            self.filtered_accel_publishers[mac_str] = self.create_publisher(Vector3Stamped, f'~/{device_info["device_name"]}/filtered_accel', 10)

        # スレッド制御用のイベント
        self._stop_event = threading.Event()

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
        timestamp = self.get_clock().now().to_msg()

        expected_length = 6  # MACアドレス6バイト
        if len(packet) < expected_length:
            self.get_logger().warn(f'不完全なパケットを受信: {len(packet)} バイト')
            return

        # MACアドレスの抽出
        mac = packet[:6][::-1]
        mac_str = ''.join(f'{b:02X}' for b in mac)

        if self.device_list.get(mac_str) is None:
            self.get_logger().warn(f'リストにないデバイスからのデータを受信: {mac_str}', throttle_duration_sec=1.0)
            return

        #self.get_logger().info(f'Received packet: {mac_str}')

        # センサーデータの抽出
        sensor_bytes = packet[6:]  # 18バイト
        data_num = len(sensor_bytes) // 2
        # self.get_logger().info(f'Received sensor bytes: {" ".join(f"{b:02X}" for b in sensor_bytes)}')

        # センサーデータをリトルエンディアンの2バイト整数としてデシリアライズ
        # フォーマット文字列：リトルエンディアンの9つのshort
        format_str = '<' + 'h' * data_num  # '<'はリトルエンディアン, 'h'はshort (2バイト)
        try:
            values = list(struct.unpack(format_str, sensor_bytes))
        except struct.error as e:
            self.get_logger().error(f'センサーデータのデシリアライズに失敗: {e}')
            return

        # センサーデータのスケーリング
        # accel: 8, gyro: 2000, orientation: 360
        arr = np.array(values, dtype=np.float64)

        # 加速度データのパブリッシュ
        if data_num < 3:
            return
        arr[0:3] = arr[0:3] / 32768.0 * 8.0
        accel_msg = Vector3Stamped()
        accel_msg.header.stamp = timestamp
        accel_msg.header.frame_id = self.device_list[mac_str]["frame_id"]
        accel_msg.vector.x = arr[0]
        accel_msg.vector.y = arr[1]
        accel_msg.vector.z = arr[2]
        self.accel_publishers[mac_str].publish(accel_msg)

        # ジャイロデータのパブリッシュ
        if data_num < 6:
            return
        arr[3:6] = arr[3:6] / 32768.0 * 2000.0
        gyro_msg = Vector3Stamped()
        gyro_msg.header.stamp = timestamp
        gyro_msg.header.frame_id = self.device_list[mac_str]["frame_id"]
        gyro_msg.vector.x = arr[3]
        gyro_msg.vector.y = arr[4]
        gyro_msg.vector.z = arr[5]
        self.gyro_publishers[mac_str].publish(gyro_msg)

        # 姿勢データのパブリッシュ
        if data_num < 9:
            return
        arr[6:9] = arr[6:9] / 32768.0 * 360.0
        orientation_msg = Vector3Stamped()
        orientation_msg.header.stamp = timestamp
        orientation_msg.header.frame_id = self.device_list[mac_str]["frame_id"]
        orientation_msg.vector.x = arr[6]
        orientation_msg.vector.y = arr[7]
        orientation_msg.vector.z = arr[8]
        self.orientation_publishers[mac_str].publish(orientation_msg)
        
        # フィルタリング加速度データのパブリッシュ
        if data_num < 12:
            return
        arr[9:12] = arr[9:12] / 32768.0 * 8.0
        filtered_accel_msg = Vector3Stamped()
        filtered_accel_msg.header.stamp = timestamp
        filtered_accel_msg.header.frame_id = self.device_list[mac_str]["frame_id"]
        filtered_accel_msg.vector.x = arr[9]
        filtered_accel_msg.vector.y = arr[10]
        filtered_accel_msg.vector.z = arr[11]
        self.filtered_accel_publishers[mac_str].publish(filtered_accel_msg)

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
    node = SyntheverySerialBridge()
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
