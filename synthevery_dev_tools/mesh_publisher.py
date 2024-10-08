import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import os

class MeshPublisher(Node):
    def __init__(self):
        super().__init__('mesh_publisher')
        # メッシュパブリッシャーの作成(プライベート)
        self.publisher_ = self.create_publisher(Marker, "~/visualization_marker", 10)
        
        # Poseトピックをサブスクライブ
        self.subscription = self.create_subscription(
            PoseStamped,
            "~/target_pose",  # サブスクライブするトピック名を指定
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # 初期マーカーの設定
        self.marker = Marker()
        self.marker.header.frame_id = "map"  # 使用する座標フレーム
        self.marker.ns = "synthevery"
        self.marker.id = 0
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.action = Marker.ADD

        # メッシュのリソースパスを指定（例としてパッケージ内のファイルを使用）
        self.marker.mesh_resource = "package://synthevery_dev_tools/meshes/synthevery.dae"
        self.marker.mesh_use_embedded_materials = True  # メッシュに埋め込まれたマテリアルを使用

        # 位置と向きを初期化
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # スケールを設定
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0

        # 色を設定（メッシュに色が埋め込まれている場合は不要）
        self.marker.color.a = 1.0  # 不透明
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0

    def pose_callback(self, msg: PoseStamped):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.pose = msg.pose  # 受信したPoseでマーカーのポーズを更新
        self.publisher_.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    mesh_publisher = MeshPublisher()
    rclpy.spin(mesh_publisher)
    mesh_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
