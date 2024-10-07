import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MeshPublisher(Node):
    def __init__(self):
        super().__init__('mesh_publisher')
        # メッシュパブリッシャーの作成(プライベート)
        self.publisher = self.create_publisher(Marker, "~/visualization_marker", 10)

        # 初期マーカーの設定
        self.marker = Marker()
        self.marker.header.frame_id = self.declare_parameter("frame_id", "world").get_parameter_value().string_value
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
        self.marker.scale.x = self.declare_parameter("scale", 10.0).get_parameter_value().double_value
        self.marker.scale.y = self.marker.scale.x
        self.marker.scale.z = self.marker.scale.x

        # 色を設定（メッシュに色が埋め込まれている場合は不要）
        self.marker.color.a = self.declare_parameter("color_alpha", 1.0).get_parameter_value().double_value  # 不透明
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0

        self.timer = self.create_timer(1.0 / self.declare_parameter("publish_rate", 100.0).get_parameter_value().double_value, self.timer_callback)

    def timer_callback(self):
        self.publisher.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    mesh_publisher = MeshPublisher()
    rclpy.spin(mesh_publisher)
    mesh_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
