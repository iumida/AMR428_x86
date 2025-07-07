import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MeshMarkerPublisher(Node):

    def __init__(self):
        super().__init__('mesh_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, '/avocado', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ""
        marker.type = Marker.MESH_RESOURCE
        marker.id = 0
        marker.action = Marker.ADD
        marker.mesh_resource = "https://你的網址/模型.dae"
        marker.mesh_use_embedded_materials = True

        marker.scale.x = 10.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = 3.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MeshMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
