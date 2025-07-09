import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from raya_status_msgs.msg import ComponentStatus
from std_msgs.msg import String  # for debugging/logs

class CameraMonitor(Node):
    def __init__(self):
        super().__init__('camera_monitor')

        # Declare and get camera name from parameter
        self.declare_parameter('camera_name', 'nav_top')
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        self.get_logger().info(f"Starting camera monitor for: {self.camera_name}")

        # Topic names
        base = f"/gary/cameras/{self.camera_name}"
        self.status_topic = f"{base}/status"
        self.color_image_topic = f"{base}/color/image_raw/compressed"
        self.depth_image_topic = f"{base}/aligned_depth_to_color/image_raw"
        self.pointcloud_topic = f"{base}/points"

        # Subscriptions
        self.create_subscription(ComponentStatus, self.status_topic, self.status_cb, 10)
        self.create_subscription(CompressedImage, self.color_image_topic, self.color_image_cb, 10)
        self.create_subscription(Image, self.depth_image_topic, self.depth_cb, 10)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_cb, 10)

    def status_cb(self, msg: ComponentStatus):
        self.get_logger().info(f"[STATUS] {self.camera_name}: ok={msg.ok}, msg={msg.message}")

    def color_image_cb(self, msg: CompressedImage):
        self.get_logger().info(f"[COLOR] {self.camera_name}: size={len(msg.data)} bytes")

    def depth_cb(self, msg: Image):
        self.get_logger().info(f"[DEPTH] {self.camera_name}: resolution={msg.width}x{msg.height}")

    def pointcloud_cb(self, msg: PointCloud2):
        self.get_logger().info(f"[POINTCLOUD] {self.camera_name}: width={msg.width}, height={msg.height}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
