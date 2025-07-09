import rclpy
import json
from rclpy.node import Node
import asyncio

# Service and message types
from raya_status_msgs.msg import ComponentStatus
from gary_cv_msgs.srv import GetAvailableModels, EnableModel, GetActiveModels
from apriltag_msgs.msg import AprilTagDetectionArray
from detection_msgs.msg import Detection2DArray

class CVEngineClient(Node):
    def __init__(self):
        super().__init__('cv_engine_client')

        # Subscriptions
        self.create_subscription(ComponentStatus, '/gary/cv/status', self.status_cb, 10)
        self.create_subscription(AprilTagDetectionArray, '/gary/cv/detector/tag/detections_101', self.apriltag_cb, 10)
        self.create_subscription(Detection2DArray, '/gary/cv/detector/object/detections_1', self.object_cb, 10)
        self.create_subscription(Detection2DArray, '/gary/cv/detector/face/detections_1', self.face_cb, 10)

        # Service clients
        self.get_available_client = self.create_client(GetAvailableModels, '/gary/cv/get_available_models')
        self.enable_model_client = self.create_client(EnableModel, '/gary/cv/enable_model')
        self.get_active_client = self.create_client(GetActiveModels, '/gary/cv/get_active_models')

        self.get_logger().info("CV Engine Client Node started")

        # Automatically run on startup for demo
        self.timer = self.create_timer(3.0, self.demo_sequence)

    def status_cb(self, msg: ComponentStatus):
        self.get_logger().info(f"[STATUS] code={msg.status} states={msg.states} info={msg.extra_info}")

    def apriltag_cb(self, msg: AprilTagDetectionArray):
        if msg.detections:
            det = msg.detections[0]
            self.get_logger().info(f"[APRILTAG] id={det.id}, family={det.family}, center=({det.center.x:.1f},{det.center.y:.1f})")

    def object_cb(self, msg: Detection2DArray):
        if msg.detections:
            det = msg.detections[0]
            self.get_logger().info(f"[YOLO] object={det.results[0].hypothesis.class_id} confidence={det.results[0].hypothesis.score:.2f}")

    def face_cb(self, msg: Detection2DArray):
        if msg.detections:
            det = msg.detections[0]
            self.get_logger().info(f"[FACE] detection - confidence={det.results[0].hypothesis.score:.2f}")

    async def demo_sequence(self):
        self.timer.cancel()

        # 1. Get Available Models
        await self.get_available_models()

        # 2. Enable AprilTags
        await self.enable_model("apriltags_cpp", "nav_bottom", {
            "families": ["tag36h11"],
            "tag_size": 0.173
        })

        # 3. Enable Yolov8
        await self.enable_model("yolov8s_coco", "nav_bottom", {"depth": True})

        # 4. Enable Face Detection
        await self.enable_model("yunet_face", "nav_bottom", {})

        # 5. Get Active Models
        await self.get_active_models()

    async def get_available_models(self):
        if not self.get_available_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service get_available_models not available")
            return
        req = GetAvailableModels.Request()
        future = self.get_available_client.call_async(req)
        await future
        res = future.result()
        self.get_logger().info(f"Available Models: {res.available_models}")

    async def enable_model(self, name, source, params):
        if not self.enable_model_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service enable_model not available")
            return
        req = EnableModel.Request()
        req.name = name
        req.source = source
        req.params = json.dumps(params)
        future = self.enable_model_client.call_async(req)
        await future
        res = future.result()
        if res.error_code == 0:
            self.get_logger().info(f"Enabled {name} from {source} → topic: {res.detections_topic}")
        else:
            self.get_logger().error(f"Failed to enable model: {res.error_msg}")

    async def get_active_models(self):
        if not self.get_active_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service get_active_models not available")
            return
        req = GetActiveModels.Request()
        future = self.get_active_client.call_async(req)
        await future
        res = future.result()
        self.get_logger().info(f"Active Models: {res.active_models}")


def main(args=None):
    rclpy.init(args=args)
    node = CVEngineClient()

    # Run asyncio coroutine in a separate executor
    loop = asyncio.get_event_loop()
    loop.create_task(node.demo_sequence())
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
