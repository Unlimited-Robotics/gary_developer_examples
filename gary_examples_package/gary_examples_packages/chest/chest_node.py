import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from raya_status_msgs.msg import ComponentStatus
from raya_chest_web_msgs.srv import ChestWebFinishCommand
from raya_chest_web_msgs.action import ChestWebCommand

from rclpy.action import ActionClient
import json


class ChestWebTester(Node):
    def __init__(self):
        super().__init__('chest_web_tester')

        # Status subscription
        self.create_subscription(ComponentStatus, '/raya/chest_web/status', self.status_cb, 10)

        # Debug publisher
        self.debug_pub = self.create_publisher(String, '/raya/chest_web/debug_msg', 10)

        # Action client for UI commands
        self.command_client = ActionClient(self, ChestWebCommand, '/raya/chest_web/command')

        # Service client to finish components
        self.finish_client = self.create_client(ChestWebFinishCommand, '/raya/chest_web/finish_command')

        self.get_logger().info("ChestWeb Tester Initialized")

        # Run test sequence
        self.create_timer(2.0, self.run_demo)

    def status_cb(self, msg: ComponentStatus):
        self.get_logger().info(f"[STATUS] code={msg.status} | state={msg.states} | info={msg.extra_info}")

    def publish_debug(self, html_text: str):
        msg = String()
        msg.data = html_text
        self.debug_pub.publish(msg)
        self.get_logger().info(f"[DEBUG] Published: {html_text}")

    async def run_demo(self):
        self.get_logger().info("Running ChestWeb demo...")

        # Wait for action server
        await self.command_client.wait_for_server()

        # Show animation
        await self.send_ui_component(
            component="Animation",
            priority=1,
            goal_data={
                "title": "Animation Title",
                "subtitle": "Animation Subtitle",
                "theme": "DARK",
                "type": "URL",
                "url": "https://media0.giphy.com/media/MZXmFVrbMA1qSDNGOt/giphy.gif",
                "lottie": {},
                "base64": "",
                "custom_style": {}
            }
        )

        # Publish debug message
        self.publish_debug("<b>Debug Info:</b> Running animation component<br>All systems nominal")

        # Show Simon Game
        await self.send_ui_component(
            component="SimonGame",
            priority=2,
            goal_data={"end_game_text": "End game"}
        )

        # Finish component of priority 2
        await self.finish_component(priority=2, include_lower=False)

        # Show a Memory Game with timeout
        await self.send_ui_component(
            component="MemoryGame",
            goal_data={
                "theme": "DARK",
                "title": "Memory Challenge",
                "end_game_text": "End game",
                "timeout": 4000  # milliseconds
            }
        )

    async def send_ui_component(self, component, goal_data, priority=1):
        goal = ChestWebCommand.Goal()
        goal.component = component
        goal.priority = priority
        goal.goal_data = json.dumps(goal_data)

        self.get_logger().info(f"[UI] Sending {component} (priority={priority})...")
        future = self.command_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        goal_handle = await future

        if not goal_handle.accepted:
            self.get_logger().error(f"[UI] {component} goal rejected")
            return

        result = await goal_handle.get_result_async()
        self.get_logger().info(f"[UI] {component} finished: {result.result}")

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"[FEEDBACK] {fb.feedback_msg}")

    async def finish_component(self, priority: int, include_lower: bool):
        if not self.finish_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("FinishCommand service not available")
            return

        req = ChestWebFinishCommand.Request()
        req.priority = priority
        req.include_lower = include_lower

        future = self.finish_client.call_async(req)
        await future
        self.get_logger().info(f"[FINISH] Closed priority={priority} include_lower={include_lower}")


def main(args=None):
    rclpy.init(args=args)
    node = ChestWebTester()

    import asyncio
    loop = asyncio.get_event_loop()
    loop.create_task(node.run_demo())

    rclpy.spin(node)
    rclpy.shutdown()
