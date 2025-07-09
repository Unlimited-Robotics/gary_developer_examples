import rclpy
from rclpy.node import Node

from gary_leds_msgs.srv import LedsTurnOffAll, LedsTurnOffGroup, LedsSetColor
from gary_leds_msgs.action import LedsPlayAnimation

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

import json

class LedsController(Node):
    def __init__(self):
        super().__init__('leds_control_node')
        self.group = ReentrantCallbackGroup()

        # Clients for services
        self.turn_off_all_cli = self.create_client(LedsTurnOffAll, '/gary/leds/leds_turn_off_all', callback_group=self.group)
        self.turn_off_group_cli = self.create_client(LedsTurnOffGroup, '/gary/leds/leds_turn_off_group', callback_group=self.group)
        self.set_color_cli = self.create_client(LedsSetColor, '/gary/leds/leds_set_color', callback_group=self.group)

        # Action client for animations
        self.anim_action_client = ActionClient(self, LedsPlayAnimation, '/gary/leds/leds_play_animation', callback_group=self.group)

        self.get_logger().info("LED Controller initialized")

        # Demo: execute a test sequence on startup
        self.create_timer(2.0, self.demo)

    async def demo(self):
        self.get_logger().info("Running LED demo...")

        await self.turn_off_all_leds()
        await self.turn_off_group('head')
        await self.set_led_color('head', 'red')
        await self.set_led_color('chest', 'green')
        await self.play_led_animation('head', 'red', 'wave', repetitions=5, speed=1, execution_control=0)
        await self.play_led_animation('chest', 'blue', 'MOTION_1', repetitions=2, speed=1, execution_control=1)

    async def turn_off_all_leds(self):
        if not self.turn_off_all_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /leds_turn_off_all not available")
            return
        req = LedsTurnOffAll.Request()
        res = await self.turn_off_all_cli.call_async(req)
        self.get_logger().info(f"[TurnOffAll] result: {res.error_code} - {res.error_msg}")

    async def turn_off_group(self, group: str):
        if not self.turn_off_group_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /leds_turn_off_group not available")
            return
        req = LedsTurnOffGroup.Request()
        req.group = group
        res = await self.turn_off_group_cli.call_async(req)
        self.get_logger().info(f"[TurnOffGroup:{group}] result: {res.error_code} - {res.error_msg}")

    async def set_led_color(self, group: str, color: str):
        if not self.set_color_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /leds_set_color not available")
            return
        req = LedsSetColor.Request()
        req.group = group
        req.color = color
        res = await self.set_color_cli.call_async(req)
        self.get_logger().info(f"[SetColor:{group}] {color} → result: {res.error_code} - {res.error_msg}")

    async def play_led_animation(self, group: str, color: str, animation: str, repetitions=1, speed=1, execution_control=0):
        if not self.anim_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server /leds_play_animation not available")
            return

        goal_msg = LedsPlayAnimation.Goal()
        goal_msg.group = group
        goal_msg.color = color
        goal_msg.animation = animation
        goal_msg.repetitions = repetitions
        goal_msg.speed = speed
        goal_msg.execution_control = execution_control
        goal_msg.extra_info = json.dumps({})  # Required field

        self.get_logger().info(f"[Animation:{group}] Playing {animation} in {color} x{repetitions}")
        future = self.anim_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.animation_feedback_cb
        )

        goal_handle = await future
        if not goal_handle.accepted:
            self.get_logger().error("Animation goal was rejected")
            return

        result = await goal_handle.get_result_async()
        self.get_logger().info(f"[Animation:{group}] Result: {result.result.error_code} - {result.result.error_msg}")

    def animation_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"[Animation Feedback] {fb.feedback_msg} | Time left: {fb.time_left:.1f}s")


def main(args=None):
    rclpy.init(args=args)
    node = LedsController()

    # Required to run asyncio tasks with ROS 2
    import asyncio
    loop = asyncio.get_event_loop()
    loop.create_task(node.demo())

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
