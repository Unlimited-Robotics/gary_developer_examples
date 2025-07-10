# LED Node Example for Gary's LED System
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import json
from gary_leds_msgs.srv import LedsTurnOffAll, LedsTurnOffGroup, LedsSetColor
from gary_leds_msgs.action import LedsPlayAnimation
import time

class LedNode(Node):
    def __init__(self):
        super().__init__('led_node_example')
        self.group = ReentrantCallbackGroup()
        # Service clients
        self.turn_off_all_cli = self.create_client(LedsTurnOffAll, '/gary/leds/leds_turn_off_all', callback_group=self.group)
        self.turn_off_group_cli = self.create_client(LedsTurnOffGroup, '/gary/leds/leds_turn_off_group', callback_group=self.group)
        self.set_color_cli = self.create_client(LedsSetColor, '/gary/leds/leds_set_color', callback_group=self.group)
        # Action client
        self.anim_action_client = ActionClient(self, LedsPlayAnimation, '/gary/leds/leds_play_animation', callback_group=self.group)

    def turn_off_all(self):
        print("\033[1;34m--------------------\033[0m")
        print("\033[1;36mTurning off ALL LEDs...\033[0m")
        while not self.turn_off_all_cli.wait_for_service(timeout_sec=1.0):
            print("\033[1;33mWaiting for /gary/leds/leds_turn_off_all service...\033[0m")
        req = LedsTurnOffAll.Request()
        future = self.turn_off_all_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res:
            print(f"\033[1;32m[TurnOffAll] result: {res.error_code} - {res.error_msg}\033[0m")
        else:
            print("\033[1;31mFailed to turn off all LEDs\033[0m")
        print("\033[1;34m--------------------\033[0m")

    def turn_off_group(self, group):
        print("\033[1;34m--------------------\033[0m")
        print(f"\033[1;36mTurning off group: {group}\033[0m")
        while not self.turn_off_group_cli.wait_for_service(timeout_sec=1.0):
            print("\033[1;33mWaiting for /gary/leds/leds_turn_off_group service...\033[0m")
        req = LedsTurnOffGroup.Request()
        req.group = group
        future = self.turn_off_group_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res:
            print(f"\033[1;32m[TurnOffGroup:{group}] result: {res.error_code} - {res.error_msg}\033[0m")
        else:
            print(f"\033[1;31mFailed to turn off group {group}\033[0m")
        print("\033[1;34m--------------------\033[0m")

    def set_led_color(self, group, color):
        print("\033[1;34m--------------------\033[0m")
        print(f"\033[1;35mSetting color of group '{group}' to '{color}'...\033[0m")
        while not self.set_color_cli.wait_for_service(timeout_sec=1.0):
            print("\033[1;33mWaiting for /gary/leds/leds_set_color service...\033[0m")
        req = LedsSetColor.Request()
        req.group = group
        req.color = color
        future = self.set_color_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res:
            print(f"\033[1;32m[SetColor:{group}] {color} → result: {res.error_code} - {res.error_msg}\033[0m")
        else:
            print(f"\033[1;31mFailed to set color for group {group}\033[0m")
        print("\033[1;34m--------------------\033[0m")

    def play_led_animation(self, group, color, animation, repetitions=1, speed=1, execution_control=0):
        print("\033[1;34m--------------------\033[0m")
        print(f"\033[1;35mPlaying animation '{animation}' on group '{group}' with color '{color}' (repetitions={repetitions}, speed={speed})...\033[0m")
        while not self.anim_action_client.wait_for_server(timeout_sec=1.0):
            print("\033[1;33mWaiting for /gary/leds/leds_play_animation action server...\033[0m")
        goal_msg = LedsPlayAnimation.Goal()
        goal_msg.group = group
        goal_msg.color = color
        goal_msg.animation = animation
        goal_msg.repetitions = repetitions
        goal_msg.speed = speed
        goal_msg.execution_control = execution_control
        goal_msg.extra_info = json.dumps({})
        send_goal_future = self.anim_action_client.send_goal_async(goal_msg, feedback_callback=self.animation_feedback_cb)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print("\033[1;31mAnimation goal was rejected\033[0m")
            print("\033[1;34m--------------------\033[0m")
            return
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        print(f"\033[1;32m[Animation:{group}] Result: {result.error_code} - {result.error_msg}\033[0m")
        print("\033[1;34m--------------------\033[0m")

    def animation_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        print(f"\033[1;36m[Animation Feedback] {fb.feedback_msg} | Time left: {fb.time_left:.1f}s\033[0m")


if __name__ == '__main__':
    rclpy.init()
    node = LedNode()

    print("\033[1;33m====================\033[0m")
    print("\033[1;33m    LED NODE DEMO    \033[0m")
    print("\033[1;33m====================\033[0m")

    # 1. Turn off all LEDs
    node.turn_off_all()
    time.sleep(1)
    # 2. Turn off head group
    node.turn_off_group('head')
    time.sleep(1)
    # 3. Set head to red
    node.set_led_color('head', 'red')
    time.sleep(1)
    # 4. Set chest to green
    node.set_led_color('chest', 'green')
    time.sleep(1)

    # 5. Play wave animation on head
    node.play_led_animation('head', 'red', 'wave', repetitions=5, speed=1, execution_control=0)
    time.sleep(1)
    # 6. Play sparkle animation on head
    node.play_led_animation('head', 'white', 'talking', repetitions=4, speed=2, execution_control=0)
    time.sleep(1)
    # 7. Play rainbow animation on head
    node.play_led_animation('head', 'purple', 'arrow_down_in', repetitions=4, speed=1, execution_control=0)
    time.sleep(1)
    # 8. Play MOTION_1 animation on chest
    node.play_led_animation('chest', 'blue', 'MOTION_2', repetitions=2, speed=1, execution_control=1)

    print("\033[1;32m====================\033[0m")
    print("\033[1;32m   DEMO FINISHED    \033[0m")
    print("\033[1;32m====================\033[0m")
    node.destroy_node()
    rclpy.shutdown()
