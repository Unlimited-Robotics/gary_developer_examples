
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from std_msgs.msg import String

# Import service and action types (assume they are available in the workspace)
from raya_audio_msgs.srv import GetPredefinedSounds, SetMicStatus
from raya_audio_msgs.action import PlayPredefinedSound, PlayFile, RecordSound
# from raya_status_msgs.msg import Status

import time

class SoundNode(Node):
    def __init__(self):
        super().__init__('sound_node_example')
        self.get_logger().info('Initializing SoundNode...')

        # Service clients
        self.predefined_sounds_cli = self.create_client(GetPredefinedSounds, '/gary/sound/get_predefined_sounds')
        self.set_mic_status_cli = self.create_client(SetMicStatus, '/gary/sound/set_mic_status')

        # Action clients
        self.play_predefined_sound_ac = ActionClient(self, PlayPredefinedSound, '/gary/sound/play_predefined_sound')
        self.play_file_ac = ActionClient(self, PlayFile, '/gary/sound/play_file')
        self.record_sound_ac = ActionClient(self, RecordSound, '/gary/sound/record_sound')

        # # Status subscriber
        # self.status_sub = self.create_subscription(
        #     Status,
        #     '/gary/sound/status',
        #     self.status_callback,
        #     QoSProfile(depth=10)
        # )

        self.latest_status = None

    # def status_callback(self, msg):
    #     self.latest_status = msg
    #     print(f"[Status] status={msg.status}, states={msg.states}, extra_info={msg.extra_info}")

    def get_predefined_sounds(self):
        print("\033[1;34m====================\033[0m")
        print('\033[1;34mRequesting predefined sounds...\033[0m')
        while not self.predefined_sounds_cli.wait_for_service(timeout_sec=1.0):
            print('\033[1;33mWaiting for /gary/sound/get_predefined_sounds service...\033[0m')
        req = GetPredefinedSounds.Request()
        req.origin = 'sound_node_example'
        future = self.predefined_sounds_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print(f"\033[1;32mPredefined sounds: {future.result().predefined_sounds}\033[0m")
            print("\033[1;34m====================\033[0m")
            return future.result().predefined_sounds
        else:
            print('\033[1;31mFailed to get predefined sounds\033[0m')
            print("\033[1;34m====================\033[0m")
            return []

    def set_mic_status(self, mic_id, status):
        print("\033[1;34m--------------------\033[0m")
        print(f'\033[1;36mSetting mic {mic_id} status to {status}...\033[0m')
        while not self.set_mic_status_cli.wait_for_service(timeout_sec=1.0):
            print('\033[1;33mWaiting for /gary/sound/set_mic_status service...\033[0m')
        req = SetMicStatus.Request()
        req.mic_id = mic_id
        req.status = status
        req.origin = 'sound_node_example'
        future = self.set_mic_status_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print(f"\033[1;32mMic status response: error_code={future.result().error_code}, error_msg={future.result().error_msg}\033[0m")
        else:
            print('\033[1;31mFailed to set mic status\033[0m')
        print("\033[1;34m--------------------\033[0m")

    def play_predefined_sound(self, name, volume=100, overwrite=False):
        print("\033[1;34m--------------------\033[0m")
        print(f'\033[1;35mPlaying predefined sound: {name} (volume={volume}, overwrite={overwrite})\033[0m')
        self.play_predefined_sound_ac.wait_for_server()
        goal = PlayPredefinedSound.Goal()
        goal.name = name
        goal.volume = volume
        goal.overwrite = overwrite
        send_goal_future = self.play_predefined_sound_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print('\033[1;31mPlayPredefinedSound goal rejected\033[0m')
            print("\033[1;34m--------------------\033[0m")
            return
        print('\033[1;32mPlayPredefinedSound goal accepted\033[0m')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        print(f'\033[1;32mPlayPredefinedSound result: error_code={result.error_code}, error_msg={result.error_msg}\033[0m')
        print("\033[1;34m--------------------\033[0m")

    def play_file(self, path, volume=100, overwrite=False):
        print("\033[1;34m--------------------\033[0m")
        print(f'\033[1;35mPlaying file: {path} (volume={volume}, overwrite={overwrite})\033[0m')
        self.play_file_ac.wait_for_server()
        goal = PlayFile.Goal()
        goal.path = path
        goal.volume = volume
        goal.overwrite = overwrite
        send_goal_future = self.play_file_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print('\033[1;31mPlayFile goal rejected\033[0m')
            print("\033[1;34m--------------------\033[0m")
            return
        print('\033[1;32mPlayFile goal accepted\033[0m')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        print(f'\033[1;32mPlayFile result: error_code={result.error_code}, error_msg={result.error_msg}\033[0m')
        print("\033[1;34m--------------------\033[0m")

    def record_sound(self, duration=2000, mic_id="mic0"):
        print("\033[1;34m--------------------\033[0m")
        print(f'\033[1;36mRecording sound from {mic_id} for {duration} ms...\033[0m')
        self.record_sound_ac.wait_for_server()
        goal = RecordSound.Goal()
        goal.duration = duration
        goal.mic_id = mic_id
        send_goal_future = self.record_sound_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print('\033[1;31mRecordSound goal rejected\033[0m')
            print("\033[1;34m--------------------\033[0m")
            return
        print('\033[1;32mRecordSound goal accepted\033[0m')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        print(f'\033[1;32mRecordSound result: error_code={result.error_code}, error_msg={result.error_msg}\033[0m')
        print("\033[1;34m--------------------\033[0m")

def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()

    print("\033[1;33m====================\033[0m")
    print("\033[1;33m   SOUND NODE DEMO   \033[0m")
    print("\033[1;33m====================\033[0m")

    # 1. Get predefined sounds
    sounds = node.get_predefined_sounds()
    if sounds:
        node.play_predefined_sound(sounds[0])
    else:
        print('\033[1;31mNo predefined sounds found.\033[0m')

    # 2. Play a file (replace with a valid path on your system)
    node.play_file('/robot/generic_persistent_data/examples/sounds/READY_DEMO_HEBREW.mp3', volume=100, overwrite=False)

    # 3. Set mic status (enable mic0)
    node.set_mic_status('mic0', True)
    time.sleep(1)
    # 4. Record sound from mic0
    node.record_sound(duration=1000, mic_id='mic0')
    # 5. Disable mic0
    node.set_mic_status('mic0', False)

    print("\033[1;32m====================\033[0m")
    print("\033[1;32m   DEMO FINISHED    \033[0m")
    print("\033[1;32m====================\033[0m")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

