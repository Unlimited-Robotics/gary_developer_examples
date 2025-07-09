import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from gary_hardware_msgs.msg import JointsInfo
from threading import Thread
from pymoveit2 import MoveIt2
import numpy as np
import math
import subprocess
import os


class ArmMoveitClient():

    def __init__(self):

        self.node = Node("right_arm",
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True
                         )

        try:
            self.repeat = self.node.get_parameter('repeat').get_parameter_value().integer_value
            self.max_velocity = self.node.get_parameter('max_velocity').get_parameter_value().double_value
            self.max_acceleration = self.node.get_parameter('max_acceleration').get_parameter_value().double_value
            self.velocity_scaling_list = self.node.get_parameter('velocity_scaling_list').get_parameter_value().double_array_value
            self.accelration_scaling_list = self.node.get_parameter('accelration_scaling_list').get_parameter_value().double_array_value
        except Exception as e:
            self.node.get_logger().error(f"Exception during parameter declaration: {e}")
            self.repeat = 1
            self.max_velocity = 0.5
            self.max_acceleration = 0.2
            self.velocity_scaling_list = [0.3, 0.5, 0.7]
            self.accelration_scaling_list = [0.1, 0.3, 0.5]

        self.prefix = "gary/arms/"
        self.joint_names = [
                    "arm_right_shoulder_FR_joint",
                     "arm_right_shoulder_RL_joint",
                     "arm_right_bicep_twist_joint",
                     "arm_right_bicep_FR_joint",
                     "arm_right_elbow_twist_joint",
                     "arm_right_elbow_FR_joint",
                     "arm_right_wrist_joint"
                     ]
        self.move_group = "right_arm"
        self.base_link = "base_link"
        self.ee_name = "center_fingers_right"
        self.trajectory_topic = "right_arm_controller/follow_joint_trajectory"
        self.__ros_init()
        self._logger = self.node.get_logger()
        self.joint_index_dict = {0 : [-169.0, 169.0],
                                 1 : [0.0, 175.0],
                                 2 : [-168.0, 168.0],
                                 3 : [-126.0, 126.0],
                                 4 : [-166.0, 166.0],
                                 5 : [-95.0, 95.0],
                                 6 : [-169.0, 169.0],}

        self.joint_error_dict = {}

        self.joint_info_msg = None

        self.test_passed = True

    ## write everything into a pkl file

    def __ros_init(self):

        self.callback_group = ReentrantCallbackGroup()

        self.py_moveit = MoveIt2(
            node=self.node,
            joint_names=self.joint_names,
            base_link_name=self.base_link,
            end_effector_name=self.ee_name,
            group_name=self.move_group,
            callback_group=self.callback_group,
            follow_joint_trajectory_action_name=self.trajectory_topic,
            prefix=self.prefix
        )

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)

        ## create a subscriber to the joint state
        self.joint_info_subscriber = self.node.create_subscription(JointsInfo, "/gary/arms/joints_info", self.joint_info_callback, 10)

    def start_rosbag_recording(self):
        rosbag_directory = '/robot/component_persistent_data/right_arm'
        if os.path.exists(rosbag_directory):
            os.system(f'rm -rf {rosbag_directory}')
        try:
            self.rosbag_process = subprocess.Popen([
                'ros2', 'bag', 'record',
                '-o', rosbag_directory,
                '--storage', 'mcap',
                '/gary/arms/joints_info'
            ])
            self._logger.info(f"Started recording rosbag at {rosbag_directory}.")
        except Exception as e:
            self._logger.error(f"Failed to start rosbag recording: {e}")
    
    
    def stop_rosbag_recording(self):
        if self.rosbag_process:
            self.rosbag_process.terminate()
            self.rosbag_process.wait()
            self._logger.info("Stopped recording rosbag.")

    def spin(self):

        executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        executor_thread.start()
        self.node.create_rate(1.0).sleep()


    def test_joint(self, number, max_position, min_position):
        self._logger.info(f"Testing joint name: {self.joint_names[number]}")

        for i in range(self.repeat):
            request_position = [0.0] * len(self.joint_names)
            request_position[1] = np.deg2rad(20.0)
            request_position[number] = max_position

            self.py_moveit.move_to_configuration(request_position,self.joint_names)
            self.py_moveit.wait_until_executed()

            request_position[number] = min_position
            self.py_moveit.move_to_configuration(request_position,self.joint_names)
            self.py_moveit.wait_until_executed()
    
    def start(self):
        self.start_rosbag_recording()
        self._logger.info("\n"
                          "████████ ███████ ███████ ████████               ███████ ████████  █████  ██████  ████████ \n"
                          "   ██    ██      ██         ██                  ██         ██    ██   ██ ██   ██    ██    \n"
                          "   ██    █████   ███████    ██        █████     ███████    ██    ███████ ██████     ██    \n"
                          "   ██    ██           ██    ██                       ██    ██    ██   ██ ██   ██    ██    \n"
                          "   ██    ███████ ███████    ██                  ███████    ██    ██   ██ ██   ██    ██    \n"
                          "                                                                                          \n"
                          "                                                                                          "
                          )

        for key in self.joint_index_dict.keys():
            for velocity in self.velocity_scaling_list:
                for acceleration in self.accelration_scaling_list:
                    self._logger.info(f"Testing velocity scaling: {velocity} and acceleration scaling: {acceleration}")
                    self.py_moveit.max_velocity = velocity
                    self.py_moveit.max_acceleration = acceleration
                    self.test_joint(key, self.joint_index_dict[key][1], self.joint_index_dict[key][0])
                    self._logger.info("-------------------------------------------------------------------------------------------")


        self._logger.info("Finished testing all joints")

        self._logger.info("-------------------------------------------------------------------------------------------")
        self._logger.info("Starting trajectory test")

        for velocity in self.velocity_scaling_list:
            for acceleration in self.accelration_scaling_list:
                self._logger.info(f"Testing velocity scaling: {velocity} and acceleration scaling: {acceleration}")
                self.py_moveit.max_velocity = velocity
                self.py_moveit.max_acceleration = acceleration
                for i in range(self.repeat):
                    request_position = [0.0] * len(self.joint_names)
                    request_position[0] = np.deg2rad(165.0)
                    request_position[1] = np.deg2rad(143.0)
                    request_position[2] = np.deg2rad(164.0)
                    request_position[3] = np.deg2rad(121.0)
                    request_position[4] = np.deg2rad(160.0)
                    request_position[5] = np.deg2rad(92.0)
                    request_position[6] = np.deg2rad(169.0)
                    self.py_moveit.move_to_configuration(request_position,self.joint_names)
                    self.py_moveit.wait_until_executed()
                    request_position[0] = np.deg2rad(-169.0)
                    request_position[1] = np.deg2rad(0.0)
                    request_position[2] = np.deg2rad(-165.0)
                    request_position[3] = np.deg2rad(-124.0)
                    request_position[4] = np.deg2rad(-161.0)
                    request_position[5] = np.deg2rad(-93.0)
                    request_position[6] = np.deg2rad(-166.0)
                    self.py_moveit.move_to_configuration(request_position,self.joint_names)
                    self.py_moveit.wait_until_executed()

        request_position[0] = np.deg2rad(-0.0)
        request_position[1] = np.deg2rad(0.0)
        request_position[2] = np.deg2rad(-0.0)
        request_position[3] = np.deg2rad(-0.0)
        request_position[4] = np.deg2rad(-0.0)
        request_position[5] = np.deg2rad(-0.0)
        request_position[6] = np.deg2rad(-0.0)
        self.py_moveit.move_to_configuration(request_position,self.joint_names)
        self.py_moveit.wait_until_executed()

        self._logger.info("-------------------------------------------------------------------------------------------")
        self._logger.info("\n"
            "████████ ███████ ███████ ████████               ██████   █████  ███████ ███████ ███████ ██████  ██        ██      \n"
            "   ██    ██      ██         ██                  ██   ██ ██   ██ ██      ██      ██      ██   ██ ██     ██  ██     \n"
            "   ██    █████   ███████    ██        █████     ██████  ███████ ███████ ███████ █████   ██   ██ ██         ██     \n"
            "   ██    ██           ██    ██                  ██      ██   ██      ██      ██ ██      ██   ██        ██  ██     \n"
            "   ██    ███████ ███████    ██                  ██      ██   ██ ███████ ███████ ███████ ██████  ██        ██      \n"
            "                                                                                                                  \n"
            "                                                                                                                  "
        )
        self._logger.info("Finished testing trajectory")
        self.stop_rosbag_recording()

    def joint_info_callback(self, msg):
        self.joint_info_msg = msg

        for info in self.joint_info_msg.joints_info:
            error_list = []
            errors = info.error.error_type
            calibrated = info.status.is_calibrated
            for type in errors:
                if type != 'OK':
                    error_list.append(type)
                if calibrated == False:
                    error_list.append("Not calibrated")

            if len(error_list) == 0:
                pass
            else:
                self.joint_error_dict[info.joint_name]= error_list


        if self.check_joints_error() == False:
            self._logger.info("\n"
                              "████████ ███████ ███████ ████████               ███████  █████  ██ ██      ███████ ██████  \n"
                              "   ██    ██      ██         ██                  ██      ██   ██ ██ ██      ██      ██   ██ \n"
                              "   ██    █████   ███████    ██        █████     █████   ███████ ██ ██      █████   ██   ██ \n"
                              "   ██    ██           ██    ██                  ██      ██   ██ ██ ██      ██      ██   ██ \n"
                              "   ██    ███████ ███████    ██                  ██      ██   ██ ██ ███████ ███████ ██████"
                              )
            # self.stop_bag_recording()
            ## kill the node
            self.node.destroy_node()
            self.executor.shutdown()
    def check_joints_error(self):
        if self.joint_error_dict != {}:
            for key in self.joint_error_dict.keys():
                self._logger.error(f"Error in joint: {key} with errors: {self.joint_error_dict[key]}")
            self.test_passed = False
            return False
        return True



def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    node = ArmMoveitClient()
    node.spin()
    node.start()
    rclpy.shutdown()


if __name__ == '__main__':
    main()