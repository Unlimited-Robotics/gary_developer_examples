import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from threading import Thread
from pymoveit2 import MoveIt2
import numpy as np
import math


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


right_joint_names = ["arm_right_shoulder_rail_joint",
                    "arm_right_shoulder_FR_joint",
                     "arm_right_shoulder_RL_joint",
                     "arm_right_bicep_twist_joint",
                     "arm_right_bicep_FR_joint",
                     "arm_right_elbow_twist_joint",
                     "arm_right_elbow_FR_joint",
                     "arm_right_wrist_joint"
                     ]

left_joint_names = ["arm_left_shoulder_rail_joint",
                     "arm_left_shoulder_FR_joint",
                     "arm_left_shoulder_RL_joint",
                     "arm_left_bicep_twist_joint",
                     "arm_left_bicep_FR_joint",
                     "arm_left_elbow_twist_joint",
                     "arm_left_elbow_FR_joint",
                     "arm_left_wrist_joint"
                     ]

right_move_group = "right_arm"
left_move_group = "left_arm"

base_link = "base_link"

right_ee_name = "center_fingers_right"
left_ee_name = "center_fingers_left"

max_velocity = 0.5
max_acceleration = 0.2

right_follow_trajectory_topic = "right_arm_controller/follow_joint_trajectory"
left_follow_trajectory_topic = "left_arm_controller/follow_joint_trajectory"
namespace = "gary/arms/"



class ArmMoveitClient():

    def __init__(self):

        self.node = Node("right_arm",
                         allow_undeclared_parameters=False,
                         automatically_declare_parameters_from_overrides=True
                         )

        self.prefix = namespace
        self.base_link = base_link
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

        self.right_joint_names = right_joint_names
        self.right_move_group = right_move_group
        self.right_ee_name = right_ee_name
        self.right_trajectory_topic = right_follow_trajectory_topic

        self.left_joint_names = left_joint_names
        self.left_move_group = left_move_group
        self.left_ee_name = left_ee_name
        self.left_trajectory_topic = left_follow_trajectory_topic

        self.__ros_init()
        self._logger = self.node.get_logger()


    ## write everything into a pkl file

    def __ros_init(self):

        self.callback_group = ReentrantCallbackGroup()

        self.right_py_moveit = MoveIt2(
            node=self.node,
            joint_names=self.right_joint_names,
            base_link_name=self.base_link,
            end_effector_name=self.right_ee_name,
            group_name=self.right_move_group,
            callback_group=self.callback_group,
            follow_joint_trajectory_action_name=self.right_trajectory_topic,
            prefix=self.prefix
        )

        self.left_py_moveit = MoveIt2(
            node=self.node,
            joint_names=self.left_joint_names,
            base_link_name=self.base_link,
            end_effector_name=self.left_ee_name,
            group_name=self.left_move_group,
            callback_group=self.callback_group,
            follow_joint_trajectory_action_name=self.left_trajectory_topic,
            prefix=self.prefix
        )

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)




    def spin(self):

        executor_thread = Thread(target=self.executor.spin, daemon=True, args=())
        executor_thread.start()
        self.node.create_rate(1.0).sleep()


    def start(self):

        request_position = ([0.0, np.deg2rad(0.0),np.deg2rad(0),np.deg2rad(0.0),np.deg2rad(90.0),np.deg2rad(0.0),np.deg2rad(0.0),np.deg2rad(-0.0)])
        self.right_py_moveit.move_to_configuration(request_position,self.right_joint_names)

        request_position = ([0.0, np.deg2rad(0.0),np.deg2rad(0),np.deg2rad(0.0),np.deg2rad(-90.0),np.deg2rad(0.0),np.deg2rad(0.0),np.deg2rad(-0.0)])
        self.left_py_moveit.move_to_configuration(request_position,self.left_joint_names)

        self.left_py_moveit.wait_until_executed()

        request_position = ([0.0, np.deg2rad(0.0),np.deg2rad(0),np.deg2rad(0.0),np.deg2rad(0.0),np.deg2rad(0.0),np.deg2rad(0.0),np.deg2rad(-0.0)])
        self.right_py_moveit.move_to_configuration(request_position,self.right_joint_names)

        request_position = ([0.0, np.deg2rad(0.0),np.deg2rad(0),np.deg2rad(0.0),np.deg2rad(0.0),np.deg2rad(0.0),np.deg2rad(0.0),np.deg2rad(-0.0)])
        self.left_py_moveit.move_to_configuration(request_position,self.left_joint_names)

        self.left_py_moveit.wait_until_executed()





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