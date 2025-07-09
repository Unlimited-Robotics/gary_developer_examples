# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import ExecuteProcess, OpaqueFunction

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os

RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
RESET = '\033[0m'

def launch_setup(context, *args, **kwargs):
    run_left = LaunchConfiguration("run_left").perform(context)
    run_right = LaunchConfiguration("run_right").perform(context)

    nodes = []
    if(run_left):
        left_gripper_node = Node(
            package = "gary_examples_package",
            executable = "left_gripper",
            parameters = [],
            output = "both"
        )
        nodes.append(left_gripper_node)


    if(run_right):
        right_gripper_node = Node(
            package = "gary_examples_package",
            executable = "right_gripper",
            parameters = [],
            output = "both"
        )
        nodes.append(right_gripper_node)



    return nodes


def generate_launch_description():
    ns_arg = DeclareLaunchArgument(
        "namespace",
        default_value="gary/example/",
        description="define namespace",
    )

    run_left = DeclareLaunchArgument(
        "run_left",
        default_value="false",
        description="Run left gripper node",
    )

    run_right = DeclareLaunchArgument(
        "run_right",
        default_value="false",
        description="Run right gripper node",
    )

    return LaunchDescription(
        [ns_arg,
        run_left,
        run_right,
         OpaqueFunction(function=launch_setup),
         ])
