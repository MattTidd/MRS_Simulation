# Copyright (c) 2024 Matthew Allan Tidd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""

This launch file brings up a prefixed robot for use in an MRS.

"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "robot" + str(i+1)
        x_pos = float(i-0.75)
        robots.append({'name' : robot_name, 'x_pose' : x_pos, 'y_pose': 0.0, 'z_pose' : 0.01})
    
    return robots

def generate_launch_description():
    # launch arguments:
    robot_type = LaunchConfiguration('robot_type')
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value = 'X3',
        description = 'Robot type to be used. Currently just the X3.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'true',
        description = 'Whether to use_sim_time or not, defaults to true.'
    )

    num_robots = LaunchConfiguration('num_robots')
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value = '2',
        description = 'Number of robots to spawn within the MRS.'
    )

    robot_name = LaunchConfiguration('robot_name')
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', 
        default_value = 'robot1',
        description = 'Name of the robot. Assigned at runtime but needed as a launch argument.'
    )

    # paths & params:
    pkg_path = get_package_share_directory('mrs_robot_launcher')
    xacro_path = PathJoinSubstitution([pkg_path, 'urdf', robot_type, 'robot.urdf.xacro'])
    robot_description = Command(['xacro ' , xacro_path, ' robot_name:=', robot_name])

    rsp = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{
            'robot_description' : robot_description,
            'use_sim_time' : use_sim_time
        }]
    )

    # jsp = Node(
    #             name = 'joint_state_publisher',
    #             executable = 'joint_state_publisher',
    #             output = 'screen',
    #             parameters = [{'use_sim_time' : use_sim_time}]

    #         )

    # # use an opaque function to generate robot launch descriptions:
    # def launch_robots(context):
    #     robots = gen_robot_list(int(context.perform_substitution(num_robots)))
    #     launch_actions = []

    #     for robot in robots:
    #         # generate a URDF with that robot name:
    #         robot_description = Command(['xacro ', xacro_path,
    #                                      ' robot_name:=', robot['name']
    #                                      ])
    #         # robot state publisher:
    #         rsp = Node(
    #             package = 'robot_state_publisher',
    #             executable = 'robot_state_publisher',
    #             namespace = robot['name'],
    #             parameters = [{
    #                 'robot_description' : robot_description, 
    #                 'use_sim_time' : use_sim_time
    #             }]
    #         )

    #         jsp = Node(
    #             name = 'joint_state_publisher',
    #             executable = 'joint_state_publisher',
    #             namespace = robot['name'],
    #             parameters = [{'use_sim_time' : use_sim_time}]

    #         )

    #         launch_actions.extend([rsp, jsp])

    #     return launch_actions

    return LaunchDescription([
        robot_type_arg,
        use_sim_time_arg, 
        num_robots_arg,
        robot_name_arg,
        rsp,
        # jsp
        # OpaqueFunction(function = launch_robots)
    ])