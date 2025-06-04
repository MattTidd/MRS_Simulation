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
import numpy as np
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "robot" + str(i+1)
        positions = np.random.uniform(-1.5, 1.5, [1,2])
        robots.append({'name' : robot_name, 'x_pose' : positions[0,0], 'y_pose': positions[0,1], 'z_pose' : 0.0, 'yaw' : 0.0})
    
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
        default_value = 'false',
        description = 'Whether to use_sim_time or not, defaults to false.'
    )

    num_robots = LaunchConfiguration('num_robots')
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value = '1',
        description = 'Number of robots to spawn within the MRS.'
    )

    # paths & params:
    pkg_path = get_package_share_directory('mrs_robot_launcher')
    xacro_path = PathJoinSubstitution([pkg_path, 'urdf', robot_type, 'robot.urdf.xacro'])
    gazebo_params_path = os.path.join(pkg_path, 'config', 'params_gazebo.yaml')

    # use an opaque function to generate robot launch descriptions:
    def launch_robots(context):
        # get list of robots:
        robots = gen_robot_list(int(num_robots.perform(context)))    # actually get the context of that variable name, turn to int
        launch_actions = []

        for robot in robots:
            # generate a URDF with that robot name:
            robot_description = Command(['xacro ' , xacro_path, ' robot_name:=', robot['name']]) 
            rsp_params = {'robot_description' : ParameterValue(robot_description, value_type = str), 'use_sim_time' : use_sim_time}

            # robot state publisher:
            rsp = Node(
                package = 'robot_state_publisher',
                executable = 'robot_state_publisher',
                namespace = robot['name'],
                parameters = [rsp_params]
            )

            jsp = Node(
                package = 'joint_state_publisher',
                executable = 'joint_state_publisher',
                name = 'joint_state_publisher',
                namespace = robot['name'],
                parameters = [{'use_sim_time' : use_sim_time}]

            )

            print("\n", robot['name'], "has position", 
                  f"x: {robot['x_pose']} | ", 
                  f"y: {robot['y_pose']} | ",
                  f"z: {robot['z_pose']} | ",
                  f"yaw: {robot['yaw']}")

            spawner = Node(
                package = 'gazebo_ros',
                executable = 'spawn_entity.py',
                namespace = robot['name'],
                output = 'screen',
                arguments = [
                    '-topic', 'robot_description',
                    '-entity', robot['name'],
                    '-x', str(robot['x_pose']),
                    '-y', str(robot['y_pose']),
                    '-z', str(robot['z_pose']),
                    '-Y', str(robot['yaw'])
                ]
            )

            launch_actions.extend([rsp, jsp, spawner])

        return launch_actions

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments = {
                # 'verbose' : 'true',
                'extra_gazebo_args' : '--ros-args --params-file ' + gazebo_params_path}.items()
    )

    return LaunchDescription([
        robot_type_arg,
        use_sim_time_arg, 
        num_robots_arg,
        OpaqueFunction(function = launch_robots), 
        gazebo
    ])