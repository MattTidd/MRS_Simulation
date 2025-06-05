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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def gen_robot_list(n_robots, buffer):
    robots = []
    positions = []
    buffer_sq = buffer ** 2
    is_thermal = False
    is_depth = False

    for i in range(n_robots):
        placed = False
        tries = 0

        while not placed and tries < 1000:
            # generate candidate position:
            x_pose = round(np.random.uniform(-1.0, 1.0), 3)
            y_pose = round(np.random.uniform(-1.0, 1.0), 3)
            too_close = False
            
            # check the existing positions:
            for px, py in positions:
                dx = x_pose - px
                dy = y_pose - py

                if dx*dx + dy*dy < buffer_sq:
                    too_close = True
                    break
            
            # we want it to be not too close:
            if not too_close:
                positions.append((x_pose, y_pose))
                placed = True
            tries += 1
        
        # fallback for never getting placed:
        if not placed:
            positions.append((x_pose, y_pose))
            print(f"Warning: Placed robot at ({x_pose:.2f}, {y_pose:.2f}) without meeting the minimum distance threshold")

        print(f'we are on robot {i+1}/{n_robots}')
        # capability choice:
        cap = np.random.choice(['Thermal', 'Depth'])
        print(f'chosen capability was: {cap}')

        # check value:
        if cap == 'Thermal':
            is_thermal = True
            print(f"setting is_thermal flag to {is_thermal}\n")
        elif cap == 'Depth':
            is_depth = True
            print(f"setting is_depth flag to {is_depth}\n")

        # if there isn't a certain type by the end:
        if is_depth == False and i+1 == n_robots:
            cap = 'Depth'
            print(f'is_depth flag false at end of assignment, set cap to {cap}')

        elif is_thermal == False and i+1 == n_robots:
            cap = 'Thermal'
            print(f'is_thermal flag false at end of assignment, set cap to {cap}')

        # make robot list:
        robot_name = "robot" + str(i+1)
        robots.append({
            'name' : robot_name,
            'x_pose' : x_pose, 
            'y_pose' : y_pose, 
            'z_pose' : 0.0,
            'yaw' : 0.0, 
            'capability': cap
        })
    
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
        robots = gen_robot_list(int(num_robots.perform(context)), 0.5)    # actually get the context of that variable name, turn to int
        launch_actions = []

        for robot in robots:
            # generate a URDF with that robot name:
            robot_description = Command(['xacro ' , xacro_path, ' robot_name:=', robot['name']]) 
            print(f"{robot['name']} has capability {robot['capability']}")
            rsp_params = {'robot_description' : ParameterValue(robot_description, value_type = str), 'use_sim_time' : use_sim_time}

            # robot state publisher:
            rsp = Node(
                package = 'robot_state_publisher',
                executable = 'robot_state_publisher',
                namespace = robot['name'],
                parameters = [rsp_params]
            )

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

            delayed_spawner = RegisterEventHandler(
                event_handler = OnProcessStart(
                    target_action = rsp,
                    on_start = [spawner]
                )
            )

            launch_actions.extend([rsp, delayed_spawner])

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