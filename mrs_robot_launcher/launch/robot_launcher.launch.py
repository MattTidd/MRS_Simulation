# Copyright (c) 2025 Matthew Allan Tidd
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

def gen_robot_list(n_robots, buffer, capabilities):
    robots = []
    positions = []
    buffer_sq = buffer ** 2

    if n_robots < len(capabilities):
        raise ValueError(f"Must have a robot for every provided capability! Given {n_robots} robots and {len(capabilities)} capabilities.")

    capabilities += list(np.random.choice(capabilities, size = n_robots - len(capabilities)))
    np.random.shuffle(capabilities)

    for i in range(n_robots):
        placed = False
        tries = 0

        while not placed and tries < 1000:
            # generate candidate position:
            x_pose = round(np.random.uniform(-2.0, 2.0), 3)
            y_pose = round(np.random.uniform(-2.0, 2.0), 3)
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

        # assign capabilities:
        cap = capabilities[i]

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
    # paths & params:
    pkg_path = get_package_share_directory('mrs_robot_launcher')
    xacro_path = PathJoinSubstitution([pkg_path, 'urdf', LaunchConfiguration('robot_type'), 'robot.urdf.xacro'])
    gazebo_params_path = os.path.join(pkg_path, 'config', 'params_gazebo.yaml')

    # launch arguments:
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
        default_value = '2',
        description = 'Number of robots to spawn within the MRS.'
    )

    world = PathJoinSubstitution([pkg_path, 'worlds', LaunchConfiguration('world_name')])
    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value = 'empty.world',
        description = 'Name of the world to be launched, contained in the Worlds folder.'
    )

    # use an opaque function to generate robot launch descriptions:
    def launch_robots(context):
        # get list of robots:

        # context is used to actually get the context of that variable name, and then turn that value to an int
        robots = gen_robot_list(int(num_robots.perform(context)), 0.5, ['thermal', 'depth'])
        launch_actions = []

        for robot in robots:
            # generate a URDF with that robot name:
            robot_description = Command(['xacro ' , xacro_path, ' robot_name:=', robot['name'], ' robot_capability:=', robot['capability']]) 
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
                    '-entity', robot['name'] + '_' + robot['capability'],
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
                'world' : world,
                'extra_gazebo_args' : '--ros-args --params-file ' + gazebo_params_path}.items()
    )

    return LaunchDescription([
        robot_type_arg,
        use_sim_time_arg, 
        num_robots_arg,
        world_arg,
        OpaqueFunction(function = launch_robots),
        gazebo
    ])