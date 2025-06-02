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
This launch file launches:
- The BT executable node, for a simple inspection task
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # paths:
    pkg_path = get_package_share_directory('mrs_bt_node')

    # launch arguments:
    tree = PathJoinSubstitution([pkg_path, 'trees', LaunchConfiguration('tree_name')])
    tree_arg = DeclareLaunchArgument(
        'tree_name',
        default_value = 'participant_tree.xml', 
        description = 'Sets the tree at runtime'
    )


    # nodes:
    bt_node = Node(
        package = 'mrs_bt_node',
        executable = 'bt_node',
        name = 'auction_test',
        output = 'screen',
        parameters = [{'tree_file' : tree}]
    )

    print(f'launching!')
    return LaunchDescription([
        tree_arg, 
        bt_node
    ])