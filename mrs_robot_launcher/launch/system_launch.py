from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

"""
this file is the launch file for launching the MRS, wherein the user specifies lists of parameters for each agent in the system, 
which are then looped over and the template launch file is called for number of agents to be launched. in this regard, this file
launches:

- everything that is contained within "mrs_robot_launcher/launch/base_launch.py"
- gazebo
- system manager gui

"""
def generate_launch_description():
    # define the paths:
    pkg_path           = get_package_share_directory("mrs_robot_launcher")
    gz_sim_path        = get_package_share_directory("ros_gz_sim")
    gazebo_launch_path = PathJoinSubstitution([gz_sim_path, "launch", "gz_sim.launch.py"])
    gui_path = os.path.join(pkg_path, "config", "gui.config")

    # define the arguments for launching:
    world = PathJoinSubstitution([pkg_path, "worlds", LaunchConfiguration("world")])
    world_arg = DeclareLaunchArgument(
        "world",
        default_value = "world_3.sdf",
        description = "Name of the world to be loaded, defaulting to world_3.sdf"
    )

    # launch gazebo:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments = {"gz_args" : [f"--gui-config {gui_path} -r -v1 ", world]}.items()
    )

    return LaunchDescription([
        world_arg,  
        gazebo, 
    ])
