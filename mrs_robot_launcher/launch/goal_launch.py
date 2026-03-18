from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory
import numpy as np # type: ignore

"""
this file launches a namespaced goal, which exists as the point agents must navigate toward. the goal can have one of two types, as
there are two classes of agents within the MRS. this typing is randomly chosen.

"""

def generate_launch_description():
    # define the paths to be used:
    pkg_path = get_package_share_directory("mrs_robot_launcher")
    xacro_path = PathJoinSubstitution([pkg_path, "urdf", "goal", "goal.urdf.xacro"])

    # define the launch arguments:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "true",
        description = "Whether or not to use sim time, defaulting to false"
    )

    goal_name = LaunchConfiguration("goal_name")
    goal_name_arg = DeclareLaunchArgument(
        "goal_name",
        default_value = "goal",
        description = "The name of the goal to be used, defaulting to goal"
    )

    goal_type = LaunchConfiguration("goal_type")
    goal_type_arg = DeclareLaunchArgument(
        "goal_type",
        default_value = "typeA",
        description = "The type of goal to be used, defaulting to typeA"
    )

    goal_initial_x_pos = LaunchConfiguration("goal_initial_x_pos")
    goal_initial_x_pos_arg = DeclareLaunchArgument(
        "goal_initial_x_pos", 
        default_value = "0.0",
        description = "The initial x position of the goal on spawn, defaulting to 0.0"
    )

    goal_initial_y_pos = LaunchConfiguration("goal_initial_y_pos")
    goal_initial_y_pos_arg = DeclareLaunchArgument(
        "goal_initial_y_pos", 
        default_value = "0.0",
        description = "The initial y position of the goal on spawn, defaulting to 0.0"
    )

    # get the description of the goal:
    robot_description = Command(["xacro ", xacro_path, " goal_name:=", goal_name, " goal_type:=", goal_type])
    rsp_parameters = {"robot_description": ParameterValue(robot_description, value_type = str), "use_sim_time" : use_sim_time}

    # define the nodes to be launched:
    rsp = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        name = "robot_state_publisher",
        namespace = goal_name,
        parameters = [rsp_parameters]
    )

    goal_spawner = Node(
        package = "ros_gz_sim",
        executable = "create",
        namespace = goal_name,
        arguments = [
            "-topic", "robot_description",
            "-name", goal_name,
            "-x", goal_initial_x_pos,
            "-y", goal_initial_y_pos,
        ],
        output = "screen"
    )

    return LaunchDescription([
        use_sim_time_arg, 
        goal_name_arg, 
        goal_type_arg,
        goal_initial_x_pos_arg,
        goal_initial_y_pos_arg,
        rsp, 
        goal_spawner
    ])