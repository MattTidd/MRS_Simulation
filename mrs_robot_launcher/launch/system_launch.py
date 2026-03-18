from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

"""
this file is the launch file for launching the MRS, wherein the user specifies lists of parameters for each agent in the system, 
which are then looped over and the template launch file is called for number of agents to be launched. in this regard, this file
launches:

- everything that is contained within "mrs_robot_launcher/launch/base_launch.py"
- gazebo

"""

def generate_launch_description():
    # define the paths:
    pkg_path = get_package_share_directory("mrs_robot_launcher")
    template_path = PathJoinSubstitution([pkg_path, "launch", "base_launch.py"])
    gazebo_launch_path = PathJoinSubstitution([pkg_path, "launch", "gazebo_launch.py"])
    bridge_path = PathJoinSubstitution([pkg_path, "config", "bridges", "system_bridge_parameters.yaml"])

    # define the arguments for launching:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "true",
        description = "Whether or not to use sim time, defaulting to false"
    )

    visualize = LaunchConfiguration("visualize")
    visualize_arg = DeclareLaunchArgument(
        "visualize",
        default_value = "false",
        description = "Whether or not to visualize using RVIZ2, defaulting to false"
    )

    world = PathJoinSubstitution([pkg_path, "worlds", LaunchConfiguration("world")])
    world_arg = DeclareLaunchArgument(
        "world",
        default_value = "empty_world.sdf",
        description = "Name of the world to be loaded, defaulting to empty_world.sdf"
    )
    
    # define the lists of parameters for launching:
    agent_names         = ["agent1" ,  "agent2"]
    agent_types         = ["typeA"  ,  "typeB"]
    agent_initial_xs    = ["-3.0"   ,  "3.0"]
    agent_initial_ys    = ["-3.0"   ,  "3.0"]
    agent_initial_yaws  = ["0.0"    ,  "0.0"]

    # agent_names         = ["agent1"]
    # agent_types         = ["typeA"]
    # agent_initial_xs    = ["-3.0" ]
    # agent_initial_ys    = ["-3.0"]
    # agent_initial_yaws  = ["0.0"]

    num_agents = len(agent_names)
    delay = 2.5
    templates = []

    # loop over the number of agents and call the base template:
    for agent in range(num_agents):
        # include the base_template:
        foo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([template_path]),
            launch_arguments = {"use_sim_time" : use_sim_time,
                                "visualize" : visualize, 
                                "agent_name" : agent_names[agent],
                                "agent_type" : agent_types[agent],
                                "agent_initial_x_pos" : agent_initial_xs[agent],
                                "agent_initial_y_pos" : agent_initial_ys[agent],
                                "agent_initial_yaw" : agent_initial_yaws[agent]}.items()
        )

        timed_foo = TimerAction(period = delay + float(agent * delay), 
                                actions = [foo])

        # add instance of template to list of templates:
        templates.append(timed_foo)

    # launch gazebo:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments = {"world" : world}.items()
    )

    # use a single ros bridge node:
    ros_gz_bridge = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        name = "ros_gz_bridge",
        arguments = ["--ros-args", "-p", ["config_file:=", bridge_path]]
    )

    return LaunchDescription([
        use_sim_time_arg, 
        visualize_arg, 
        world_arg,  
        gazebo, 
        ros_gz_bridge
    ] + templates)
