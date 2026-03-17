from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

# what does this file launch?
# - rsp
# - jsp
# - RVIZ2

def generate_launch_description():
    # define paths:
    pkg_path = get_package_share_directory("mrs_robot_launcher")
    config_path = os.path.join(pkg_path, "config", "agent_rviz.rviz")
    xacro_path = PathJoinSubstitution([pkg_path, "urdf", "agent.urdf.xacro"])

    # define the launch arguments:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "false",
        description = "Whether or not to use sim time, defaulting to false"
    )

    agent_type = LaunchConfiguration("agent_type")
    agent_type_arg = DeclareLaunchArgument(
        "agent_type",
        default_value = "typeA",
        description = "The type of agent to be used"
    )

    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent1",
        description = "The name of the agent to be used"
    )

    visualize = LaunchConfiguration("visualize")
    visualize_arg = DeclareLaunchArgument(
        "visualize",
        default_value = "true",
        description = "Whether or not the user is simply visualizing the URDF, which is required for using RVIZ2"
    )

    # set the required parameters:
    robot_description = Command(["xacro ", xacro_path, " agent_name:=", agent_name, " agent_type:=", agent_type, " visualize:=", visualize])
    rsp_parameters = {"robot_description": ParameterValue(robot_description, value_type = str), "use_sim_time" : use_sim_time}

    # nodes:
    rsp = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        namespace = agent_name,
        parameters = [rsp_parameters],
    )

    jsp = Node(
        package = "joint_state_publisher",
        executable = "joint_state_publisher",
        namespace = agent_name, 
        output = "screen", 
    )

    rviz = Node(
        package = "rviz2",
        executable = "rviz2",
        namespace = agent_name,
        output = "screen",
        parameters = [{"use_sim_time" : use_sim_time}],
        arguments = ["-d", config_path]
    )

    return LaunchDescription([
        use_sim_time_arg,
        agent_type_arg, 
        agent_name_arg,
        visualize_arg,
        rsp, 
        jsp,
        rviz
    ])
