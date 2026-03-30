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

    # define the launch arguments:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "false",
        description = "Whether or not to use sim time, defaulting to false"
    )

    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent1",
        description = "The name of the agent to be used"
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
        agent_name_arg,
        rviz
    ])
