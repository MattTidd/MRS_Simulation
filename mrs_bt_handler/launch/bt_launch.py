from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # define the paths to be used:
    pkg_path   = get_package_share_directory("mrs_bt_handler")
    model_path = os.path.join(pkg_path, "suitability_model")

    # define the arguments for launching:
    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent1",
        description   = "Name of the agent to be launched"
    )

    agent_type = LaunchConfiguration("agent_type")
    agent_type_arg = DeclareLaunchArgument(
        "agent_type",
        default_value = "typeA",
        description   = "Type of the agent to be launched"
    )

    agent_initial_x = LaunchConfiguration("agent_initial_x")
    agent_initial_x_arg = DeclareLaunchArgument(
        "agent_initial_x",
        default_value = "0.0",
        description   = "Initial x position of the agent."
    )

    agent_initial_y = LaunchConfiguration("agent_initial_y")
    agent_initial_y_arg = DeclareLaunchArgument(
        "agent_initial_y",
        default_value = "0.0",
        description   = "Initial y position of the agent."
    )

    agent_initial_yaw = LaunchConfiguration("agent_initial_yaw")
    agent_initial_yaw_arg = DeclareLaunchArgument(
        "agent_initial_yaw",
        default_value = "0.0",
        description = "Initial yaw position of the agent."
    )

    num_agents = LaunchConfiguration("num_agents")
    num_agents_arg = DeclareLaunchArgument(
        "num_agents",
        default_value = "2",
        description   = "Number of agents in the MRS"
    )

    drl_model = LaunchConfiguration("drl_model")
    drl_model_arg = DeclareLaunchArgument(
        "drl_model",
        default_value = "SAC_001",
        description   = "DRL model to be used for the navigational policy"
    )

    # define the node to be launched:
    bt_node = Node(
        package     = "mrs_bt_handler",
        executable  = "bt_node",
        name        = "bt_node",
        namespace   = agent_name,
        parameters  = [{
            "agent_name"        : agent_name,
            "agent_type"        : agent_type, 
            "agent_initial_x"   : agent_initial_x,
            "agent_initial_y"   : agent_initial_y,
            "agent_initial_yaw" : agent_initial_yaw,
            "model_name"        : drl_model,
            "num_agents"        : num_agents,
            "model_path"        : model_path
        }],
    )

    return LaunchDescription([
        # args:
        agent_name_arg,
        agent_type_arg, 
        agent_initial_x_arg,
        agent_initial_y_arg,
        agent_initial_yaw_arg,
        num_agents_arg, 
        drl_model_arg,

        # nodes:
        bt_node
    ])