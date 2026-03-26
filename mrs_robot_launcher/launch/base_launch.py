from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory

"""
this file is the base launch template for an agent in the MRS. from here, the following namespaced nodes are launched:
- robot_state_publisher
- spawner
- diff_drive_spawner
- joint_broadcaster_spawner
- laser_scan_matcher
- sensor_covariance_node
- ekf

and the idea is that this launch file template be included and populated in a loop over the number of agents desired
"""

def generate_launch_description():
    # define the paths to be used:
    pkg_path = get_package_share_directory("mrs_robot_launcher")
    xacro_path = PathJoinSubstitution([pkg_path, "urdf", "agent", "agent.urdf.xacro"])
    odom_launch_path = PathJoinSubstitution([pkg_path, "launch", "odom_launch.py"])

    # define the launch arguments:
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

    agent_type = LaunchConfiguration("agent_type")
    agent_type_arg = DeclareLaunchArgument(
        "agent_type",
        default_value = "typeA",
        description = "The type of agent to be used, defaulting to typeA"
    )

    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent1",
        description = "The name of the agent to be used, defaulting to agent1"
    )

    agent_initial_x_pos = LaunchConfiguration("agent_initial_x_pos")
    agent_initial_x_pos_arg = DeclareLaunchArgument(
        "agent_initial_x_pos",
        default_value = "0.0",
        description = "The initial x position of the agent on spawn, defaulting to 0.0"
    )

    agent_initial_y_pos = LaunchConfiguration("agent_initial_y_pos")
    agent_initial_y_pos_arg = DeclareLaunchArgument(
        "agent_initial_y_pos",
        default_value = "0.0",
        description = "The initial y position of the agent on spawn, defaulting to 0.0"
    )

    agent_initial_yaw = LaunchConfiguration("agent_initial_yaw")
    agent_initial_yaw_arg = DeclareLaunchArgument(
        "agent_initial_yaw",
        default_value = "0.0",
        description = "The initial yaw of the agent on spawn, defaulting to 0.0"
    )

    # set the required parameters:
    robot_description = Command(["xacro ", xacro_path, " agent_name:=", agent_name, " agent_type:=", agent_type, " visualize:=", visualize])
    rsp_parameters = {"robot_description": ParameterValue(robot_description, value_type = str), "use_sim_time" : use_sim_time}

    # define the nodes to be launched:
    rsp = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        name = "robot_state_publisher",
        namespace = agent_name,
        parameters = [rsp_parameters]
    )

    agent_spawner = Node(
        package = "ros_gz_sim",
        executable = "create",
        namespace = agent_name,
        arguments = [
            "-topic", "robot_description",
            "-name", agent_name,
            "-x", agent_initial_x_pos,
            "-y", agent_initial_y_pos,
            "-Y", agent_initial_yaw
        ],
        output = "screen"
    )

    diff_drive_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        namespace = agent_name,
        arguments = ["diff_controller"]
    )

    joint_broadcaster_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        namespace = agent_name,
        arguments = ["joint_broad"]
    )

    odom_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([odom_launch_path]),
        launch_arguments = {"use_sim_time" : use_sim_time,
                            "agent_name" : agent_name}.items()
    )

    return LaunchDescription([
        # args:
        use_sim_time_arg,
        visualize_arg,
        agent_type_arg, 
        agent_name_arg,
        agent_initial_x_pos_arg,
        agent_initial_y_pos_arg,
        agent_initial_yaw_arg,

        # nodes:
        rsp,
        agent_spawner, 
        diff_drive_spawner, 
        joint_broadcaster_spawner,
        odom_nodes
    ])



    

