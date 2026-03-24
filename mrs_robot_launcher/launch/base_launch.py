from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

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

    # need to use the passed agent_name parameter to set the path of the bridge params, ekf params, and bridge node name:
    ekf_path = PathJoinSubstitution([pkg_path, "config", "ekfs", [agent_name, TextSubstitution(text = "_ekf_params.yaml")]])
    base_frame = [agent_name, TextSubstitution(text = "_base_link")]
    imu_frame = [agent_name, TextSubstitution(text = "_imu_link")]

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

    laser_scan_matcher = Node(
        package = "rf2o_laser_odometry",
        executable = "rf2o_laser_odometry_node",
        name = "laser_odometry_node",
        namespace = agent_name, 
        output = "screen",
        parameters = [{
            "laser_scan_topic" : "scan",
            "odom_topic" : "lidar_odom",
            "publish_tf" : False,
            "base_frame_id" : base_frame,
            "odom_frame_id" : "odom",
            "init_pose_from_topic" : "",
            "freq" : 60.0}]
    )

    covariance_filter_node = Node(
        package = "covariance_filter",
        executable = "covariance_filter_node",
        name = "covariance_filter",
        output = "screen", 
        parameters = [{"imu_frame" : imu_frame}]
    )

    covariance_filter_node = GroupAction(
        actions = [
            PushRosNamespace(agent_name),
            covariance_filter_node
        ]
    )

    ekf_node = Node(
        package = "robot_localization",
        executable = "ekf_node", 
        name = "ekf_filter_node",
        namespace = agent_name,
        output = "screen", 
        parameters = [ekf_path, {"use_sim_time" : use_sim_time}], 
        remappings = [
            ("odometry/filtered", "odom")
        ]
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
        laser_scan_matcher,
        covariance_filter_node, 
        ekf_node
    ])



    

