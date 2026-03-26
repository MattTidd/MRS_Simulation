import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory

# what does this file launch?
# - laser_scan_matcher
# - imu/lidar covariance filter node
# - extended kalman filter

def generate_launch_description():
    # set the required paths:
    pkg_path = get_package_share_directory("mrs_robot_launcher")

    # define the launch arguments:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value = "true",
        description = "Whether or not to use sim time, defaulting to false"
    )

    agent_name = LaunchConfiguration("agent_name")
    agent_name_arg = DeclareLaunchArgument(
        "agent_name",
        default_value = "agent1",
        description = "The name of the agent to be used, defaulting to agent1"
    )

    # use passed agent name to set paths and parameters:
    ekf_path = PathJoinSubstitution([pkg_path, "config", "ekfs", [agent_name, TextSubstitution(text = "_ekf_params.yaml")]])
    base_frame = [agent_name, TextSubstitution(text = "_base_link")]
    imu_frame = [agent_name, TextSubstitution(text = "_imu_link")]

    # nodes to be launched:
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
        agent_name_arg,

        # nodes:
        laser_scan_matcher,
        covariance_filter_node, 
        ekf_node
    ])
