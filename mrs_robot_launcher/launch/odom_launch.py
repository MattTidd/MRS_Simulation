from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# what does this file launch?
# - laser_scan_matcher
# - imu/lidar covariance filter node
# - extended kalman filter

# method for generating the ekf params .yaml file:
def generate_ekf_config(agent_name : str, output_dir : str):
    # define the parameters:
    ekf_params = {
        f"{agent_name}/ekf_filter_node" : {
            "ros__parameters" : {
                # preamble settings:
                "frequency" : 30.0,
                "sensor_timeout" : 0.5,
                "two_d_mode" : True,
                "transform_time_offset" : 0.0,
                "transform_timeout" : 0.0,
                "print_diagnostics" : False,
                "debug" : False,
                "debug_out_file" : "robot_localization_debug.txt",
                "permit_corrected_publication" : False,
                "publish_acceleration" : False,
                "publish_tf" : True,

                # frame settings:
                "map_frame" : "map",
                "odom_frame" : "odom",
                "base_link_frame" : f"{agent_name}_base_link",
                "world_frame" : "odom",

                # sensor settings:
                "odom0" : "wheel_odom",
                "odom0_config" : [False, False, False,
                                  False, False, False, 
                                  True, True, False,
                                  False, False, False,
                                  False, False, False],
                "odom0_queue_size" : 10,
                "odom0_nodelay" : False,
                "odom0_differential" : False,
                "odom0_relative" : False,
                "odom0_pose_use_child_frame" : False,

                "odom1" : "lidar_odom_filtered",
                "odom1_config" : [True,  True,  False,
                                  False, False, False,
                                  True,  True,  False,
                                  False, False, True,
                                  False, False, False],
                "odom1_queue_size" : 5,
                "odom1_nodelay" : False,
                "odom1_differential" : False,
                "odom1_relative" : False,
                "odom1_pose_use_child_frame" : False,

                "imu0" : "imu_data_filtered",
                "imu0_config" : [False, False, False,
                                 False, False, True,
                                 False, False, False,
                                 False, False, True,
                                 False, False, False],
                "imu0_queue_size" : 10,
                "imu0_nodelay" : False,
                "imu0_differential" : False,
                "imu0_relative" : False,
                "imu0_remove_gravitational_acceleration" : True,

                # covariances:
                "use_control" : False,
                "process_noise_covariance" : [
                    1e-3,  0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   1e-3,  0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.06,   0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.03,   0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.03,   0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.06,   0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.05,    0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.05,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.04,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.01,   0.0,    0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.1,    0.0,    0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0,
                    0.0,   0.0,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.015
                ],
                                              
                "initial_estimate_covariance" : [
                    1e-9, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  1e-9,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    1e-9,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    1e-9,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    1e-9,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    1e-9,   0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    1e-9,   0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1e-9,   0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1e-9,   0.0,     0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    1e-9,    0.0,     0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     1e-9,    0.0,     0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     1e-9,    0.0,    0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     1e-9,   0.0,    0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    1e-9,   0.0,
                    0.0,  0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,     0.0,    0.0,    1e-9
                ]
            }
        }
    }

    # write these parameters to a file:
    path = os.path.join(output_dir, f"{agent_name}_ekf_params.yaml")
    with open(path, "w") as f:
        yaml.dump(ekf_params, f)

# define an opaque function so I can actually use the agent_name:
def setup(context, *args, **kwargs):
    # get agent_name:
    agent_name = LaunchConfiguration("agent_name").perform(context)

    # get the paths:
    pkg_path = get_package_share_directory("mrs_robot_launcher")
    ekf_config_path = os.path.join(pkg_path, "config", "ekfs")

    # generate the file:
    generate_ekf_config(agent_name = agent_name, output_dir = ekf_config_path)

    return []

def generate_launch_description():
    # set the required paths:
    pkg_path = get_package_share_directory("mrs_robot_launcher")
    ekf_config_path = os.path.join(pkg_path, "config", "ekfs")

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

        # opaque function:
        OpaqueFunction(function = setup),

        # nodes:
        laser_scan_matcher,
        covariance_filter_node, 
        ekf_node
    ])
