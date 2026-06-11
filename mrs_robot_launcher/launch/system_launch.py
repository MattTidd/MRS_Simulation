from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import numpy as np
import yaml
import tempfile
import os

"""
this file is the launch file for launching the MRS, wherein the user specifies lists of parameters for each agent in the system, 
which are then looped over and the template launch file is called for number of agents to be launched. in this regard, this file
launches:

- everything that is contained within "mrs_robot_launcher/launch/base_launch.py"
- gazebo
- system manager gui

"""

# method for generating the bridge parameters .yaml file:
def generate_bridge_config(agent_names : list) -> str:
    # initialize an empty list to hold parameters:
    bridge_params = []

    # add the clock bridge, which is always present:
    bridge_params.append({
        "direction" : "GZ_TO_ROS",
        "gz_topic_name" : "clock",
        "gz_type_name" : "gz.msgs.Clock",
        "ros_topic_name" : "clock",
        "ros_type_name" : "rosgraph_msgs/msg/Clock"
    })

    # need to add the bridged sensor topics:
    for agent in agent_names:
        bridge_params += [
            # add the cmd_vel port:
            {
                "direction" : "ROS_TO_GZ",
                "gz_topic_name" : f"{agent}_cmd_vel",
                "gz_type_name" : "gz.msgs.Twist",
                "ros_topic_name" : f"{agent}/cmd_vel",
                "ros_type_name" : "geometry_msgs/msg/TwistStamped"
            }, 
            # add the scan port:
            {
                "direction" : "GZ_TO_ROS",
                "gz_topic_name" : f"{agent}_scan",
                "gz_type_name" : "gz.msgs.LaserScan",
                "ros_topic_name" : f"{agent}/scan",
                "ros_type_name" : "sensor_msgs/msg/LaserScan"
            },
            # add the scan points port:
            {
                "direction" : "GZ_TO_ROS",
                "gz_topic_name" : f"{agent}_scan/points",
                "gz_type_name" : "gz.msgs.PointCloudPacked",
                "ros_topic_name" : f"{agent}/scan/points",
                "ros_type_name" : "sensor_msgs/msg/PointCloud2"
            },
            # add the IMU port:
            {
                "direction" : "GZ_TO_ROS",
                "gz_topic_name" : f"{agent}_imu_data",
                "gz_type_name" : "gz.msgs.IMU",
                "ros_topic_name" : f"{agent}/imu_data",
                "ros_type_name" : "sensor_msgs/msg/Imu"
            }
        ]

    # write these parameters to a file and return its path:
    tmp = tempfile.NamedTemporaryFile(mode = "w", suffix = ".yaml", delete = False)
    yaml.dump(bridge_params, tmp)
    tmp.close()

    # return file path to user:
    return tmp.name

def generate_launch_description():
    # define the paths:
    pkg_path           = get_package_share_directory("mrs_robot_launcher")
    bt_pkg_path        = get_package_share_directory("mrs_bt_handler")
    gui_path           = get_package_share_directory("system_gui")
    template_path      = PathJoinSubstitution([pkg_path, "launch", "base_launch.py"])
    bt_launch_path     = PathJoinSubstitution([bt_pkg_path, "launch", "bt_launch.py"])
    gazebo_launch_path = PathJoinSubstitution([pkg_path, "launch", "gazebo_launch.py"])
    goal_launch_path   = PathJoinSubstitution([pkg_path, "launch", "goal_launch.py"])
    points_path        = os.path.join(gui_path, "locations", "points.JSON")

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
        default_value = "world_3.sdf",
        description = "Name of the world to be loaded, defaulting to world_3.sdf"
    )
    
    # define parameters for launching agents:
    # positions           = [["-3.0", "-3.0"], ["-3.0", "3.0"], ["3.0", "3.0"], ["3.0", "-3.0"]]
    positions           = [["-9.0", "-5.0"], ["-6.0", "6.0"], ["1.0", "5.0"], ["1.0", "-4.0"]]
    agent_names         = ["agent1", "agent2", "agent3", "agent4"]
    agent_types         = ["typeA", "typeB", "typeA", "typeB"]
    agent_initial_xs    = [str(p[0]) for p in positions]
    agent_initial_ys    = [str(p[1]) for p in positions]
    agent_initial_yaws  = ["0.785398", "-0.785398", "-2.356194", "2.356194"]
    flattened_positions = [float(coord) for p in positions for coord in p]
    flattened_yaws      = [float(y) for y in agent_initial_yaws]

    num_agents = len(agent_names)
    a_types    = np.random.permutation(agent_types)
    delay      = 2.5

    templates = []
    bts       = []

    # loop over the number of agents:
    for agent in range(num_agents):
        # pick a random type:
        a_type = a_types[agent]

        # include the base_template:
        foo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([template_path]),
            launch_arguments = {"use_sim_time" : use_sim_time,
                                "visualize" : visualize, 
                                "agent_name" : agent_names[agent],
                                "agent_type" : a_type,
                                "agent_initial_x_pos" : agent_initial_xs[agent],
                                "agent_initial_y_pos" : agent_initial_ys[agent],
                                "agent_initial_yaw" : agent_initial_yaws[agent]}.items()
        )

        timed_foo = TimerAction(period = delay + float(agent * delay), 
                                actions = [foo])

        # add instance of template to list of templates:
        templates.append(timed_foo)

        # include the BT node per agent:
        bar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bt_launch_path]),
            launch_arguments = {"agent_name" : agent_names[agent],
                                "agent_type" : a_type,
                                "num_agents" : str(num_agents),
                                "agent_initial_x" : str(positions[agent][0]),
                                "agent_initial_y" : str(positions[agent][1]),
                                "agent_initial_yaw" : str(agent_initial_yaws[agent])
                                }.items()
        )

        timed_bar = TimerAction(period = delay + float(agent * delay),
                                actions = [bar])
        
        # add instance of bt node to list of bt nodes:
        bts.append(timed_bar)

    # launch gazebo:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments = {"world" : world}.items()
    )

    # use a single ros bridge node:
    bridge_path = generate_bridge_config(agent_names = agent_names)

    ros_gz_bridge = Node(
        package = "ros_gz_bridge",
        executable = "parameter_bridge",
        name = "ros_gz_bridge",
        arguments = ["--ros-args", "-p", ["config_file:=", bridge_path]]
    )

    # bt manager GUI:
    bt_gui = Node(
        package = "system_gui",
        executable = "bt_gui_node",
        name = "bt_gui",
        parameters = [{
            "agent_names"       : agent_names,
            "positions"         : flattened_positions,
            "agent_initial_yaw" : flattened_yaws,
            "world_name"        : world,
            "points_path"       : points_path
        }]
    )

    delayed_gui = TimerAction(period = delay + float(num_agents * delay),
                              actions = [bt_gui])

    return LaunchDescription([
        use_sim_time_arg, 
        visualize_arg, 
        world_arg,  
        gazebo, 
        ros_gz_bridge,
        delayed_gui,
    ] 
    + templates 
    + bts
    )
