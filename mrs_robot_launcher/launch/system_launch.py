from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import numpy as np

"""
this file is the launch file for launching the MRS, wherein the user specifies lists of parameters for each agent in the system, 
which are then looped over and the template launch file is called for number of agents to be launched. in this regard, this file
launches:

- everything that is contained within "mrs_robot_launcher/launch/base_launch.py"
- gazebo
- system manager gui

"""

def generate_launch_description():
    # define the paths:
    pkg_path                =    get_package_share_directory("mrs_robot_launcher")
    bt_pkg_path             =    get_package_share_directory("mrs_bt_handler")
    template_path           =    PathJoinSubstitution([pkg_path, "launch", "base_launch.py"])
    bt_launch_path          =    PathJoinSubstitution([bt_pkg_path, "launch", "bt_launch.py"])
    gazebo_launch_path      =    PathJoinSubstitution([pkg_path, "launch", "gazebo_launch.py"])
    goal_launch_path        =    PathJoinSubstitution([pkg_path, "launch", "goal_launch.py"])
    bridge_path             =    PathJoinSubstitution([pkg_path, "config", "bridges", "system_bridge_parameters.yaml"])

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
        default_value = "world_1.sdf",
        description = "Name of the world to be loaded, defaulting to world_1.sdf"
    )
    
    # define parameters for launching agents:
    positions = [["-3.0", "-3.0"], ["-3.0", "3.0"]]  # in form [(x1, y1), (x2, y2)]
    agent_names         = ["agent1", "agent2"]
    agent_types         = ["typeA", "typeB"]
    agent_initial_xs    = [str(p[0]) for p in positions]
    agent_initial_ys    = [str(p[1]) for p in positions]
    agent_initial_yaws  = ["0.0", "0.0"]
    flattened_positions = [float(coord) for p in positions for coord in p]

    # should have a seperate goal manager node probably that provides pertinent information about the goal, 
    # and spawns and kills goals as information comes in

    num_agents = len(agent_names)
    a_types = np.random.permutation(agent_types)
    delay = 2.5
    templates = []
    bts = []

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
                                "agent_initial_y" : str(positions[agent][1])
                                }.items()
        )

        timed_bar = TimerAction(period = delay + float(agent * delay),
                                actions = [bar])
        
        # add instance of bt node to list of bt nodes:
        bts.append(timed_bar)

    # define the goal settings:
    goal_name = "goal"
    goal_type = np.random.choice(["typeA", "typeB"])
    goal_initial_x_pos = "0.0"
    goal_initial_y_pos = "0.0"

    # launch the goal:
    goal_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([goal_launch_path]),
        launch_arguments = {"use_sim_time" : use_sim_time,
                            "goal_name" : goal_name,
                            "goal_type" : goal_type,
                            "goal_initial_x_pos" : goal_initial_x_pos,
                            "goal_initial_y_pos" : goal_initial_y_pos}.items()
    )

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

    # bt manager GUI:
    bt_gui = Node(
        package = "system_gui",
        executable = "bt_gui_node",
        name = "bt_gui",
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
    ] + templates 
    + bts
    )
