# import packages:
from ament_index_python.packages import get_package_share_directory
from mrs_drl_interfaces.msg import Goal, AgentMetrics
from std_msgs.msg import String, Bool
from rclpy.node import Node
import numpy as np
import subprocess
import threading
import tempfile
import psutil
import random
import signal
import rclpy
import json
import yaml
import time
import sys
import csv
import re
import os

# gui-specific packages:
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QComboBox, QPushButton, QGroupBox, QLineEdit
from PyQt5.QtCore import QTimer, Qt, pyqtSignal 

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

# class for the main node:
class MissionGuiNode(Node):
    """
    Primary class for the ``MissionGuiNode``, which is responsible for hosting the 
    mission-queuing GUI.
    - Inherits from ``rclpy.node.Node``.
    """
    # constructor for node:
    def __init__(self):
        """
        Constructor for the node. Declares and adds parameters to the class, and instantiates subscribers/publishers.

        :param world_name: Name of the world to be used. This parameter is used for Gazebo service calls.
        :type world_name: string

        :param points_path: Path to the .JSON file containing coordinate points for spawning.
        :type points_path: string
        """
        # inherit from parent class:
        super().__init__("mission_gui_node")

        # add the GUI to the node:
        self.gui = None

        # display to the user when the node has started:
        self.get_logger().info("GUI node started")

        # declare parameters:
        self.declare_parameter("world_name", "world_3.sdf")
        self.declare_parameter("points_path", "")
        self.declare_parameter("drl_model", "SAC_001")

        # get parameter values:
        world_path     = self.get_parameter("world_name").value
        points_path    = self.get_parameter("points_path").value
        self.drl_model = self.get_parameter("drl_model").value

        # add parameters to the class:
        self.world_name = re.split(r'[/.]', world_path)[-2]
        with open(points_path) as f:
            data = json.load(f)
            self.goal_points  = data["goals"]
            self.agent_points = data["agents"]

        # establish subscribers:
        self.goal_sub    = self.create_subscription(Goal, "/goal", self._goal_callback, 10)
        self.reset_sub   = self.create_subscription(String, "/reset_agent", self._reset_agent_callback, 10)
        self.metrics_sub = self.create_subscription(AgentMetrics, "/agent_metrics", self._metrics_callback, 10)

        # establish publishers:
        self.goal_pub             = self.create_publisher(Goal, "/goal", 10)
        self.start_pub            = self.create_publisher(String, "/simulation_start", 10)
        self.reset_complete_pub   = self.create_publisher(Bool, "/reset_complete", 10)
        self.mission_complete_pub = self.create_publisher(String, "/mission_complete", 10)

        # define variables for storage:
        self.agent_metrics      = {}
        self.mission_start_time = None
        self.makespan           = 0
        self.reauction_count    = 0
        self.goals_completed    = 0

    # define a callback for the goal subscriber:
    def _goal_callback(self, msg: Goal):
        """
        Callback method used by the goal subscriber. Calls the ``_publish_next_goal()`` method of the GUI upon receiving
        an empty goal message.

        :param msg: Goal message that is subscribed to.
        :type msg: Goal
        """
        # if receiving an empty goal message:
        if msg.required_capability == "":
            self.goals_completed += 1
            self.gui._publish_next_goal()
    
    # define a callback for resetting agents:
    def _reset_agent_callback(self, msg: String):
        # increment the re-auction counter:
        self.reauction_count += 1

        # get the name of the agent to be reset:
        agent_name = msg.data

        # get the position of that agent:
        agent_pos = self.gui.agent_queue[agent_name][1]
        agent_yaw = self.gui.agent_queue[agent_name][2]
        qz = round(np.sin(agent_yaw / 2), 3)
        qw = round(np.cos(agent_yaw / 2), 3)

        # move the agent back to its spawn location:
        subprocess.Popen([
            "ign", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "1000",
            "--req", f"name: '{agent_name}' position: {{x: {agent_pos[0]}, y: {agent_pos[1]}, z: {0.0}}} orientation: {{x: {0.0}, y: {0.0}, z: {qz}, w: {qw}}}"
        ])

        # delete its odometry nodes, recall them:
        nodes = ["ekf_node", "covariance_filter_node", "rf2o_laser_odom"]
        for node in nodes:
            self.gui._kill_namespaced_node(namespace = agent_name, node = node)

        # recall the odometry launch file:
        self.odom_process = subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "odom_launch.py", f"agent_name:={agent_name}"],
                                                       stdout=subprocess.DEVNULL,
                                                       stderr=subprocess.DEVNULL)
        
        # add a fixed delay while the odometry loads:
        time.sleep(5.0)

        # formulate and publish reset complete message:
        msg = Bool()
        msg.data = True
        self.reset_complete_pub.publish(msg)

    # define a callback for receiving metrics:
    def _metrics_callback(self, msg: AgentMetrics):
        # populate own dict using metrics message:
        self.agent_metrics[msg.agent_name] = {
            "distance"   : msg.total_distance,
            "tasks"      : msg.load_history,
            "collisions" : msg.collisions,
            "timeouts"   : msg.timeouts
        }

        # dump metrics to CSV if all are in:
        if len(self.agent_metrics) == len(self.gui.agent_queue):
            self._write_metrics()

    # define a method for writing metrics to CSV:
    def _write_metrics(self):
        pass
        # instantiate a row:
        row = {
            "goals_completed" : self.goals_completed,
            "makespan"        : round(self.makespan, 3),
            "reauctions"      : self.reauction_count
        }

        # for every agent in the metrics dict:
        for agent_name, m in self.agent_metrics.items():
            # append to the row:
            row[f"{agent_name}_tasks"]      = m["tasks"]
            row[f"{agent_name}_distance"]   = round(m["distance"], 3)
            row[f"{agent_name}_collisions"] = m["collisions"]
            row[f"{agent_name}_timeouts"]   = m["timeouts"]

        # specify the path to write to:
        path = os.path.expanduser("mission_metrics.csv")
        write_header = not os.path.exists(path)

        # write the row to a file:
        with open(path, "a", newline = "") as f:
            # make writer:
            writer = csv.DictWriter(f, fieldnames = row.keys())
            
            # write the row:
            if write_header:
                writer.writeheader()
            writer.writerow(row)

        # log to user:
        self.get_logger().info(f"Metrics written to {path}")

# class for the actual GUI:
class MainWindow(QWidget):
    """
    Primary class for the ``MainWindow``, which contains the GUI. Responsible defining the layout of the elements within
    the GUI, as well as their functionalities. 
    - Inherits from ``PyQT5.QtWidgets.QWidget``.
    """
    # signal for buttons:
    button_handling = pyqtSignal()

    # constructor for the GUI:
    def __init__(self, node : Node):
        """
        Constructor for the GUI. Instantiates the components within the system, and defines their layout within the window. 
        Also connects the functionality for the resetting of buttons.
        """
        # inherit from parent class:
        super().__init__()

        # add the node to the GUI:
        self.node = node

        # flag for despawning:
        self.goal_number = 0

        # set empty dicts for queuing:
        self.goal_queue  = {}
        self.agent_queue = {}

        # set the title of the window:
        self.setWindowTitle("ROS2 MRS Mission GUI")

        # set the size of the GUI:
        self.setFixedWidth(600)
        self.setFixedHeight(300)

        # set a style sheet:
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size:   14px;
            }
            QComboBox {
                min-width: 100px;
                min-height: 20px;
            }
            QPushButton {
                padding: 6px;
                border-radius: 4px;
                background-color: #00B7FF;
                color: white;
                font-weight: bold;
                min-width: 100px;
                max-width: 200px;
                min-height: 20px;
                max-height: 20px;
            }         
            QPushButton:hover {
                background-color: #005fa3;
            }
            QLabel {
                font-weight: bold;
                font-size: 14px;
            }
            QLineEdit {
                min-width: 120px;
                min-height: 20px;
            }
        """)

        # main layout manager:
        main_layout = QVBoxLayout()
        main_layout.setSpacing(15)

        # instantiate child layouts:
        group0 = QGroupBox("Mission Settings")
        grid0  = QGridLayout()
        group0.setLayout(grid0)

        group1 = QGroupBox("Simulation Settings")
        grid1  = QGridLayout()
        group1.setLayout(grid1)

        ##### grid 0 - mission related settings: #####
        # add a typeable field for setting the number of agents:
        # self.agent_num_input = QLineEdit()
        # self.agent_num_input.setPlaceholderText("Number of Agents")
        # self.agent_num_input.setAlignment(Qt.AlignCenter)
        # grid0.addWidget(self.agent_num_input, 0, 0, alignment = Qt.AlignCenter) 

        # add a typeable field for setting the mission length:
        self.mission_length_input = QLineEdit()
        self.mission_length_input.setPlaceholderText("Mission Length")
        self.mission_length_input.setAlignment(Qt.AlignCenter)
        grid0.addWidget(self.mission_length_input, 0, 0, alignment = Qt.AlignCenter)

        # add a button for randomizing missions:
        self.randomize_mission_button = QPushButton("Randomize Mission")
        self.randomize_mission_button.clicked.connect(self._on_randomize_mission_clicked)
        grid0.addWidget(self.randomize_mission_button, 0, 1, alignment = Qt.AlignCenter)

        ##### grid 1 - simulation related settings: #####
        # add a button for starting the simulation:
        self.start_sim_button = QPushButton("Start Simulation")
        self.start_sim_button.clicked.connect(self._on_start_sim_clicked)
        grid1.addWidget(self.start_sim_button, 0, 0, alignment = Qt.AlignCenter)

        # add a button for clearing the simulation:
        self.clear_sim_button = QPushButton("Clear Simulation")
        self.clear_sim_button.clicked.connect(self._on_clear_sim_clicked)
        grid1.addWidget(self.clear_sim_button, 0, 1, alignment = Qt.AlignCenter)

        # add child layouts to main layout:
        main_layout.addWidget(group0)
        main_layout.addWidget(group1)

        # apply the layout:
        self.setLayout(main_layout)

        # connect signal for button handling:
        self.button_handling.connect(self._enable_buttons)

    # method for locking the buttons:
    def _lock_buttons(self):
        """
        This method locks the buttons, so that the user can not interact with them while a process runs.
        Locks and modifies the text of the buttons.
        """
        # lock all buttons:
        self.start_sim_button.setEnabled(False)
        self.randomize_mission_button.setEnabled(False)

        # modify the text of the buttons:
        self.start_sim_button.setText("Waiting...")
        self.randomize_mission_button.setText("Waiting...")

    # method for enabling the buttons:
    def _enable_buttons(self):
        """
        This method enables the buttons, so that they may be used by the user. Unlocks the buttons and 
        sets their text to their native values prior to being locked.
        """
        # unlock buttons:
        self.start_sim_button.setEnabled(True)
        self.randomize_mission_button.setEnabled(True)

        # modify the text of the buttons:
        self.start_sim_button.setText("Start Simulation")
        self.randomize_mission_button.setText("Randomize Mission")

    # method for pressing the randomize mission button:
    def _on_randomize_mission_clicked(self):
        """
        Method for when the randomize mission button has been hit. Locks all buttons on the GUI and instantiates another thread, 
        which calls the ``_randomize_mission_process()`` method.
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._randomize_mission_process, args = (), daemon = True).start()

    # method for randomize mission process:
    def _randomize_mission_process(self):
        """    
        Method responsible for queuing a randomized mission. This method is ran within its own thread. Samples a specified number of 
        randomized goal positions and requirements, agent positions and agent types, and appends these to their respective queues. 
        """
        # make sure the current agent and goal queues are clear:
        self.goal_queue.clear()
        self.agent_queue.clear()

        # define the types to be sampled:
        types = ["typeA", "typeB"]

        # extract mission information from the GUI:
        try:
            mission_length = int(self.mission_length_input.text())
            half_length    = int(mission_length / 2)
            num_agents     = 4
        except Exception as e:
            self.node.get_logger().info(f"Provided mission specifications are invalid: {e}!")
            time.sleep(0.5)
            self.button_handling.emit()
            return
        
        #  ensure that the user did not pass zero goals:
        if mission_length == 0:
            self.node.get_logger().info(f"Please provide a valid mission composition!")
            time.sleep(0.5)
            self.button_handling.emit()
            return
        
        # sample the required number of points for both the number of agents and goals:
        goals = random.sample(self.node.goal_points.copy(), k = mission_length)

        ##### GOAL QUEUING #####
        # for every goal that was sampled from the points:
        for goal_num, goal_points in enumerate(goals):
            # form the goal specifications:
            goal_name = f"goal_{len(self.goal_queue) + 1}"

            # alternate goal types:
            if goal_num % 2 == 0:
                goal_type = types[0]
            else:
                goal_type = types[1]

            # append goal to the queue:
            self.goal_queue[goal_name] = [goal_type, goal_points[0], goal_points[1]]

        ##### AGENT QUEUING #####
        # set the types:
        agent_types = random.sample(types, k = 4, counts = [2, 2])

        # for every agent position:
        for agent_num in range(num_agents):
            # form the agent specifications:
            agent_name = f"agent{agent_num + 1}"
            agent_type = agent_types[agent_num]
            agent_pos  = self.node.agent_points[agent_num]
            agent_yaw  = 0.0
            qz = round(np.sin(agent_yaw / 2), 3)
            qw = round(np.cos(agent_yaw / 2), 3)

            # add agent to the queue:
            self.agent_queue[agent_name] = [agent_type, agent_pos, agent_yaw, qz, qw]

        # ensure that there is a capability/requirement match:
        # goal_types_used = {v[0] for v in self.goal_queue.values()}
        # agent_names = list(self.agent_queue.keys())
        # for i, goal_type in enumerate(goal_types_used):
        #     self.agent_queue[agent_names[i]][0] = goal_type

        # re-enable the buttons:
        time.sleep(0.5)
        self.button_handling.emit()

    # method for pressing the start sim button:
    def _on_start_sim_clicked(self):
        """
        Method for when the start sim button has been hit. Locks all buttons on the GUI and instantiates another thread, 
        which calls the ``_start_sim_process()`` method.
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._start_sim_process, args = (), daemon = True).start()

    # method for start sim process:
    def _start_sim_process(self):
        """
        Method responsible for the starting the simultion. This method is ran within its own thread. Spawns all agents within the environment, 
        and then populates and publishes a message signalling that the simulation has started. This method then checks for any goals within
        the queue. If goal is found, the ``_publish_next_goal()`` method is called. If no goals are found, then nothing happens, and the
        buttons are re-enabled.
        """
        # call the spawn agents method:
        self._spawn_agents()

        # populate the start sim message:
        msg      = String()
        msg.data = "start"

        # publish the message:
        self.node.start_pub.publish(msg)

        # check for goals in queue, handle accordingly:
        if self.goal_queue:
            # log to user:
            self.node.get_logger().info("Goal detected in queue, publishing!\n\n")

            # trigger the goal publishing loop:
            self._publish_next_goal()
        else:
            # log to user:
            self.node.get_logger().info("No goals provided for the current mission!")

            # re-enable the buttons:
            time.sleep(0.5)
            self.button_handling.emit()

    # method for pressing the clear sim button:
    def _on_clear_sim_clicked(self):
        """
        Method for when the clear sim button has been hit. Locks all buttons on the GUI and instantiates another thread, 
        which calls the ``_clear_sim_process()`` method. 
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._clear_sim_process, args = (), daemon = True).start()

    # method for killing processes:
    def kill_process_tree(self, process):
        try: 
            parent = psutil.Process(process.pid)
            for child in parent.children(recursive = True):
                child.kill()
            parent.kill()
        except psutil.NoSuchProcess:
            pass

    # method for clear sim process:
    def _clear_sim_process(self):
        """
        Method for completely clearing the simulation. This entails deleting any active goals, if they exist, and deleting
        the nodes relating to each of the agents within the system. This method is ran within its own thread. 
        """
        # kill the gazebo process:
        self.kill_process_tree(process = self.gazebo_process)

        # kill the parameter bridge:
        self._kill_namespaced_node(namespace = "", node = "parameter_bridge")

        # need to specify a list of processes pertaining to a given agent:
        processes = ["bt_node", 
                     "covariance_filter_node",
                     "ekf_node",
                     "rf2o_laser_odometry_node",
                     "robot_state_publisher"]
        
        # loop over each of these nodes and kill them, for each agent:
        for agent in self.agent_queue.keys():
            # kill the nodes related to the agent:
            for node in processes:
                self._kill_namespaced_node(namespace = agent, node = node)

        # check for goal publisher:
        result = subprocess.run(["pgrep", "-f", "robot_state_pub.*__ns:=/goal"],
                                capture_output = True,
                                text = True)
        
        if result.stdout.strip():
            # kill the goal publisher if it exists
            self._kill_namespaced_node(namespace = "goal", node = "robot_state_pub")


        # need to now clear both the agent and goal queues:
        self.goal_queue.clear()
        self.agent_queue.clear()

        # re-enable the buttons:
        time.sleep(0.5)
        self.button_handling.emit()

    # method for spawning agents from the agent queue:
    def _spawn_agents(self):
        # launch gazebo:
        self.gazebo_process = subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "system_launch.py", 
                                                f"world:={self.node.world_name}.sdf"])
        
        # wait for gazebo to finish launching:
        time.sleep(5)

        # need to launch the ROS2-gazebo bridge:
        bridge_path = generate_bridge_config(agent_names = list(self.agent_queue.keys()))
        self.bridge_process = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                                "--ros-args", "-p", f"config_file:={bridge_path}"],
                                                stdout = subprocess.DEVNULL,
                                                stderr = subprocess.DEVNULL)
        
        # for every agent that has been added to the queue:
        for agent in self.agent_queue:
            # check to see that the agent has not yet been spawned:
            result = subprocess.run(["pgrep", "-f", f"robot_state_pub.*__ns:=/{agent}"],
                                    capture_output = True,
                                    text = True)
            
            # if the PID of the process exists:
            if result.stdout.strip():
                self.node.get_logger().info(f"{agent} spawned already!")
                pass
            else:
                # extract the agent specifications:
                agent_type, agent_pos, agent_yaw, _, _ = self.agent_queue[agent]

                # need to then launch the nodes corresponding to that agent:
                self.agent_process = subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "base_launch.py",
                                                       f"agent_name:={agent}",
                                                       f"agent_type:={agent_type}",
                                                       f"agent_initial_x_pos:={agent_pos[0]}",
                                                       f"agent_initial_y_pos:={agent_pos[1]}",
                                                       f"agent_initial_yaw:={agent_yaw}"])
                
                self.bt_process = subprocess.Popen(["ros2", "launch", "mrs_bt_handler", "bt_launch.py",
                                                    f"agent_name:={agent}",
                                                    f"agent_type:={agent_type}",
                                                    f"drl_model:={self.node.drl_model}",
                                                    f"num_agents:={len(self.agent_queue)}",
                                                    f"agent_initial_x:={agent_pos[0]}",
                                                    f"agent_initial_y:={agent_pos[1]}",
                                                    f"agent_initial_yaw:={agent_yaw}"
                                                    ])
                
                # add a slight delay between spawns:
                time.sleep(2.5)

    # method for publishing goals:
    def _publish_next_goal(self):
        """
        Method responsible for publishing the next goal. First, it checks if the goal number is not zero. If so, the method logs that 
        the goal has been complete, deletes the previous goal using the ``_kill_namespaced_node()`` method, and removes the goal from the 
        simulation using a subprocess Gazebo service call.

        If the queue is not empty, this method then pops the first item from the queue, and spawns it within the environment using a 
        subprocess Gazebo service call. Following this, a goal message is built and published on the ``/goal`` topic, for agents to auction
        for.
        """
        # start timer on mission start:
        if self.goal_number == 0:
            self.node.mission_start_time = time.time()
            
        # if it is not the first goal:
        if self.goal_number != 0:
            # kill the previous goal:
            self._kill_namespaced_node(namespace = "goal", node = "robot_state_pub")

            # despawn the goal body in the simulation:
            subprocess.run(["ign", "service", "-s", f"/world/{self.node.world_name}/remove",
                        "--reqtype", "ignition.msgs.Entity",
                        "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "2000",
                        "--req", f"name: 'goal' type: MODEL"],
                        stdout = subprocess.DEVNULL,
                        stderr = subprocess.DEVNULL)
        
        # if the goal queue is empty:
        if not self.goal_queue:
            # log to user:
            self.node.get_logger().info("Goal queue is empty, current simulation is complete!")

            # get the makespan:
            self.node.makespan = time.time() - self.node.mission_start_time

            # signal agents to publish their metrics:
            msg      = String()
            msg.data = "complete"
            self.node.mission_complete_pub.publish(msg)

            # reset the goal counter:
            self.goal_number = 0

            # re-enable buttons:
            time.sleep(0.5)
            self.button_handling.emit()
            return
        
        # otherwise, pop the first item from the queue:
        key = next(iter(self.goal_queue))
        goal_data = self.goal_queue.pop(key)

        # spawn this goal within the environment:
        subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "goal_launch.py", 
                          "use_sim_time:=true",
                          "goal_name:=goal", 
                          f"goal_type:={goal_data[0]}",
                          f"goal_initial_x_pos:={goal_data[1]}",
                          f"goal_initial_y_pos:={goal_data[2]}"],
                          stdout = subprocess.DEVNULL,
                          stderr = subprocess.DEVNULL)
        
        # increment the goal_number:
        self.goal_number += 1

        # build and publish a goal message:
        msg                         = Goal()
        msg.pose.header.stamp       = self.node.get_clock().now().to_msg()
        msg.id                      = key
        msg.required_capability     = goal_data[0]
        msg.pose.pose.position.x    = goal_data[1]
        msg.pose.pose.position.y    = goal_data[2]
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.w = 1.0

        # publish:
        self.node.goal_pub.publish(msg)
        print(f"Goal published at: ({goal_data[1]}, {goal_data[2]}) with type: {goal_data[0]}!")

    # method for killing nodes:
    def _kill_namespaced_node(self, namespace : str, node):
        """
        Method for killing namespaced nodes. Does this by running a ``"kill"`` subprocess on the ``PID`` of the
        node process. Accepts single nodes, with the idea being that the user loops over a list of nodes, killing
        a singular node each time.

        :param namespace: Namespace used by the node to be killed. 
        :type namespace: str

        :param node: Individual node to be killed.
        :type node: str
        """
        # handle both namespaced and non-namespaced nodes:
        pattern = f"{node}.*__ns:=/{namespace}" if namespace else node

        # pull the PID of the node:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output = True,
            text = True
        )

        # pull the PID:
        pids = result.stdout.strip().split()

        # if that PID exists, kill it:
        if pids:
            subprocess.run(["kill", "-9"] + pids)
            time.sleep(0.1)

# define main execution of node:
def main():
    # start the GUI:
    app = QApplication(sys.argv)

    # initialize rclpy:
    rclpy.init()

    # instantiate the node:
    node = MissionGuiNode()

    # spin ROS2 in a background thread so it doesn't block the GUI:
    ros_thread = threading.Thread(target = rclpy.spin, args = (node, ), daemon = True)
    ros_thread.start()

    # instantiate the window, and add it to the node:
    window   = MainWindow(node = node)
    node.gui = window
    
    # display the GUI:
    window.show()

    # allow python to read signals every 500ms:
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda : None)

    # handle shutdown of node and GUI:
    signal.signal(signal.SIGINT, lambda *args: app.quit())
    exit_code = app.exec_()
    node.destroy_node()
    time.sleep(1)
    rclpy.shutdown()
    sys.exit(exit_code)

# main:
if __name__ == "__main__":
    main()