# import packages:
import re
import sys
import tempfile
import yaml
import time
import threading
import signal
import subprocess
import rclpy
import numpy as np
from rclpy.node import Node
from mrs_drl_interfaces.msg import Goal
from std_msgs.msg import String, Bool

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
class GuiNode(Node):
    """
    Primary class for the ``GuiNode``, which is responsible for hosting the GUI.
    - Inherits from ``rclpy.node.Node``.
    """
    # constructor for node:
    def __init__(self):
        """
        Constructor for the node. Declares and adds parameters to the class, and instantiates subscribers/publishers. 

        :param world_name: Name of the world to be used. This parameter is used for Gazebo service calls.
        :type world_name: string
        
        """
        # inherit from parent class:
        super().__init__("bt_gui_node")

        # add gui to the node:
        self.gui = None

        # display to user when node has started:
        self.get_logger().info("GUI node started")

        # declare parameters:
        self.declare_parameter("world_name", "world_3.sdf")

        # add parameters to the class:
        world_path           = self.get_parameter("world_name").value
        self.world_name      = re.split(r'[/.]', world_path)[-2]
    
        # establish subscribers:
        self.goal_sub  = self.create_subscription(Goal, "/goal", self._goal_callback, 10)
        self.reset_sub = self.create_subscription(String, "/reset_agent", self._reset_agent_callback, 10)

        # establish publishers:
        self.goal_pub           = self.create_publisher(Goal, "/goal", 10)
        self.start_pub          = self.create_publisher(String, "/simulation_start", 10)
        self.reset_complete_pub = self.create_publisher(Bool, "/reset_complete", 10)

    # define a callback for the goal subscriber:
    def _goal_callback(self, msg : Goal):
        """
        Callback method used by the goal subscriber. Calls the ``_publish_next_goal()`` method of the GUI upon receiving 
        an empty goal message. 

        :param msg: Goal message that is subscribed to. 
        :type msg: Goal
        """
        # if receiving an empty goal message:
        if msg.required_capability == "":
            self.gui._publish_next_goal()

    # define a callback for resetting agents:
    def _reset_agent_callback(self, msg : String):
        """
        Callback method used by the reset agent subscriber. Extracts the desired agent from the message, 
        gets its position within space, and then moves the agent back to that spawn location. It then deletes the 
        odometry nodes of that agent, and recalls them. After waiting a fixed delay amount, it then formulates and 
        publishes a reset complete message.

        :param msg: String message that is subscribed to.
        :type msg: String
        """
        # get the name of the agent to be reset:
        agent_name = msg.data

        # get the positon of that agent:
        agent_pos_x = self.gui.agent_queue[agent_name][1]
        agent_pos_y = self.gui.agent_queue[agent_name][2]
        qz = round(np.sin(0.0 / 2), 3)
        qw = round(np.cos(0.0 / 2), 3)

        # move the agent back to its spawn location:
        subprocess.Popen([
            "ign", "service", "-s", f"/world/{self.world_name}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "1000",
            "--req", f"name: '{agent_name}' position: {{x: {agent_pos_x}, y: {agent_pos_y}, z: {0.0}}} orientation: {{x: {0.0}, y: {0.0}, z: {qz}, w: {qw}}}"
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

# class for the actual GUI:
class MainWindow(QWidget):
    """
    Primary class for the ``MainWindow``, which contains the GUI. Responsible for defining the layout of the elements within
    the GUI, as well as their functionalities. 
    - Inherits from ``PyQT5.QtWidgets.QWidget``.
    """
    # signal for buttons:
    button_handling = pyqtSignal()

    # constructor for GUI:
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
        self.goal_queue = {}
        self.agent_queue = {}

        # set the title of window:
        self.setWindowTitle("ROS2 MRS GUI")

        # set the size of the GUI:
        self.setFixedWidth(600)
        self.setFixedHeight(400)

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

        # instantiate the child layouts:
        group0 = QGroupBox("Agent Settings:")
        grid0  = QGridLayout()
        group0.setLayout(grid0)

        group1 = QGroupBox("Goal Settings:")
        grid1  = QGridLayout()
        group1.setLayout(grid1)

        group2 = QGroupBox("Simulation Settings:")
        grid2  = QGridLayout()
        group2.setLayout(grid2)

        ##### grid 0 - agent related settings: #####
        # add a typeable field for setting the agent name:
        self.agent_name_input = QLineEdit()
        self.agent_name_input.setPlaceholderText("Agent Name")
        self.agent_name_input.setAlignment(Qt.AlignCenter)
        grid0.addWidget(self.agent_name_input, 0, 0, alignment = Qt.AlignCenter)

        # add a combo box for setting the agent type:
        self.agent_type_combo_box = QComboBox()
        self.agent_type_combo_box.setEditable(True)
        self.agent_type_combo_box.lineEdit().setAlignment(Qt.AlignCenter)
        self.agent_type_combo_box.lineEdit().setReadOnly(True)
        self.agent_type_combo_box.addItem("typeA")
        self.agent_type_combo_box.addItem("typeB")
        grid0.addWidget(self.agent_type_combo_box, 1, 0, alignment = Qt.AlignCenter)

        # add a typeable field for setting the agent x position:
        self.agent_x_pos = QLineEdit()
        self.agent_x_pos.setPlaceholderText("X Position")
        self.agent_x_pos.setAlignment(Qt.AlignCenter)
        grid0.addWidget(self.agent_x_pos, 0, 1, alignment = Qt.AlignCenter)

        # add a typeable field for setting the agent y position:
        self.agent_y_pos = QLineEdit()
        self.agent_y_pos.setPlaceholderText("Y Position")
        self.agent_y_pos.setAlignment(Qt.AlignCenter)
        grid0.addWidget(self.agent_y_pos, 1, 1, alignment = Qt.AlignCenter)

        # add a button for queuing the agent:
        self.agent_queue_button = QPushButton("Queue Agent")
        self.agent_queue_button.clicked.connect(self._on_agent_queue_clicked)
        grid0.addWidget(self.agent_queue_button, 0, 2, alignment = Qt.AlignCenter)

        # add a button for spawning all agents:
        self.agent_spawn_button = QPushButton("Spawn Agents")
        self.agent_spawn_button.clicked.connect(self._on_agent_spawn_clicked)
        grid0.addWidget(self.agent_spawn_button, 1, 2, alignment = Qt.AlignCenter)

        ##### grid 1 - goal related settings: #####
        # need to have a combo box for selecting the goal type:
        self.goal_type_combo_box = QComboBox()
        self.goal_type_combo_box.setEditable(True)
        self.goal_type_combo_box.lineEdit().setAlignment(Qt.AlignCenter)
        self.goal_type_combo_box.lineEdit().setReadOnly(True)

        # TODO: dynamically pull the type of the agents and populate combo-box:
        self.goal_type_combo_box.addItem("typeA")
        self.goal_type_combo_box.addItem("typeB")

        # add combo box to the grid:
        grid1.addWidget(self.goal_type_combo_box, 0, 0, 2, 1, alignment = Qt.AlignCenter)

        # add an entry field for the goal x-position:
        self.x_input = QLineEdit()
        self.x_input.setPlaceholderText("X Position")
        self.x_input.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self.x_input, 0, 1, alignment = Qt.AlignCenter)

        # add an entry field for the goal y-position:
        self.y_input = QLineEdit()
        self.y_input.setPlaceholderText("Y Position")
        self.y_input.setAlignment(Qt.AlignCenter)
        grid1.addWidget(self.y_input, 1, 1, alignment = Qt.AlignCenter)

        # add a button for queueing goals:
        self.queue_goal_button = QPushButton("Queue Goal")
        self.queue_goal_button.clicked.connect(self._on_queue_goal_clicked)
        grid1.addWidget(self.queue_goal_button, 0, 3, 2, 1, alignment = Qt.AlignCenter)

        ##### grid 2 - simulation related settings: #####
        # add a button for resetting the simulation:
        self.reset_sim_button = QPushButton("Reset Simulation")
        self.reset_sim_button.clicked.connect(self._on_reset_sim_clicked)
        grid2.addWidget(self.reset_sim_button, 0, 0, alignment = Qt.AlignCenter)

        # add a button for starting the simulation:
        self.start_sim_button = QPushButton("Start Simulation")
        self.start_sim_button.clicked.connect(self._on_start_sim_clicked)
        grid2.addWidget(self.start_sim_button, 0, 2, alignment = Qt.AlignCenter)

        # add child layouts to main layout:
        main_layout.addWidget(group0)
        main_layout.addWidget(group1)
        main_layout.addWidget(group2)

        # apply the layout:
        self.setLayout(main_layout)

        # connect signal for button handling:
        self.button_handling.connect(self._enable_buttons)

    # method for locking the buttons:
    def _lock_buttons(self):
        """
        Method for locking the buttons contained within the GUI, thus preventing users from hitting them while a 
        process runs. Locks and modifies the text of buttons. 
        """
        # lock all buttons:
        self.queue_goal_button.setEnabled(False)
        self.agent_spawn_button.setEnabled(False)
        self.agent_queue_button.setEnabled(False)
        self.start_sim_button.setEnabled(False)

        # modify text of buttons:
        self.queue_goal_button.setText("Waiting...")
        self.agent_spawn_button.setText("Waiting...")
        self.agent_queue_button.setText("Waiting...")
        self.start_sim_button.setText("Waiting...")

    # method for enabling the buttons:
    def _enable_buttons(self):
        """
        Method for enabling the buttons contained within the GUI, thus allowing users to interact with them. Unlocks the buttons
        and sets their text to their native values prior to being locked.
        """
        # unlock buttons:
        self.queue_goal_button.setEnabled(True)
        self.agent_spawn_button.setEnabled(True)
        self.agent_queue_button.setEnabled(True)
        self.start_sim_button.setEnabled(True)

        # modify the text of the buttons:
        self.queue_goal_button.setText("Publish Goal")
        self.agent_spawn_button.setText("Spawn Agents")
        self.agent_queue_button.setText("Queue Agent")
        self.start_sim_button.setText("Start Simulation")

    # method for queuing goals:
    def _on_queue_goal_clicked(self):
        """
        Method for when the queue goal button has been hit. Locks all buttons on the GUI and instantiates another thread, 
        which calls the ``_goal_queue_process()`` method.
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._goal_queue_process, args = (), daemon = True).start()

    # method for goal queue process:
    def _goal_queue_process(self):
        """
        Method responsible for the actual queuing of goals. This method is ran within its own thread. Extracts values 
        related to the desired goal, verifies that they are correct, and then adds that goal to a goal queue dictionary, before 
        unlocking the buttons of the GUI.
        """
        # print to the user:
        self.node.get_logger().info(f"Adding goal to queue...")

        # extract the values related to the goal: 
        x = self.x_input.text()
        y = self.y_input.text()
        goal_type = self.goal_type_combo_box.currentText()

        # ensure that these values are their correct typings:
        try: 
            x = float(x)
            y = float(y)
        # catch the exception on value typing:
        except Exception as e:
            self.node.get_logger().info(f"Provided goal pose is invalid: {e}")

            # perform the re-enable before returning:
            time.sleep(0.5)
            self.button_handling.emit()
            return

        # add current goal into the goal queue dictionary:
        self.goal_queue[f"goal_{len(self.goal_queue) + 1}"] = [goal_type, x, y]

        # re-enable buttons:
        time.sleep(0.5)
        self.button_handling.emit()

    # method for queuing agents:
    def _on_agent_queue_clicked(self):
        """
        Method for when the agent queue button has been hit. Locks all buttons on the GUI and instantiates another thread, which calls
        the ``_agent_queue_process()`` method. 
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._agent_queue_process, args = (), daemon = True).start()

    # method for agent queue process:
    def _agent_queue_process(self):
        """
        Method responsible for the actual queuing of agents. This method is ran within its own thread. Extracts values 
        related to the agent, verifies that they are correct, and then adds that agent to an agent queue dictionary, before 
        unlocking the buttons of the GUI.
        """
        # print to the user:
        self.node.get_logger().info(f"Adding agent to queue...")

        # extract the values related to the agent:
        agent_x_pos = self.agent_x_pos.text()
        agent_y_pos = self.agent_y_pos.text()
        agent_name  = self.agent_name_input.text()
        agent_type  = self.agent_type_combo_box.currentText()

        # ensure that these values are their correct typings:
        try: 
            agent_x_pos = float(agent_x_pos)
            agent_y_pos = float(agent_y_pos)
        # catch the exception on value typing:
        except Exception as e:
            self.node.get_logger().info(f"Provided agent pose is invalid: {e}")

            # perform the re-enable before returning:
            time.sleep(0.5)
            self.button_handling.emit()
            return
        
        # add agent to agent queue:
        self.agent_queue[agent_name] = [agent_type, agent_x_pos, agent_y_pos]

        # re-enable buttons:
        time.sleep(0.5)
        self.button_handling.emit()

    # method for spawning agents:
    def _on_agent_spawn_clicked(self):
        """
        Method for when the agent spawn button has been hit. Locks all buttons on the GUI and instantiates another thread, which calls
        the ``_agent_spawn_process()`` method. 
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._agent_spawn_process, args = (), daemon = True).start()

    # method for agent spawn process:
    def _agent_spawn_process(self):
        """
        Method responsible for the actual spawning of agents. This method is ran within its own thread. Pops agents from the 
        agent queue, and spawns them within the environment, launching their odometry and BT nodes in the process, before 
        unlocking the buttons.
        """
        # need to start the ROS2-gazebo bridge:
        bridge_path = generate_bridge_config(agent_names = list(self.agent_queue.keys()))
        self.bridge_process = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                                "--ros-args", "-p", f"config_file:={bridge_path}"], 
                                                stdout = subprocess.DEVNULL,
                                                stderr = subprocess.DEVNULL)
        
        # for every agent that has been added to the queue:
        for agent in self.agent_queue:
            # check to see that agent has not yet been spawned:
            result = subprocess.run(["pgrep", "-f", f"robot_state_pub.*__ns:=/{agent}"],
                                    capture_output = True,
                                    text = True)
            
            # get the PID of the process:
            pid = result.stdout.strip()

            # if the process exists:
            if pid:
                self.node.get_logger().info(f"{agent} spawned already!")
                pass
            else:
                # get the type, x position, and y position of the agent:
                agent_type, agent_x_pos, agent_y_pos = self.agent_queue[agent]

                # need to then launch the nodes corresponding to that agent:
                self.agent_process = subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "base_launch.py",
                                                        f"agent_name:={agent}",
                                                        f"agent_type:={agent_type}",
                                                        f"agent_initial_x_pos:={agent_x_pos}",
                                                        f"agent_initial_y_pos:={agent_y_pos}",
                                                        f"agent_initial_yaw:={0.0}"])
                
                self.bt_process = subprocess.Popen(["ros2", "launch", "mrs_bt_handler", "bt_launch.py",
                                                    f"agent_name:={agent}",
                                                    f"agent_type:={agent_type}",
                                                    f"num_agents:={len(self.agent_queue)}",
                                                    f"agent_initial_x:={agent_x_pos}",
                                                    f"agent_initial_y:={agent_y_pos}",
                                                    f"agent_initial_yaw:={0.0}"])
                
                # add a slight delay between spawns:
                time.sleep(2.5)
            
        # re-enable buttons:
        time.sleep(0.5)
        self.button_handling.emit()

    # method for killing nodes:
    def _kill_namespaced_node(self, namespace : str, node = None):
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

    # method for pressing the reset sim button:
    def _on_reset_sim_clicked(self):
        """
        Method for when the reset sim button has been hit. Locks all buttons on the GUI and instantiates another thread, 
        which calls the ``_reset_sim_process()`` method. 
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._reset_sim_process, args = (), daemon = True).start()

    # method for reset sim process:
    def _reset_sim_process(self):
        """
        Method responsible for the actual resetting of the simulation. This method is ran within its own thread. Extracts the 
        positions of each agent, and moves the agent to its initial pose using a subprocess Gazebo service call. The 
        odometric nodes of the agent are then killed using the ``_kill_namespaced_node()`` method, and are relaunched. The goal is 
        also killed using the ``_kill_namespaced_node()`` method, and removed from the simulation using a subprocess Gazebo service call.
        Finally, the goal queue is cleared and the buttons are re-enabled. 

        :param agent_name: Name of the agent to be reset.
        :type agent_name: str
        """
        ##### SEND GLOBAL RESET SIGNAL #####
        # formulate and send the reset message:
        msg      = String()
        msg.data = "reset"
        self.node.start_pub.publish(msg)
        time.sleep(1)

        ##### RESET THE POSITIONS OF EACH AGENT #####
        # if there are no agents:
        if not self.agent_queue:
            self.node.get_logger().info("No agents to be reset!")
        else:
            # for each agent in the queue:
            for agent in self.agent_queue:
                # get the x and y position of the agent:
                _, agent_x_pos, agent_y_pos = self.agent_queue[agent]

                # move the position of the agent:
                subprocess.run(["ign", "service", "-s", f"/world/{self.node.world_name}/set_pose",
                                "--reqtype", "ignition.msgs.Pose",
                                "--reptype", "ignition.msgs.Boolean",
                                "--timeout", "2000",
                                "--req", f"name: '{agent}', position: {{x: {agent_x_pos}, y: {agent_y_pos}, z: {0.0}}}, orientation: {{x: {0.0}, y: {0.0}, z: {0.0}, w: {0.0}}}"],
                                stdout = subprocess.DEVNULL,
                                stderr = subprocess.DEVNULL)

                # reset the odometry of that agent:
                nodes = ["ekf_node", "covariance_filter_node", "rf2o_laser_odom"]
                for node in nodes:
                    self._kill_namespaced_node(namespace = agent, node = node)

                # call the launch file for the odometry nodes:
                self.odom_process = subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "odom_launch.py", f"agent_name:={agent}"],
                                                       stdout=subprocess.DEVNULL,
                                                       stderr=subprocess.DEVNULL)
        
        ##### RESET THE GOAL, IF IT EXISTS #####
        # check for goal publisher:
        result = subprocess.run(["pgrep", "-f", "robot_state_pub.*__ns:=/goal"],
                                capture_output = True,
                                text = True)
        
        # if there is a process:
        if result.stdout.strip():
            # kill the goal publisher if it exists
            self._kill_namespaced_node(namespace = "goal", node = "robot_state_pub")

            # remove the goal from Gazebo:
            subprocess.run(["ign", "service", "-s", f"/world/{self.node.world_name}/remove",
                            "--reqtype", "ignition.msgs.Entity",
                            "--reptype", "ignition.msgs.Boolean",
                            "--timeout", "2000",
                            "--req", f"name: 'goal' type: MODEL"],
                            stdout = subprocess.DEVNULL,
                            stderr = subprocess.DEVNULL)

        # re-enable the buttons:
        time.sleep(0.5)
        self.button_handling.emit()

    # # method for pressing the randomize mission buttom:
    # def _on_randomize_mission_clicked(self):
    #     """
    #     Method for when the randomize mission button has been hit. Locks all buttons on the GUI and instantiates another thread,
    #     which calls the ``_randomize_mission_process()`` method.
    #     """
    #     # lock buttons:
    #     self._lock_buttons()

    #     # use another thread to call the button execution:
    #     threading.Thread(target = self._randomize_mission_process, args = (), daemon = True).start()

    # # method for randomize mission process:
    # def _randomize_mission_process(self):
    #     """
    #     Method responsible for queuing a randomized mission. This method is ran within its own thread. Samples ten randomized goal positions and requirements,
    #     and then appends these to the goal queue.
    #     """
    #     # clear the goal queue:
    #     self.goal_queue.clear()

    #     # goal types:
    #     goal_types = ["typeA", "typeB"]

    #     # sample ten random goal positions from the set of possible positions:
    #     points = random.sample(self.node.goal_points, 3 + len(self.node.names))
    #     goals  = points[0:3]
    #     agents = points[3:]
        
    #     # for every goal that was sampled from the points:
    #     for goal in goals:
    #         # form a goal:
    #         goal_type = random.choice(goal_types)
    #         x = goal[0]
    #         y = goal[1]

    #         # append to the queue:
    #         self.goal_queue[f"goal_{len(self.goal_queue) + 1}"] = [goal_type, x, y]

    #     # # for every agent in the system:
    #     # for agent, _ in enumerate(self.node.names):
    #     #     # get the position of the agent:
    #     #     self.node.get_logger().info(f"agent is: {agent} | type: {type(agent)}\n")
    #     #     name = self.node.names[agent]
    #     #     pos  = agents[agent]
    #     #     yaw  = self.agent_yaws[name]
    #     #     x, y = pos[0], pos[1]
    #     #     qz = np.sin(yaw / 2)
    #     #     qw = np.cos(yaw / 2)

    #     #     # overwrite current positions of agent in dict for future resetting:
    #     #     self.node.agent_positions[name] = pos

    #     #     # move the agent to the desired location:
    #     #     subprocess.run(["ign", "service", "-s", f"/world/{self.node.world_name}/set_pose",
    #     #                 "--reqtype", "ignition.msgs.Pose",
    #     #                 "--reptype", "ignition.msgs.Boolean",
    #     #                 "--timeout", "2000",
    #     #                 "--req", f"name: '{name}', position: {{x: {x}, y: {y}, z: {0.0}}}, orientation: {{x: {0.0}, y: {0.0}, z: {qz}, w: {qw}}}"])
            
    #     #     # kill the nodes related to the odometry of that agent:
    #     #     self._kill_namespaced_node(namespace = name)

    #     #     # call the launch file for the odometry nodes:
    #     #     self.odom_process = subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "odom_launch.py", f"agent_name:={name}"])

    #     # let user know that a mission has been formed:
    #     self.node.get_logger().info("Mission formed!")

    #     # re-enable the buttons:
    #     time.sleep(2)
    #     self.button_handling.emit()

    # method for pressing the start sim button:
    def _on_start_sim_clicked(self):
        """
        Method for when the start simulation button has been hit. Locks all buttons on the GUI and instantiates another thread,
        which calls the ``_start_sim_process()`` method. 
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._start_sim_process, args = (), daemon = True).start()

    # method for start sim process:
    def _start_sim_process(self):
        """
        Method responsible for the starting the simultion. This method is ran within its own thread. Populates and publishes a message
        signalling that the simulation has started. Then, this method checks for any goals within the queue. If a goal is found, the 
        ``_publish_next_goal()`` method is called. If no goals are found, then nothing happens. After this check, the buttons are re-enabled.
        """
        # 1 - SEND THE SIGNAL THAT THE SIMULATION HAS STARTED:
        # populate the start sim message:
        msg      = String()
        msg.data = "start"

        # publish the message:
        self.node.start_pub.publish(msg)

        # 2 - IF THERE IS ANY GOAL IN THE QUEUE:
        if self.goal_queue:
            self.node.get_logger().info("Goal detected in queue, publishing!\n\n")
            self._publish_next_goal()
        else:
            self.node.get_logger().info("No goals provided for the current mission!")
            # re-enable buttons:
            time.sleep(0.5)
            self.button_handling.emit()

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
        # if it is not the first goal:
        if self.goal_number != 0:
            # display to user:
            # self.node.get_logger().info("Goal complete!")

            # delete the previous goal:
            self._kill_namespaced_node(namespace = "goal", node = "robot_state_pub")

            # despawn the goal body in the simulation:
            subprocess.run(["ign", "service", "-s", f"/world/{self.node.world_name}/remove",
                        "--reqtype", "ignition.msgs.Entity",
                        "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "2000",
                        "--req", f"name: 'goal' type: MODEL"],
                        stdout = subprocess.DEVNULL,
                        stderr = subprocess.DEVNULL)

        # if queue is empty:
        if not self.goal_queue:
            self.node.get_logger().info("Goal queue is empty, current simulation is complete!")

            # re-enable buttons:
            time.sleep(0.5)
            self.button_handling.emit()
            return

        # pop the first item from the queue:
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
        msg.required_capability     = goal_data[0]
        msg.pose.pose.position.x    = goal_data[1]
        msg.pose.pose.position.y    = goal_data[2]
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.w = 1.0

        # publish:
        self.node.goal_pub.publish(msg)
        print(f"Goal published at: ({goal_data[1]}, {goal_data[2]}) with type: {goal_data[0]}!")

# define main execution of node:
def main():
    # start the GUI:
    app = QApplication(sys.argv)

    # initialize rclpy:
    rclpy.init()

    # instantiate the node:
    node = GuiNode()

    # spin ROS2 in a background thread so it doesn't block the GUI:
    ros_thread = threading.Thread(target = rclpy.spin, args = (node, ), daemon = True)
    ros_thread.start()

    window   = MainWindow(node = node)
    node.gui = window
    window.show()

    # allow python to read signal every 500ms:
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda:None)

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