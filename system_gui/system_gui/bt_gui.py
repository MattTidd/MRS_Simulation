# import packages:
import re
import sys
import time
import threading
import signal
import subprocess
import rclpy
from rclpy.node import Node
from mrs_drl_interfaces.msg import Goal
from std_msgs.msg import String

# gui-specific packages:
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QComboBox, QPushButton, QGroupBox, QLineEdit
from PyQt5.QtCore import QTimer, Qt, pyqtSignal 

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

        :param positions: List containing the starting positions of each agent, measured globally. 
        :type positions: list

        :param agent_names: List containing the names of each of the agents. 
        :type agent_names: list
        
        """
        # inherit from parent class:
        super().__init__("bt_gui_node")

        # add gui to the node:
        self.gui = None

        # display to user when node has started:
        self.get_logger().info("GUI node started")

        # declare parameters:
        self.declare_parameter("positions", [0.0])
        self.declare_parameter("agent_names", [""])
        self.declare_parameter("world_name", "world_1")

        # add parameters to the class:
        flat                 = self.get_parameter("positions").value
        names                = self.get_parameter("agent_names").value
        world_path           = self.get_parameter("world_name").value
        
        self.world_name      = re.split(r'[/.]', world_path)[-2]
        positions            = [[flat[i], flat[i+1]] for i in range(0, len(flat), 2)]
        self.agent_positions = dict(zip(names, positions))

        # establish subscribers:
        self.goal_sub  = self.create_subscription(Goal, "/goal", self._goal_callback, 10)

        # establish publishers:
        self.goal_pub   = self.create_publisher(Goal, "/goal", 10)
        self.start_pub  = self.create_publisher(String, "/simulation_start", 10)

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

        # get the position dict:
        self.agent_positions = self.node.agent_positions

        # flag for despawning:
        self.goal_number = 0

        # set an empty dict for goal queuing:
        self.goal_queue = {}

        # set the title of window:
        self.setWindowTitle("ROS2 MRS GUI")

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
                min-width: 100px;
                min-height: 20px;
            }
        """)

        # main layout manager:
        main_layout = QVBoxLayout()
        main_layout.setSpacing(15)

        # instantiate the child layouts:
        group0 = QGroupBox("Goal Settings:")
        grid0  = QGridLayout()
        group0.setLayout(grid0)

        group1 = QGroupBox("Simulation Settings:")
        grid1  = QGridLayout()
        group1.setLayout(grid1)

        ##### grid 0 - goal related settings: #####
        # need to have a combo box for selecting the goal type:
        self.goal_type_combo_box = QComboBox()
        self.goal_type_combo_box.setEditable(True)
        self.goal_type_combo_box.lineEdit().setAlignment(Qt.AlignCenter)
        self.goal_type_combo_box.lineEdit().setReadOnly(True)

        # TODO: dynamically pull the type of the agents and populate combo-box:
        self.goal_type_combo_box.addItem("typeA")
        self.goal_type_combo_box.addItem("typeB")

        # add combo box to the grid:
        grid0.addWidget(self.goal_type_combo_box, 0, 0, alignment = Qt.AlignCenter)

        # add an entry field for the goal x-position:
        self.x_input = QLineEdit()
        self.x_input.setPlaceholderText("x position")
        self.x_input.setAlignment(Qt.AlignCenter)
        grid0.addWidget(self.x_input, 0, 1, alignment = Qt.AlignCenter)

        # add an entry field for the goal y-position:
        self.y_input = QLineEdit()
        self.y_input.setPlaceholderText("y position")
        self.y_input.setAlignment(Qt.AlignCenter)
        grid0.addWidget(self.y_input, 0, 2, alignment = Qt.AlignCenter)

        # add a button for queueing goals:
        self.queue_goal_button = QPushButton("Queue Goal")
        self.queue_goal_button.clicked.connect(self._on_queue_goal_clicked)
        grid0.addWidget(self.queue_goal_button, 0, 3, alignment = Qt.AlignCenter)

        ##### grid 1 - simulation related settings: #####
        # add a button for resetting the simulation:
        self.reset_sim_button = QPushButton("Reset Simulation")
        self.reset_sim_button.clicked.connect(self._on_reset_sim_clicked)
        grid1.addWidget(self.reset_sim_button, 0, 0, alignment = Qt.AlignCenter)

        # add a button for starting the simulation:
        self.start_sim_button = QPushButton("Start Simulation")
        self.start_sim_button.clicked.connect(self._on_start_sim_clicked)
        grid1.addWidget(self.start_sim_button, 0, 1, alignment = Qt.AlignCenter)

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
        Method for locking the buttons contained within the GUI, thus preventing users from hitting them while a 
        process runs. Locks and modifies the text of buttons. 
        """
        # lock all buttons:
        self.queue_goal_button.setEnabled(False)

        # modify text of buttons:
        self.queue_goal_button.setText("Waiting...")

    # method for enabling the buttons:
    def _enable_buttons(self):
        """
        Method for enabling the buttons contained within the GUI, thus allowing users to interact with them. Unlocks the buttons
        and sets their text to their native values prior to being locked.
        """
        # unlock buttons:
        self.queue_goal_button.setEnabled(True)

        # modify the text of the buttons:
        self.queue_goal_button.setText("Publish Goal")

    # method for killing nodes:
    def _kill_namespaced_node(self, namespace : str, node = None):
        """
        Method for killing namespaced nodes. Does this by running a ``"kill"`` subprocess on the ``PID`` of the
        node process. Either accepts single nodes, or if no single node was passed, can be used to kill all nodes
        relating to the odometry of the agent.

        :param namespace: Namespace used by the node to be killed. 
        :type namespace: str

        :param node: Individual node to be killed.
        :type node: str
        """
        # list the process names of the odometry executables:
        nodes = ["ekf_node", "covariance_filter_node", "rf2o_laser_odom"]

        # if the user passes a single node:
        if node:
            # pull the PID of the node that they are looking for:
            result = subprocess.run(
                    ["pgrep", "-f", f"{node}.*__ns:=/{namespace}"],
                    capture_output = True,
                    text = True
                )
            
            # back out the PID:
            pid = result.stdout.strip()

            # if this process exists, kill it:
            if pid:
                subprocess.run(["kill", pid])
        else:
            # for every executable:
            for executable in nodes:
                # find the executable that corresponds to that agent:
                result = subprocess.run(
                    ["pgrep", "-f", f"{executable}.*__ns:=/{namespace}"],
                    capture_output = True,
                    text = True
                )

                # back out the PID:
                pid = result.stdout.strip()

                # if the process exists, kill it
                if pid:
                    subprocess.run(["kill", pid])

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
            time.sleep(1)
            self.button_handling.emit()
            return

        # add current goal into the goal queue dictionary:
        self.goal_queue[f"goal_{len(self.goal_queue) + 1}"] = [goal_type, x, y]

        # re-enable buttons:
        time.sleep(1)
        self.button_handling.emit()

    # method for pressing the reset sim button:
    def _on_reset_sim_clicked(self):
        """
        Method for when the reset sim button has been hit. Locks all buttons on the GUI and instantiates another thread, 
        which calls the ``_reset_sim_process()`` method. 
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        for agent_name in self.agent_positions:
            threading.Thread(target = self._reset_sim_process, args = (agent_name, ), daemon = True).start()

    # method for reset sim process:
    def _reset_sim_process(self, agent_name : str):
        """
        Method responsible for the actual resetting of the simulation. This method is ran within its own thread. Extracts the 
        positions of each agent, and moves the agent to its initial pose using a subprocess Gazebo service call. The 
        odometric nodes of the agent are then killed using the ``_kill_namespaced_node()`` method, and are relaunched. The goal is 
        also killed using the ``_kill_namespaced_node()`` method, and removed from the simulation using a subprocess Gazebo service call.
        Finally, the goal queue is cleared and the buttons are re-enabled. 

        :param agent_name: Name of the agent to be reset.
        :type agent_name: str
        """
        # need to extract the positions of each agent:
        pos  = self.agent_positions[agent_name]
        x, y = pos[0], pos[1]

        # move the position of the agent name passed to the process:
        subprocess.run(["ign", "service", "-s", f"/world/{self.node.world_name}/set_pose",
                        "--reqtype", "ignition.msgs.Pose",
                        "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "2000",
                        "--req", f"name: '{agent_name}', position: {{x: {x}, y: {y}, z: {0.0}}}"])
        
        # kill the nodes related to the odometry of that agent:
        self._kill_namespaced_node(namespace = agent_name)

        # call the launch file for the odometry nodes:
        self.odom_process = subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "odom_launch.py", f"agent_name:={agent_name}"])

        # despawn the goal:
        self._kill_namespaced_node(namespace = "goal", node = "robot_state_pub")

        subprocess.run(["ign", "service", "-s", f"/world/{self.node.world_name}/remove",
                    "--reqtype", "ignition.msgs.Entity",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "2000",
                    "--req", f"name: 'goal' type: MODEL"])

        # clear the goal queue:
        self.goal_queue.clear()

        # formulate a reset message:
        msg      = String()
        msg.data = "reset"

        # publish the message:
        self.node.start_pub.publish(msg)

        # re-enable the buttons:
        time.sleep(2)
        self.button_handling.emit()

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
            self.node.get_logger().info("Goal detected in queue, publishing!")
            self._publish_next_goal()
        else:
            self.node.get_logger().info("No goals provided for the current mission!")

        # re-enable buttons:
        time.sleep(2)
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
            self.node.get_logger().info("Goal complete!")

            # delete the previous goal:
            self._kill_namespaced_node(namespace = "goal", node = "robot_state_pub")

            # despawn the goal body in the simulation:
            subprocess.run(["ign", "service", "-s", f"/world/{self.node.world_name}/remove",
                        "--reqtype", "ignition.msgs.Entity",
                        "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "2000",
                        "--req", f"name: 'goal' type: MODEL"])

        # if queue is empty:
        if not self.goal_queue:
            self.node.get_logger().info("Goal queue is empty, current simulation is complete!")
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
                          f"goal_initial_y_pos:={goal_data[2]}"])
        
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