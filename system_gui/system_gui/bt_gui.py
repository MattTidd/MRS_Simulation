# import packages:
import sys
import os
import random
import time
import threading
import signal
import subprocess
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory 
from mrs_drl_interfaces.msg import Goal
from std_msgs.msg import String

# gui-specific packages:
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel, QComboBox, QPushButton, QGroupBox, QLineEdit
from PyQt5.QtCore import QTimer, Qt, pyqtSignal 
from qtwidgets import AnimatedToggle

# class for the main node:
class GuiNode(Node):
    # constructor for node:
    def __init__(self):
        # inherit from parent class:
        super().__init__("bt_gui_node")

        # display to user when node has started:
        self.get_logger().info("GUI node started")

        # declare parameters:
        self.declare_parameter("positions", [0.0])
        self.declare_parameter("agent_names", [""])

        # add parameter to the class:
        flat                    =    self.get_parameter("positions").value
        names                   =    self.get_parameter("agent_names").value
        positions               =    [[flat[i], flat[i+1]] for i in range(0, len(flat), 2)]
        self.agent_positions    =    dict(zip(names, positions))

        # establish publishers:
        self.goal_pub   = self.create_publisher(Goal, "/goal", 10)
        self.start_pub  = self.create_publisher(String, "/simulation_start", 10)

# class for the actual GUI:
class MainWindow(QWidget):
    # signal for buttons:
    button_handling = pyqtSignal()

    # constructor for GUI:
    def __init__(self, node : Node):
        # inherit from parent class:
        super().__init__()

        # add the node to the GUI:
        self.node = node

        # get the position dict:
        self.agent_positions = self.node.agent_positions

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

        # add a button for publishing the goal:
        self.publish_goal_button = QPushButton("Publish Goal")
        self.publish_goal_button.clicked.connect(self._on_publish_goal_clicked)
        grid0.addWidget(self.publish_goal_button, 0, 3, alignment = Qt.AlignCenter)

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
        # lock all buttons:
        self.publish_goal_button.setEnabled(False)

        # modify text of buttons:
        self.publish_goal_button.setText("Waiting...")

    # method for enabling the buttons:
    def _enable_buttons(self):
        # unlock buttons:
        self.publish_goal_button.setEnabled(True)

        # modify the text of the buttons:
        self.publish_goal_button.setText("Publish Goal")

    # method for killing nodes:
    def _kill_namespaced_node(self, namespace : str, node = None):
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

    # method for pressing the goal button:
    def _on_publish_goal_clicked(self):
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._goal_publish_process, args = (), daemon = True).start()

    # method for goal publish process:
    def _goal_publish_process(self):
        # print to user:
        print("\npublishing goal...")

        # pull values needed to populate message:
        x = self.x_input.text()
        y = self.y_input.text()
        goal_type = self.goal_type_combo_box.currentText()
        
        # need to ensure that these values are their correct typings:
        try:
            x = float(x)
            y = float(y)
        # catch the exception:
        except Exception as e:
            print(f"Please provide a valid goal pose: \n{e}")

            # perform the re-enable before returning:
            time.sleep(2)
            self.button_handling.emit()
            return
        
        # need to then kill the current goal publishing nodes:
        self._kill_namespaced_node(namespace = "goal", node = "robot_state_pub")

        # despawn the goal body in the simulation:
        subprocess.run(["ign", "service", "-s", "/world/world_1/remove",
                    "--reqtype", "ignition.msgs.Entity",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "2000",
                    "--req", f"name: 'goal' type: MODEL"])
        
        # relaunch the goal:
        subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "goal_launch.py", 
                          "use_sim_time:=true",
                          "goal_name:=goal", 
                          f"goal_type:={goal_type}",
                          f"goal_initial_x_pos:={x}",
                          f"goal_initial_y_pos:={y}"])
            
        # need to populate the goal message:
        msg                             =   Goal()
        msg.pose.header.stamp           =   self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x        =   x
        msg.pose.pose.position.y        =   y
        msg.pose.pose.position.z        =   0.0
        msg.pose.pose.orientation.w     =   1.0
        msg.required_capability         =   goal_type

        # publish:
        self.node.goal_pub.publish(msg)
        print(f"Goal published at: ({x}, {y})!")

        # re-enable buttons:
        time.sleep(2)
        self.button_handling.emit()

    # method for pressing the reset sim button:
    def _on_reset_sim_clicked(self):
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        for agent_name in self.agent_positions:
            threading.Thread(target = self._reset_sim_process, args = (agent_name, ), daemon = True).start()

    # method for reset sim process:
    def _reset_sim_process(self, agent_name : str):
        # need to extract the positions of each agent:
        pos = self.agent_positions[agent_name]
        x, y = pos[0], pos[1]

        # move the position of the agent name passed to the process:
        subprocess.run(["ign", "service", "-s", "/world/world_1/set_pose",
                        "--reqtype", "ignition.msgs.Pose",
                        "--reptype", "ignition.msgs.Boolean",
                        "--timeout", "2000",
                        "--req", f"name: '{agent_name}', position: {{x: {x}, y: {y}, z: {0.0}}}"])
        
        # kill the nodes related to the odometry of that agent:
        self._kill_namespaced_node(namespace = agent_name)

        # call the launch file for the odometry nodes:
        self.odom_process = subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "odom_launch.py", f"agent_name:={agent_name}"])

        # re-enable the buttons:
        time.sleep(2)
        self.button_handling.emit()

    # method for pressing the start sim button:
    def _on_start_sim_clicked(self):
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._start_sim_process, args = (), daemon = True).start()

    # method for start sim process:
    def _start_sim_process(self):
        # print to user:
        print("starting simulation...")

        # populate the start sim message:
        msg = String()
        msg.data = "start"

        # publish the message:
        self.node.start_pub.publish(msg)

        # re-enable buttons:
        time.sleep(2)
        self.button_handling.emit()

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

    window = MainWindow(node = node)
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