# import packages:
import sys
import re
import os
import time
import threading
import signal
import subprocess
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory 

# gui-specific packages:
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel, QComboBox, QPushButton, QGroupBox
from PyQt5.QtCore import QTimer, Qt, pyqtSignal 
from qtwidgets import AnimatedToggle


# class for the node:
class GuiNode(Node):
    # constructor for the node:
    def __init__(self):
        # inherit from the parent node class:
        super().__init__("gui_node")

        # display to user when node has started:
        self.get_logger().info("GUI node started")

# class for the actual GUI:
class MainWindow(QWidget):
    # signal for updating GUI buttons from threads:
    reset_finished = pyqtSignal()

    # window constructor:
    def __init__(self):
        # inherit from parent:
        super().__init__()

        # set title of window:
        self.setWindowTitle("ROS2 MRS GUI")

        # set size of GUI:
        self.setFixedWidth(500)
        self.setFixedHeight(300)

        # set a style sheet:
        self.setStyleSheet("""
            QPushButton {
                padding: 6px 14px;
                border-radius: 4px;
                background-color: #00B7FF;
                color: white;
                font-weight: bold;
                min-width: 120px;
                max-width: 120px;
                min-height: 20px;
                max-height: 20px;
                margin: 0px;
            }         
            QPushButton:hover {
                background-color: #005fa3;
            }
            QComboBox {
                padding: 4px;
                min-width: 120px;
            }
            QLabel {
                font-weight: bold;
                font-size: 14px;
            }
                           
            QGroupBox {
                font-weight: bold;
                font-size:   14px;
            }
        """)

        # main layout manager:
        main_layout = QVBoxLayout()
        main_layout.setSpacing(15)

        # instantiate the child layouts:
        group1 = QGroupBox("Agent Settings")    # this is the group for the agent settings
        grid1 = QGridLayout()                   # this is a grid to hold the agent drop down and reset button
        group1.setLayout(grid1)                 # set the layout to be the grid

        group2 = QGroupBox("Goal Settings")     # this is the group for the goal settings
        grid2 = QGridLayout()                   # this is for the goal type drop down and randomize
        group2.setLayout(grid2)                 # set the layout to be the grid

        group3 = QGroupBox("Simulation Settings")   # this is the group for the simulation settings
        grid3 = QGridLayout()                       # this is a grid for resetting and running simulation buttons
        group3.setLayout(grid3)                     # set the layout to be the grid

        ##### grid 1 - agent related settings: #####
        # add the dropdown:
        self.agent_combo_box = QComboBox()      # instantiate the combo_box
        result = subprocess.run(["ros2", "node", "list"], capture_output = True, text = True)

        # pull number of agents:
        agents = sorted(list(set(re.findall(r'/(agent\d+)/', result.stdout))))

        # add these agents to the combo box:
        self.agent_combo_box.addItems(agents)
        
        # add combo box to the grid:
        grid1.addWidget(self.agent_combo_box, 0, 0, alignment = Qt.AlignCenter)

        # add the button for resetting the agent to the grid:
        self.agent_reset_button = QPushButton("Reset Agent")
        self.agent_reset_button.clicked.connect(self._on_agent_reset_clicked)
        grid1.addWidget(self.agent_reset_button, 0, 1, alignment = Qt.AlignCenter)

        ##### grid 2 - goal related settings: ##### 
        # add the dropdown:
        self.goal_combo_box = QComboBox()
        goal_types = ["typeA", "typeB"]
        self.goal_combo_box.addItems(goal_types)

        # add combo box to the grid:
        grid2.addWidget(self.goal_combo_box, 0, 0, alignment = Qt.AlignCenter)

        # button for randomizing the goal:
        self.randomize_goal_button = QPushButton("Randomize Goal")
        self.randomize_goal_button.clicked.connect(self._on_goal_randomize_clicked)
        grid2.addWidget(self.randomize_goal_button, 0, 1, alignment = Qt.AlignCenter)

        ##### grid 3 - randomization and resetting buttons: ##### 
        # button for resetting the sim:
        self.sim_reset_button = QPushButton("Reset Simulation")
        self.sim_reset_button.clicked.connect(self._on_sim_reset_clicked)
        grid3.addWidget(self.sim_reset_button, 0, 0, alignment = Qt.AlignCenter)

        # button for starting sim:
        self.start_sim_button = QPushButton("Start Simulation")
        self.start_sim_button.clicked.connect(self._on_start_sim_clicked)
        grid3.addWidget(self.start_sim_button, 0, 1, alignment = Qt.AlignCenter)

        # add grids/rows to main layout:
        main_layout.addWidget(group1)
        main_layout.addWidget(group2)
        main_layout.addWidget(group3)
    
        # apply the layout:
        self.setLayout(main_layout)

        # connect signals:
        self.reset_finished.connect(self._enable_buttons)

    # define method for resetting buttons:
    def _lock_buttons(self):
        # lock all buttons:
        self.agent_reset_button.setEnabled(False)
        self.randomize_goal_button.setEnabled(False)
        self.sim_reset_button.setEnabled(False)
        self.start_sim_button.setEnabled(False)

        # modify text of buttons:
        self.agent_reset_button.setText("Waiting...")
        self.randomize_goal_button.setText("Waiting...")
        self.sim_reset_button.setText("Waiting...")
        self.start_sim_button.setText("Waiting...")

    # define method for enabling buttons:
    def _enable_buttons(self):
        # unlock all buttons:
        self.agent_reset_button.setEnabled(True)
        self.randomize_goal_button.setEnabled(True)
        self.sim_reset_button.setEnabled(True)
        self.start_sim_button.setEnabled(True)

        # modify text of buttons:
        self.agent_reset_button.setText("Reset Agent")
        self.randomize_goal_button.setText("Randomize Goal")
        self.sim_reset_button.setText("Reset Simulation")
        self.start_sim_button.setText("Start Simulation")

    ##### AGENT RESET BUTTON METHODS: #####
    # define method for agent reset button:
    def _on_agent_reset_clicked(self):
        # lock the buttons:
        self._lock_buttons()

        # get value of agent in menu:
        agent_name = self.agent_combo_box.currentText()

        # use another thread to call reset function:
        threading.Thread(target = self._reset_agent_process, args = (agent_name, ), daemon = True).start()

    # define actual agent reset method:
    def _reset_agent_process(self, agent_name : str):
        # dummy position for testing:
        x = "-3.0"
        y = "-3.0"

        # move the position of the agent passed:
        subprocess.run(["ign", "service", "-s", "/world/world_1/set_pose",
                        "--reqtype", "ignition.msgs.Pose",
                        "--reptype", "ignition.msgs.Boolean", 
                        "--timeout", "2000",
                        "--req", f"name: '{agent_name}', position: {{x: {x}, y: {y}, z: {0.0}}}"])
        
        self._kill_namespaced_nodes(agent_name = agent_name)

        # call the launch file:
        self.odom_process = subprocess.Popen(["ros2", "launch", "mrs_robot_launcher", "odom_launch.py", f"agent_name:={agent_name}"])

        # re-enable the buttons:
        time.sleep(2)
        self.reset_finished.emit()

    # define method for killing nodes based on namespace:
    def _kill_namespaced_nodes(self, agent_name : str):
        # executables:
        nodes = ["ekf_node", "covariance_filter_node", "rf2o_laser_odom"]

        # for every executable:
        for executable in nodes:
            # find the executable that corresponds to that agent:
            result = subprocess.run(
                ["pgrep", "-f", f"{executable}.*__ns:=/{agent_name}"],
                capture_output = True,
                text = True
            )

            # strip down to the PID:
            pid = result.stdout.strip()

            # if the process exists, kill it
            if pid:
                subprocess.run(["kill", pid])

    ##### START SIM BUTTON METHODS: #####
    # define method for start sim button:
    def _on_start_sim_clicked(self):
        # lock the buttons:
        self._lock_buttons()

        # use another thread to call the start sim function:
        threading.Thread(target = self._start_sim_process, args = (), daemon = True).start()

    # define actual start simulation method:
    def _start_sim_process(self):
        # testing:

        # re-enable the buttons:
        time.sleep(2)
        self.reset_finished.emit()

    ##### RANDOMIZE GOAL BUTTON METHODS: #####
    # define method for randomize goal button:
    def _on_goal_randomize_clicked(self):
        # lock the buttons:
        self._lock_buttons()

        # use another thread to call the goal randomize function:
        threading.Thread(target = self._goal_randomize_process, args = (), daemon = True).start()

    # define actual goal randomize method:
    def _goal_randomize_process(self):
        # testing:
        print("randomizing goal!")

        # re-enable the buttons:
        time.sleep(2)
        self.reset_finished.emit() 
    
    ##### RESET SIMULATION BUTTON METHODS: #####
    # define method for reset simulation button:
    def _on_sim_reset_clicked(self):
        # lock the buttons:
        self._lock_buttons()

        # use another thread to call the goal randomize function:
        threading.Thread(target = self._sim_reset_process, args = (), daemon = True).start()

    # define actual simulation reset method:
    def _sim_reset_process(self):
        # testing:
        print("resetting simulation!")

        # re-enable the buttons:
        time.sleep(2)
        self.reset_finished.emit() 

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

    # start the GUI:
    app = QApplication(sys.argv)
    window = MainWindow()
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