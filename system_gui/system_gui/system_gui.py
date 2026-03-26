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
        # for agent in sorted(agents):
        self.agent_combo_box.addItems(agents)
        
        # add combo box to the grid:
        grid1.addWidget(self.agent_combo_box, 1, 0, alignment = Qt.AlignCenter)

        # add the button for resetting the agent to the grid:
        self.agent_reset_button = QPushButton("Reset Agent")
        self.agent_reset_button.clicked.connect(self._on_agent_reset_clicked)
        grid1.addWidget(self.agent_reset_button, 1, 1, alignment = Qt.AlignCenter)

        ##### grid 2 - goal related settings: ##### 
        # add the dropdown:
        self.goal_combo_box = QComboBox()
        goal_types = ["typeA", "typeB"]
        self.goal_combo_box.addItems(goal_types)

        # add combo box to the grid:
        grid2.addWidget(self.goal_combo_box, 1, 0, alignment = Qt.AlignCenter)

        # button for randomizing the goal:
        self.randomize_goal_button = QPushButton("Randomize Goal")
        self.randomize_goal_button.clicked.connect(self._on_goal_randomize_clicked)
        grid2.addWidget(self.randomize_goal_button, 1, 1, alignment = Qt.AlignCenter)

        ##### grid 3 - randomization and resetting buttons: ##### 
        # button for resetting the sim:
        self.sim_reset_button = QPushButton("Reset Simulation")
        self.sim_reset_button.clicked.connect(self._on_sim_reset_clicked)
        grid3.addWidget(self.sim_reset_button, 1, 0, alignment = Qt.AlignCenter)

        # button for starting sim:
        self.start_sim_button = QPushButton("Start Simulation")
        self.start_sim_button.clicked.connect(self._on_start_sim_clicked)
        grid3.addWidget(self.start_sim_button, 1, 1, alignment = Qt.AlignCenter)

        # add grids/rows to main layout:
        main_layout.addWidget(group1)
        main_layout.addWidget(group2)
        main_layout.addWidget(group3)
    
        # apply the layout:
        self.setLayout(main_layout)

    # define method for agent reset button:
    def _on_agent_reset_clicked(self):
        # lock all buttons:
        self.agent_reset_button.setEnabled(False)
        self.start_sim_button.setEnabled(False)
        # self.randomize

        # get value of agent in menu:
        agent_name = self.agent_combo_box.currentText()

    
        print(f"resetting {agent_name}!")

    # define method for start sim button:
    def _on_start_sim_clicked(self):
        print("starting simulation!")

    # define method for randomize goal button:
    def _on_goal_randomize_clicked(self):
        print("randomizing goal!")

    # define method for reset simulation button:
    def _on_sim_reset_clicked(self):
        print("resetting simulation!")

# define main execution of node:
def main():
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