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
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QLineEdit, QComboBox, QPushButton
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

        # main layout manager:
        main_layout = QVBoxLayout()

        # instantiate the child layouts:
        grid1 = QGridLayout()   # this is for the agent drop down and reset
        row1 = QHBoxLayout()    # this is for the run simulation button
        grid2 = QGridLayout()   # this is for randomization and resetting

        ##### grid 1 - agent related settings: #####
        # add the label for the dropdown:
        grid1.addWidget(QLabel("Agent"), 0, 0, alignment = Qt.AlignCenter)
        
        # add the dropdown:
        self.agent_combo_box = QComboBox()      # instantiate the combo_box
        result = subprocess.run(["ros2", "node", "list"], capture_output = True, text = True)

        # pull number of agents:
        agents = list(set(re.findall(r'/(agent\d+)/', result.stdout)))

        # add these agents to the combo box:
        for agent in sorted(agents):
            self.agent_combo_box.addItem(agent)
        
        # add combo box to the grid:
        grid1.addWidget(self.agent_combo_box, 1, 0, alignment = Qt.AlignCenter)

        # add the button for resetting the agent to the grid:
        self.agent_reset_button = QPushButton("Reset Agent")
        self.agent_reset_button.clicked.connect(self._on_agent_reset_clicked)
        grid1.addWidget(self.agent_reset_button, 1, 1, alignment = Qt.AlignCenter)

        ##### row 1 - button for starting simulation: ##### 
        self.start_sim_button = QPushButton("Start Simulation")
        self.start_sim_button.clicked.connect(self._on_start_sim_clicked)
        row1.addWidget(self.start_sim_button)

        ##### grid 2 - randomization and resetting buttons: ##### 
        # button for randomizing the goal:
        self.randomize_goal_button = QPushButton("Randomize Goal")
        self.randomize_goal_button.clicked.connect(self._on_goal_randomize_clicked)
        grid2.addWidget(self.randomize_goal_button, 0, 0, alignment = Qt.AlignCenter)

        # button for resetting the sim:
        self.sim_reset_button = QPushButton("Reset Simulation")
        self.sim_reset_button.clicked.connect(self._on_sim_reset_clicked)
        grid2.addWidget(self.sim_reset_button, 0, 1, alignment = Qt.AlignCenter)

        # add grids/rows to main layout:
        main_layout.addLayout(grid1)
        main_layout.addLayout(row1)
        main_layout.addLayout(grid2)
    
        # apply the layout:
        self.setLayout(main_layout)

    # define method for agent reset button:
    def _on_agent_reset_clicked(self):
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