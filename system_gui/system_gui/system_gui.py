# import packages:
import sys
import os
import time
import threading
import signal
import subprocess
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory # type: ignore

# gui-specific packages:
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QLineEdit, QComboBox, QPushButton # type: ignore
from PyQt5.QtCore import QTimer, Qt, pyqtSignal # type: ignore
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