# import packages:
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String

import py_trees
import py_trees_ros
import subprocess
import signal
import os
import numpy as np

from mrs_drl_interfaces.action import NavigateToGoal
from mrs_drl_interfaces.msg import Bid, Goal
from mrs_bt_handler.trees.agent_tree import create_tree

# define a class for the node:
class BTNode(Node):
    # constructor for the node:
    def __init__(self):
        # inherit from parent class:
        super().__init__("bt_node")     # set the name of the node

        ##### declare parameters: #####
        self.declare_parameter("agent_name", "agent1")
        self.declare_parameter("agent_type", "typeA")
        self.declare_parameter("agent_initial_x", 0.0)
        self.declare_parameter("agent_initial_y", 0.0)
        self.declare_parameter("model_name", "SAC_001")
        self.declare_parameter("num_agents", 2)
        self.declare_parameter("model_path", "")
        self.declare_parameter("goal_tolerance", 0.125)

        ##### add parameters to the class: #####
        self.agent_name         =   self.get_parameter("agent_name").value
        self.agent_type         =   self.get_parameter("agent_type").value
        self.agent_initial_x    =   self.get_parameter("agent_initial_x").value
        self.agent_initial_y    =   self.get_parameter("agent_initial_y").value
        self.model_name         =   self.get_parameter("model_name").value
        self.num_agents         =   self.get_parameter("num_agents").value
        self.model_path         =   self.get_parameter("model_path").value
        self.goal_tolerance     =   self.get_parameter("goal_tolerance").value

        ##### storage for the important states that are used by the node/tree: #####
        self.goal:                  PoseStamped     |   None    =     None      # current pose of goal
        self.latest_odom:           Odometry        |   None    =     None      # latest odometry
        self.total_distance:        float                       =     0.0       # total distance travelled
        self.distance_history:      float                       =     0.0       # load history (number of tasks completed thus far)
        self.all_bids:              dict                        =     {}        # dictionary of form {agent_name : suitability_score}
        self.simulation_started:    bool                        =     False     # flag for whether the simulation has started or not
        self.policy_process                                     =     None      # handle for the policy subprocess
        self.goal_process                                       =     None      # handle for the goal subprocess

        ##### create subscribers: #####
        # subscriber for the goal:
        self.goal_sub = self.create_subscription(
            Goal, "/goal", self._goal_callback, 10
        )

        # subscriber for agent odometry:
        self.odom_sub = self.create_subscription(
            Odometry, f"/{self.agent_name}/odom", self._odom_callback, 10
        )

        # subscriber for the simulation start signal from the GUI:
        self.start_sub = self.create_subscription(
            String, "/simulation_start", self._start_callback, 10
        )

        # instantiate dict for agent bids:
        self.bid_subs = {}

        # for each agent in the MRS:
        for i in range(1, self.num_agents + 1):
            # set the key of the dict:
            name = f"agent{i}"

            # create a subscriber for each of the agents in the system:
            self.bid_subs[name] = self.create_subscription(
                Bid, f"/{name}/bid",
                lambda msg, n = name: self._bid_callback(msg, n), 10
            )

        ##### create publishers: #####
        # publisher for bid of an agent:
        self.bid_pub = self.create_publisher(Bid, f"/{self.agent_name}/bid", 10)

        ##### action client for navigation: #####
        self.nav_client = ActionClient(self, NavigateToGoal, "navigate_to_goal")

        ##### build and start the behaviour tree: #####
        self.tree = create_tree(node = self, model_path = self.model_path)
        self.tree.setup(timeout = 2)
        self.tree_timer = self.create_timer(0.1, self._tick_tree)   # tick the tree at 10Hz
        self.get_logger().info(f"BT node started for {self.agent_name}")

    # define goal callback method:
    def _goal_callback(self, msg : Goal):
        # let the user know that the goal has been received:
        self.get_logger().info(f"Goal received at: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")

        # add the goal pose to the class:
        self.goal = msg.pose

        # add the goal required capabilities to the class:
        self.required_capability = msg.required_capability

        # reset the list of bids upon receiving a new goal:
        self.all_bids = {}

    # define the odometry callback method:
    def _odom_callback(self, msg : Odometry):
        # track the total distance that the agent has travelled:
        if self.latest_odom is not None:
            # get previous values:
            x_prev = self.latest_odom.pose.pose.position.x
            y_prev = self.latest_odom.pose.pose.position.y

            # get current values from msg:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # compute total distance:
            self.distance_history += np.sqrt((x - x_prev)**2 + (y - y_prev)**2)
        
        # advance latest odom via msg:
        self.latest_odom = msg
        
    # define the bid callback method:
    def _bid_callback(self, msg : Bid, agent_name : str):
        # add the bid of the agent to the list of total bids:
        self.all_bids[agent_name] = (msg.suitability, msg.capability)

    # define the start callback method:
    def _start_callback(self, msg : String):
        # if the topic reads start:
        if msg.data == "start":
            # set the flag for simulation starting to true:
            self.simulation_started = True
        # otherwise if the topic reads stop:
        elif msg.data == "stop":
            # set the flag for simulation starting to false:
            self.simulation_started = False
    
    # define method for ticking the tree:
    def _tick_tree(self):
        # send a tick to the tree:
        self.tree.tick()

    # define method for publishing a bid:
    def publish_bid(self, suitability : float):
        # create empty Float32 message:
        msg = Bid()

        # populate the bid:
        msg.agent_name      =    self.agent_name
        msg.suitability     =    suitability
        msg.capability      =    self.agent_type

        # publish the bid:
        self.bid_pub.publish(msg)

    # define a method for determining agent that is the winner:
    def is_winner(self) -> bool:
        # if not all bids are in yet:
        if len(self.all_bids) < self.num_agents:
            return False
        
        # check eligibility of bids:
        eligible = {k: v for k, v in self.all_bids.items() if v[1] == self.required_capability}
        
        # return to user based on eligibility:
        if not eligible:
            return False

        winner = max(eligible, key = lambda k: eligible[k][0])

        # return name of winning agent:
        return winner == self.agent_name
    
    # define method for rebroadcasting the goal:
    def rebroadcast_goal(self):
        # if navigation fails, call this method:
        self.get_logger().warn(f"{self.agent_name} failed, rebroadcasting goal...")

        # clear the list of bids, as there will be a new goal:
        self.all_bids = {}

        # clear goal:
        self.goal = None

    # define method for spinning policy node up:
    def spin_up_policy(self):
        # if there is no active policy process:
        if self.policy_process is None or self.policy_process.poll() is not None:
            # spin up node, similar to GUI:
            self.policy_process = subprocess.Popen([
                "ros2", "run", "mrs_drl_policy", "policy_node", "--ros-args", 
                "-p", f"model_name:={self.model_name}",
                "-p", f"agent_name:={self.agent_name}"], start_new_session = True)
            
        # if there is no active goal process:
        if self.goal_process is None or self.goal_process.poll() is not None:
            # need to extract the position of the goal within the frame of the agent:
            dx = self.goal.pose.position.x - self.agent_initial_x
            dy = self.goal.pose.position.y - self.agent_initial_y

            # spin up the goal client, similar to the GUI:
            self.goal_process = subprocess.Popen(["ros2", "run", "mrs_drl_policy", "goal_client", str(dx), str(dy), f"{self.goal_tolerance}"], start_new_session = True)

    # define method for killing the policy node:
    def kill_policy(self):
        # if there is a policy process active:
        if self.policy_process and self.policy_process.poll() is None:
            # send a SIGTERM to the node:
            os.killpg(os.getpgid(self.policy_process.pid), signal.SIGTERM)

            # wait for it to die:
            self.policy_process.wait()

            # set the policy process to None:
            self.policy_process = None

# define the main function:
def main():
    # initialize rclpy:
    rclpy.init()

    # instantiate the node:
    node = BTNode()

    # call a multi-threaded executor:
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # start spinning it:
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.kill_policy()
        node.destroy_node()
        rclpy.shutdown()

# main:
if __name__ == "__main__":
    main()
