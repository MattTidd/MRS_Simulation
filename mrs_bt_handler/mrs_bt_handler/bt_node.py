# import packages:
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Bool

import subprocess
import signal
import os
import numpy as np
import threading
import time

from mrs_drl_interfaces.action import NavigateToGoal
from mrs_drl_interfaces.msg import Bid, Goal
from mrs_bt_handler.trees.agent_tree import create_tree

# define a class for the node:
class BTNode(Node):
    """
    Primary class for the ``BTNode``, which is responsible for hosting and ticking the developed behaviour tree, and routing information
    into variables that are accessed by the tree.
    - Inherits from ``rclpy.node.Node``
    """
    # constructor for the node:
    def __init__(self):
        """
        Constructor for the node. Declares and adds parameters to the class, instantiates subscribers and publishers, and starts
        running a dedicated thread for the tree.

        :param agent_name: Name of the agent.
        :type agent_name: str

        :param agent_type: Type of the agent. Representative of its capabilities.
        :type agent_type: str

        :param agent_initial_x: Initial x position of the agent within the environment, measured globally.
        :type agent_initial_x: float

        :param agent_initial_y: Initial y position of the agent within the environment, measured globally.
        :type agent_initial_y: float

        :param agent_initial_yaw: Initial yaw orientation of the agent within the environment, measured globally.
        :type agent_initial_yaw: float

        :param model_name: Name of the DRL model to be used. 
        :type model_name: str

        :param num_agents: Number of agents within the system.
        :type num_agents: int

        :param model_path: Path to the directory containing the suitability inference model.
        :type model_path: str

        :param goal_tolerance: Minimum threshold for successful navigation, in meters.
        :type goal_tolerance: float

        :param goal_timeout: Allotted time for goal completion, in seconds.
        :type goal_timeout: float

        """
        # inherit from parent class:
        super().__init__("bt_node") # set the name of the node

        ##### declare parameters: #####
        self.declare_parameter("agent_name", "agent1")
        self.declare_parameter("agent_type", "typeA")
        self.declare_parameter("agent_initial_x", 0.0)
        self.declare_parameter("agent_initial_y", 0.0)
        self.declare_parameter("agent_initial_yaw", 0.0)
        self.declare_parameter("model_name", "SAC_001")
        self.declare_parameter("num_agents", 2)
        self.declare_parameter("model_path", "")
        self.declare_parameter("goal_tolerance", 0.125)
        self.declare_parameter("goal_timeout", 120.0)

        ##### add parameters to the class: #####
        self.agent_name        = self.get_parameter("agent_name").value
        self.agent_type        = self.get_parameter("agent_type").value
        self.agent_initial_x   = self.get_parameter("agent_initial_x").value
        self.agent_initial_y   = self.get_parameter("agent_initial_y").value
        self.agent_initial_yaw = self.get_parameter("agent_initial_yaw").value
        self.model_name        = self.get_parameter("model_name").value
        self.num_agents        = self.get_parameter("num_agents").value
        self.model_path        = self.get_parameter("model_path").value
        self.goal_tolerance    = self.get_parameter("goal_tolerance").value
        self.goal_timeout      = self.get_parameter("goal_timeout").value

        ##### storage for the important states that are used by the node/tree: #####
        self.goal:                  PoseStamped     |   None    =     None      # current pose of goal
        self.latest_odom:           Odometry        |   None    =     None      # latest odometry
        self.distance_to_goal:      float                       =     0.0       # distance to the current goal
        self.total_distance:        float                       =     0.0       # total distance travelled
        self.load_history:          float                       =     0.0       # load history (number of tasks completed thus far)
        self.suitability:           float                       =     0.0       # suitability for the task at hand
        self.all_bids:              dict                        =     {}        # dictionary of form {agent_name : suitability_score}
        self.simulation_started:    bool                        =     False     # flag for whether the simulation has started or not
        self.new_goal:              bool                        =     False     # flag for whether a new goal has arrived or not
        self.nav_failed:            bool                        =     False     # flag for navigational success
        self.policy_process                                     =     None      # handle for the policy subprocess
        self.goal_process                                       =     None      # handle for the goal subprocess
        self.rebroadcast_count                                  =     0         # int counter for goal rebroadcasting
        self.reset_requested                                    =     False     # flag for whether a reset has been requested
        self.reset_complete                                     =     False     # flag for whether the reset of an agent is complete

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

        # subscriber for reset complete signal:
        self.reset_complete_sub = self.create_subscription(Bool, "/reset_complete", self._reset_complete_callback, 10)

        ##### create publishers: #####
        # cmd_vel publisher:
        self.cmd_vel_pub = self.create_publisher(TwistStamped, f"/{self.agent_name}/cmd_vel", 10)

        # publisher for bid of an agent:
        self.bid_pub  = self.create_publisher(Bid, f"/{self.agent_name}/bid", 10)

        # publisher for the goal:
        self.goal_pub = self.create_publisher(Goal, "/goal", 10)

        # publisher for resetting:
        self.reset_pub = self.create_publisher(String, "/reset_agent", 10)

        # ##### action client for navigation: #####
        # self.nav_client = ActionClient(self, NavigateToGoal, "navigate_to_goal")

        ##### build and start the behaviour tree: #####
        self.tree = create_tree(node = self, model_path = self.model_path)
        self.tree.setup(timeout = 2)

        # run tree in a separate thread:
        self._tree_thread = threading.Thread(target = self._run_tree, args = (10, ), daemon = True).start()

        # instantiate a thread lock:
        self._odom_lock = threading.Lock()

    # method for ticking BT in thread:
    def _run_tree(self, frequency : int):
        """
        Method for ticking the tree. 

        :param frequency: Frequency at which the tree is ticked.
        :type frequency: int
        """
        while rclpy.ok():
            # tick the tree:
            self.tree.tick()
            time.sleep(1 / frequency)

    # define the goal callback method:
    def _goal_callback(self, msg : Goal):
        """
        Callback method called by the goal subscriber. Adds a received goal pose to the class, as well as its required capability.
        Resets the dictionary of bids upon receiving a new goal. Also handles interpretation of empty goal messages, which 
        signify completed goals.

        :param msg: Goal message that is subscribed to. 
        :type msg: Goal
        """
        # handle goal clearance signal:
        if msg.required_capability == "":
            # self.get_logger().info("Goal cleared by winner")
            self.goal     = None
            self.all_bids = {}
            self.new_goal = False
            return

        # let the user know that the goal has been received:
        self.get_logger().info(f"Goal received at: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")

        # add the goal pose to the class:
        self.goal = msg.pose

        # add the goal required capabilities to the class:
        self.required_capability = msg.required_capability

        # add the goal id to the class:
        self.goal_id = msg.id

        # reset rebroadcast counter on fresh goals:
        if "_rebroadcast" not in msg.id:
            self.rebroadcast_count = 0

        # reset the list of bids upon receiving a new goal:
        self.all_bids = {}

        # flag for new goal:
        self.new_goal = True

    # define the odometry callback method:
    def _odom_callback(self, msg : Odometry):
        """
        Odometry callback called by the odometry subscriber. Tracks the total distance that the agent has travelled thus far. 

        :param msg: Odometry message that is subscribed to. 
        :type msg: Odometry
        
        """
        with self._odom_lock:
            # track the total distance that the agent has travelled:
            if self.latest_odom is not None:
                # get previous values:
                x_prev = self.latest_odom.pose.pose.position.x
                y_prev = self.latest_odom.pose.pose.position.y

                # get current values from msg:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y

                # compute total distance:
                self.total_distance += np.sqrt((x - x_prev)**2 + (y - y_prev)**2)
            
            # advance latest odom via msg:
            self.latest_odom = msg
        
    # define the bid callback method:
    def _bid_callback(self, msg : Bid, agent_name : str):
        """
        Bid callback called by the bid subscriber. Adds the bid of an agent to the list of total bids.

        :param msg: Bid message that is subscribed to. 
        :type msg: Bid

        :param agent_name: Name of the agent whose bid was received.
        :type agent_name: str
        """
        # add the bid of the agent to the list of total bids:
        self.all_bids[agent_name] = (msg.suitability, msg.capability)

    # define the start callback method:
    def _start_callback(self, msg : String):
        """
        Start callback called by the start subscriber. Reads whether or not the simulation has started or not.
        Also is used to determine when to reset the objective parameters of an agent. On reset, the load history, 
        total distance, and latest odometry are reset. 

        :param msg: Start message that is subscribed to. 
        :type msg: String
        """
        # if the topic reads start:
        if msg.data == "start":
            # set the flag for simulation starting to true:
            self.simulation_started = True
        # otherwise if the topic reads reset:
        elif msg.data == "reset":
            with self._odom_lock:
                # set the flag for simulation starting to false:
                self.simulation_started = False

                # reset the objective parameters of the agent:
                self.load_history   = 0.0
                self.total_distance = 0.0

                # drop the stale odom reference:
                self.latest_odom = None

                # drop the stale goal reference:
                self.goal = None

                # clear the dict of bids:
                self.all_bids = {}

                # reset new goal flag:
                self.new_goal = False

    # define the reset complete callback method:
    def _reset_complete_callback(self, msg : Bool):
        """
        Reset callback used by the reset complete subscriber. Flips the ``reset_complete`` flag based on the 
        boolean value contained in the message.

        :param msg: Reset complete message that is subscribed to.
        :type msg: Bool
        """
        # flip the flag based on the value of the topic:
        self.reset_complete = msg.data

    # define method for publishing a bid:
    def publish_bid(self, suitability : float):
        """
        Method for publishing a bid. Takes a suitability, and populates a bid using the name and capability of the agent, along 
        with the calculated suitability. 

        :param suitability: Calculated suitability of the agent for the task at hand. 
        :type suitability: float
        """
        # create empty Float32 message:
        msg = Bid()

        # populate the bid:
        msg.agent_name  = self.agent_name
        msg.suitability = suitability
        msg.capability  = self.agent_type

        # publish the bid:
        self.bid_pub.publish(msg)

    # define a method for determining agent that is the winner:
    def is_winner(self) -> bool:
        """
        Method for determining the winner of an auction. Checks first to see if all of the bids are in, then checks the eligibility 
        of the bids. If the agent is not eligible, return false. Determines the name of the winner agent.

        :returns: ``False`` if not eligible or all bids are not in yet, and the ``agent_name`` of the winning agent.  
        
        """
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
    
    # define method for broadcasting that the goal is complete:
    def broadcast_goal_clear(self):
        """
        Method for broadcasting if a goal has been cleared. Creates a dummy message and publishes it, as an empty required capability 
        is interpreted as goal completion. 

        """
        # create dummy goal message:
        msg = Goal()
        self.goal_pub.publish(msg)
        self.get_logger().info(f"{self.agent_name.capitalize()} broadcasting goal completion\n\n")

    # define method for rebroadcasting the goal on failure:
    def rebroadcast_goal(self):
        """
        Republishes the current goal onto /goal to retrigger bidding on all agents. 
        Called by the ``RecallAuction`` behaviour when navigation fails.
        """
        # check for None type goal:
        if self.goal is None:
            self.get_logger().warn("rebroadcast_goal called but goal is None — cannot rebroadcast.")
            return
        
        # log to user:
        self.get_logger().warn(f"{self.agent_name}: rebroadcasting goal to retrigger auction...")
        
        # form and publish the goal:
        msg = Goal()
        base_id = self.goal_id.split("_rebroadcast")[0]
        msg.id = f"{base_id}_rebroadcast_{self.rebroadcast_count}"
        msg.pose = self.goal
        msg.required_capability = self.required_capability
        self.goal_pub.publish(msg)

        # advance rebroadcast counter:
        self.rebroadcast_count += 1

    # define method for resetting the agent:
    def reset_agent(self):
        """
        Method used to call agent resetting. Flips the ``reset_complete`` flag to false, and then 
        publishes the name of the agent to be reset, which is interpreted externally by the GUI.
        """
        # log to user:
        self.get_logger().info(f"{self.agent_name}: hit obstacle, moving to spawn...")

        # flip the reset complete flag:
        self.reset_complete = False

        # publish the reset signal:
        reset_msg = String()
        reset_msg.data = self.agent_name
        self.reset_pub.publish(reset_msg)

    # define method for monitoring goal process:
    def _monitor_goal_process(self):
        """
        Waits for the goal_process subprocess to exit, then sets nav_failed based on its 
        return code. Non-zero means that navigation has failed.
        """
        # check for a goal_process:
        if self.goal_process is None:
            return
        
        # simply wait until there is a returncode:
        self.goal_process.wait()

        # if there is a non-zero returncode:
        if self.goal_process.returncode not in (0, -15):    # -15 is termination via SIGTERM (success)
            # log to user:
            self.get_logger().warn(
            f"{self.agent_name}: goal_client exited with code: "
            f"{self.goal_process.returncode}, flagging nav_failed."
            )

            # flip flag:
            self.nav_failed = True

    # define method for spinning the navigation policy and goal nodes up:
    def spin_up_policy(self):
        """
        Method for spinning up the policy and goal processes. Checks if either are inactive, and if so, spins them up via subprocesses.
        
        """
        # if there is no active policy process:
        if self.policy_process is None or self.policy_process.poll() is not None:
            # spin up node, similar to GUI:
            self.policy_process = subprocess.Popen([
                "ros2", "run", "mrs_drl_policy", "policy_node", "--ros-args", 
                "-p", f"model_name:={self.model_name}",
                "-p", f"agent_name:={self.agent_name}",
                "-p", f"agent_initial_yaw:={self.agent_initial_yaw}",
                "-p", f"goal_timeout:={self.goal_timeout}"], start_new_session = True)
            
        # if there is no active goal process:
        if self.goal_process is None or self.goal_process.poll() is not None:
            # need to extract the position of the goal within the frame of the agent:
            dx = self.goal.pose.position.x - self.agent_initial_x
            dy = self.goal.pose.position.y - self.agent_initial_y

            # spin up the goal client, similar to the GUI:
            self.goal_process = subprocess.Popen(["ros2", "run", "mrs_drl_policy", "goal_client", str(dx), str(dy), f"{self.goal_tolerance}"], start_new_session = True)

            # reset failure flag, start monitor thread:
            self.nav_failed = False
            threading.Thread(target = self._monitor_goal_process, args = (), daemon = True).start()

    # define method for killing the policy node:
    def kill_policy(self):
        """
        Method for killing the policy. Performs cleanup on the goal process if it has not self-terminated, and then sends a 
        ``SIGTERM`` to the policy process. 
        """
        # cleanup on goal process if left hanging:
        if self.goal_process is not None:
            if self.goal_process.poll() is None:
                # still running, therefore kill it:
                os.killpg(os.getpgid(self.goal_process.pid), signal.SIGTERM)
                self.goal_process.wait()
                self.get_logger().info("Goal process killed.")
            else:
                self.get_logger().info("Goal process already self-terminated.")

        # clear handle:
        self.goal_process = None

        # if there is a policy process active:
        if self.policy_process and self.policy_process.poll() is None:
            # buffer for letting the current control loop iteration to finish:
            time.sleep(0.1)

            # send a SIGTERM to the node:
            os.killpg(os.getpgid(self.policy_process.pid), signal.SIGTERM)

            # wait for it to die:
            self.policy_process.wait()

            # set the policy process to None:
            self.policy_process = None

            # print to user:
            self.get_logger().info("Policy process killed.")
        else:
            pass
            # self.get_logger().warn("kill_policy method called but no active process found.")

        # publish a zero velocity:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.05)

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
