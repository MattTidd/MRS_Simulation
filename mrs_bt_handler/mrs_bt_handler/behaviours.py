# import packages:
import numpy as np
import py_trees
import os
import subprocess
import signal
import time
from mrs_drl_interfaces.action import NavigateToGoal
from rclpy.action import ActionClient

# what behaviours do I need to define?
# - IsSimulationStarted
# - ActiveGoal
# - ComputeAndPublishBid
# - WaitForAllBids
# - RemainIdle
# - CheckForWin
# - NavigateToGoal
# - RecallAuction

# define the condition node for checking if the simulation has started yet:
class IsSimulationStarted(py_trees.behaviour.Behaviour):
    # constructor for the behaviour:
    def __init__(self, node):
        # inherit from parent class:
        super().__init__("IsSimulationStarted")

        # add node to class:
        self.node = node

    # define update method:
    def update(self):
        # if the node has its simulation started flag flipped to true, return success:
        if self.node.simulation_started:
            return py_trees.common.Status.SUCCESS
        
        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")
        
        # else return failure:
        return py_trees.common.Status.FAILURE
    
# define the condition node for checking if there is an active goal:
class ActiveGoal(py_trees.behaviour.Behaviour):
    # constructor for the behaviour:
    def __init__(self, node):
        # inherit from parent:
        super().__init__("ActiveGoal")

        # add node to class:
        self.node = node  

    # define update method:
    def update(self):
        # if the node has a PoseStamped message for the goal (i.e. not None type), return success:
        if self.node.goal is not None:
            return py_trees.common.Status.SUCCESS
        
        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")
        
        # else return failure:
        return py_trees.common.Status.FAILURE
    
# define an action for computing and publishing a bid:
class ComputeAndPublishBid(py_trees.behaviour.Behaviour):
    # constructor for the behaviour:
    def __init__(self, node, model_path : str):
        # inherit from parent class:
        super().__init__("ComputeAndPublishBid")

        # add to class:
        self.node               =    node
        self.model              =    None
        self.model_path         =    os.path.join(model_path, "ann_model.h5")
        self.scaler_path        =    os.path.join(model_path, "ann_scaler.pkl")
        self.bid_published      =    False
        self.last_goal          =    None

    # define method for setting up the model:
    def setup(self, **kwargs):
        # try to load up the model used for suitability calculations:
        try:
            # load packages that will be needed:
            from keras.models import load_model
            import tensorflow as tf
            from keras.losses import MeanSquaredError
            from pickle import load

            # load model based on the path provided:
            self.model = load_model(self.model_path, custom_objects = {'mse' : MeanSquaredError()})
            self.node.get_logger().info(f"Suitability model loaded from {self.model_path}")

            # load the scaler:
            self.scaler = load(open(self.scaler_path, "rb"))
            self.node.get_logger().info(f"Scaler loaded from {self.scaler_path}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to load suitability model: {e}")

    # define method for initializing (MUST USE BRITISH SPELLING):
    def initialise(self):
        # reset only if the goal has changed:
        if self.node.goal != self.last_goal:
            self.bid_published  =    False
            self.last_goal      =    self.node.goal
    
    # define method for updating the bid:
    def update(self):
        # if there is a published bid, just return success:
        if self.bid_published:
            return py_trees.common.Status.SUCCESS
        
        # if the node failed to load the model, return failure:
        if self.model is None:
            self.node.get_logger().error(f"Suitability model was not loaded")
            return py_trees.common.Status.FAILURE
        
        # if the latest_odom or goal has not yet arrived, return running:
        if self.node.latest_odom is None or self.node.goal is None:
            self.node.get_logger().info("Waiting for arrival of goal or odometry...")
            return py_trees.common.Status.RUNNING
        
        # global goal position:
        gx = self.node.goal.pose.position.x
        gy = self.node.goal.pose.position.y

        # global agent position:
        x = self.node.agent_initial_x + self.node.latest_odom.pose.pose.position.x
        y = self.node.agent_initial_y + self.node.latest_odom.pose.pose.position.y

        # distance to the goal:
        d_goal = np.sqrt((gx - x) ** 2 + (gy - y) ** 2)

        # form an input vector:
        input = np.array([[self.node.load_history, d_goal, self.node.total_distance]])

        # scale the input vector:
        scaled_input = self.scaler.transform(input)

        # run inference on the input vector:
        suitability = float(self.model.predict(scaled_input, verbose = 0))
        self.node.get_logger().info(f"{self.node.agent_name} suitability: {suitability:.4f} | TDT: {round(self.node.total_distance, 3)} | LH: {self.node.load_history} | DTT: {round(d_goal, 3)}")

        # publish the bid:
        self.node.publish_bid(suitability)
        self.bid_published = True

        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")

        # return success after publishing a bid:
        return py_trees.common.Status.SUCCESS
    
# define the condition node to check if all the bids are in:
class WaitForAllBids(py_trees.behaviour.Behaviour):
    # constructor for the behaviour:
    def __init__(self, node):
        # inherit from parent:
        super().__init__("WaitForAllBids")

        # add the node to the class:
        self.node = node

    # define update method:
    def update(self):
        # define the number of expected bids:
        n_bids = len(self.node.all_bids)

        # if the number of bids matches the number of agents within the system:
        if n_bids == self.node.num_agents:
            return py_trees.common.Status.SUCCESS
        
        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")
        
        # otherwise log that you are waiting and return running:
        self.node.get_logger().info(f"Waiting for bids: {n_bids}/{self.node.num_agents}")
        return py_trees.common.Status.RUNNING
    
# define an action for simply remaining idle:
class RemainIdle(py_trees.behaviour.Behaviour):
    # constructor for the behaviour:
    def __init__(self, node):
        # inherit from parent:
        super().__init__("RemainIdle")

        # add node to the class:
        self.node = node
    
    # define update method:
    def update(self):
        # log status:
        self.node.get_logger().info(f"{self.node.agent_name} lost bid, idling...")

        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")

        # return success:
        return py_trees.common.Status.SUCCESS
    
# define condition node to check to see if agent wins:
class CheckForWin(py_trees.behaviour.Behaviour):
    # constructor for the behaviour:
    def __init__(self, node):
        # inherit from parent:
        super().__init__("CheckForWin")

        # add node to the class:
        self.node = node
    
    # define update method:
    def update(self):
        # if the agent wins the bid:
        if self.node.is_winner():
            # self.node.get_logger().info(f"{self.node.agent_name} won the bid.")
            return py_trees.common.Status.SUCCESS
        
        # otherwise:
        # self.node.get_logger().info(f"{self.node.agent_name} did not win the bid.")
        return py_trees.common.Status.FAILURE

# define an action for navigating to the goal:
class NavigateToGoal(py_trees.behaviour.Behaviour):
    # constructor for the behaviour:
    def __init__(self, node, timeout : float = 60.0):
        # inherit from parent class:
        super().__init__("NavigateToGoal")

        # add node to class:
        self.node = node

        # add timer for timeout tracking:
        self.timeout        =    timeout
        self._start_time    =    None

    # define initialise method for class:
    def initialise(self):
        # if there is a new goal:
        if self.node.new_goal:
            # reset flag:
            self.node.new_goal = False

            # start a timer:
            self._start_time = time.time()

            # spin policy node:
            self.node.spin_up_policy()

            # print to user:
            self.node.get_logger().info(f"{self.node.agent_name} navigating to goal")

    # define update method for class:
    def update(self):
        # if there is either no goal or no odometry coming in:
        if self.node.latest_odom is None or self.node.goal is None:
            self.node.get_logger().info("no odom or latest goal")
            return py_trees.common.Status.RUNNING
        
        # check for a timeout:
        if self._start_time is not None and (time.time() - self._start_time) > self.timeout:
            self.node.get_logger().warn(f"{self.node.agent_name} navigation timed out.")
            return py_trees.common.Status.FAILURE
        
        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")
        
        # global goal position:
        gx = self.node.goal.pose.position.x
        gy = self.node.goal.pose.position.y

        # global agent position:
        x = self.node.agent_initial_x + self.node.latest_odom.pose.pose.position.x
        y = self.node.agent_initial_y + self.node.latest_odom.pose.pose.position.y

        # distance to the goal:
        d_goal = np.sqrt((gx - x) ** 2 + (gy - y) ** 2)

        # check d_goal for completion:
        if d_goal <= self.node.goal_tolerance:
            # increment load history:
            self.node.load_history += 1.0

            # clear the active goal:
            self.node.goal = None

            # return success:
            return py_trees.common.Status.SUCCESS
        
        # otherwise keep running:
        return py_trees.common.Status.RUNNING
    
    # define termination method:
    def terminate(self, new_status):
        # kill the policy:
        self.node.kill_policy()

        # clear the goal for the other agents on success:
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.broadcast_goal_clear()
        # rebroadcast the goal on failure:
        elif new_status == py_trees.common.Status.FAILURE:
            self.node.rebroadcast_goal()

# define class for recalling the auction:
class RecallAuction(py_trees.behaviour.Behaviour):
    # constructor for behaviour:
    def __init__(self, node):
        # inherit from parent:
        super().__init__("RecallAuction")

        # add node to class:
        self.node = node

    # define update method:
    def update(self):
        # log to user:
        self.node.get_logger().warn(f"{self.node.agent_name} failed navigation, recalling auction")

        # clear bids, and goal:
        self.node.all_bids  =    {}
        self.node.goal      =    None

        # return success on clearing:
        return py_trees.common.Status.SUCCESS

