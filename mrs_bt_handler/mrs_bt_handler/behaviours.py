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
        self.node.get_logger().info(f"{self.node.agent_name} suitability: {suitability:.4f}")

        # publish the bid:
        self.node.publish_bid(suitability)
        self.bid_published = True

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

        # return success:
        return py_trees.common.Status.SUCCESS
    
# define condition node to check to see if agent wins:
class CheckForWin(py_trees.behaviour.Behaviour):
    # constructor for node:
    def __init__(self, node):
        # inherit from parent:
        super().__init__("CheckForWin")

        # add node to the class:
        self.node = node
    
    # define update method:
    def update(self):
        # if the agent wins the bid:
        if self.node.is_winner():
            self.node.get_logger().info(f"{self.node.agent_name} won the bid.")
            return py_trees.common.Status.SUCCESS
        
        # otherwise:
        self.node.get_logger().info(f"{self.node.agent_name} did not win the bid.")
        return py_trees.common.Status.FAILURE

# define an action for navigating to the goal:
class NavigateToGoal(py_trees.behaviour.Behaviour):
    # define a constructor for the node:
    def __init__(self, node):
        # inherit from parent:
        super().__init__("NavigateToGoal")

        # add to the class:
        self.node           = node
        self._goal_handle   = None
        self._result        = None
        self._goal_sent     = False
        self._done          = False
        self._succeeded     = False

    # define a method for initializing (MUST USE BRITISH SPELLING):
    def initialise(self):
        # reset each of the states on each new activation:
        self._goal_handle   = None
        self._result        = None
        self._goal_sent     = False
        self._done          = False
        self._succeeded     = False

        # spin the policy node:
        self.node.spin_up_policy()

        # wait for policy node to launch:
        time.sleep(2)

        # wait for the action server:
        self.node.get_logger().info("Waiting for action server...")
        self.node.nav_client.wait_for_server()

        # form the goal message:
        goal_msg                            = NavigateToGoal.Goal()
        goal_msg.target_pose                = self.node.goal
        goal_msg.goal_tolerance             = 0.125

        # send the goal message:
        send_goal_future = self.node.nav_client.send_goal_async(
            goal_msg,
            feedback_callback = self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

        # set flag to true, log that agent is navigating:
        self._goal_sent = True
        self.node.get_logger().info(f"{self.node.agent_name} navigating to goal")

    # define update method:
    def update(self):
        # if not done yet:
        if not self._done:
            return py_trees.common.Status.RUNNING
        
        # if successful:
        if self._succeeded:
            # increment load history of the agent:
            self.node.load_history += 1.0

            # set goal to None:
            self.node.goal = None

            # report success:
            return py_trees.common.Status.SUCCESS
        # otherwise:
        else:
            return py_trees.common.Status.FAILURE
        
    # define termination method:
    def terminate(self, new_status):
        # kill the policy node when done, regardless of what happens:
        self.node.kill_policy()

        # if failed, trigger a rebroadcast of the goal:
        if new_status == py_trees.common.Status.FAILURE:
            self.node.rebroadcast_goal()

    # define a callback for the goal response:
    def _goal_response_callback(self, future):
        self._goal_handle = future.result()

        # if goal is rejected:
        if not self._goal_handle.accepted:
            self.node.get_logger().warn("Goal rejected")
            self._done          =    True
            self._succeeded     =    False
            return
        
        # get result, add callback:
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    # define result callback:
    def _result_callback(self, future):
        self._result     = future.result().result
        self._succeeded  = self._result.success
        self._done       = True
        self.node.get_logger().info(f"Navigation result: {self._result.message}")

    # define feedback callback:
    def _feedback_callback(self, feedback):
        pass

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

