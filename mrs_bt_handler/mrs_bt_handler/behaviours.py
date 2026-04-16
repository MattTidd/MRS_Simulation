# import packages:
import numpy as np
import py_trees
import os
import time
from rclpy.node import Node

# define the condition node for checking if the simulation has started yet:
class IsSimulationStarted(py_trees.behaviour.Behaviour):
    """
    ``IsSimulationStarted`` condition node for use in the behaviour tree. This node returns success upon
    verifying that the simulation has started, routes ticks to the next node upon emitting this.

    - Inherits from ``py_trees.behaviour.Behaviour``.
    """
    # constructor for the behaviour:
    def __init__(self, node : Node):
        """
        Constructor for the behaviour.

        :param node: The ROS2 BT node that the tree is attached to.
        :type node: Node
        
        """
        # inherit from parent class:
        super().__init__("IsSimulationStarted")

        # add node to class:
        self.node = node

    # define update method:
    def update(self):
        """
        Update method of the behaviour. Checks the boolean value of the ``simulation_started`` flag of the passed parent node.

        - If the ``simulation_started`` flag is ``True``, return ``py_trees.common.Status.SUCCESS``.

        - If the ``simulation_started`` flag is ``False``, return ``py_trees.common.Status.FAILURE``.

        :returns: Either ``py_trees.common.Status.SUCCESS`` or ``py_trees.common.Status.FAILURE`` depending on the status of ``simulation_started``.
        
        """
        # if the node has its simulation started flag flipped to true, return success:
        if self.node.simulation_started:
            return py_trees.common.Status.SUCCESS
        
        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")
        
        # else return failure:
        return py_trees.common.Status.FAILURE
    
# define the condition node for checking if there is an active goal:
class ActiveGoal(py_trees.behaviour.Behaviour):
    """
    ``ActiveGoal`` condition node for use in the behaviour tree. This node returns success upon receiving a ``PoseStamped`` message
    for the goal. If the goal message is ``None`` type, then it returns failure.

    - Inherits from ``py_trees.behaviour.Behaviour``.
    """
    # constructor for the behaviour:
    def __init__(self, node : Node):
        """
        Constructor for the behaviour.

        :param node: The ROS2 BT node that the tree is attached to.
        :type node: Node
        
        """
        # inherit from parent:
        super().__init__("ActiveGoal")

        # add node to class:
        self.node = node  

    # define update method:
    def update(self):
        """
        Update method of the behaviour. Monitors the ``goal`` of the parent node. 
        
        - If the node has a ``PoseStamped`` message present for the goal, return ``py_trees.common.Status.SUCCESS``.

        - If the node has a ``None`` type message present for the goal, return ``py_trees.common.Status.FAILURE``.

        :returns: Either ``py_trees.common.Status.SUCCESS`` or ``py_trees.common.Status.FAILURE`` depending on the type of ``goal``.
        
        """
        # if the node has a PoseStamped message for the goal (i.e. not None type), return success:
        if self.node.goal is not None:
            return py_trees.common.Status.SUCCESS
        
        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")
        
        # else return failure:
        return py_trees.common.Status.FAILURE
    
# define an action for computing and publishing a bid:
class ComputeAndPublishBid(py_trees.behaviour.Behaviour):
    """
    ``ComputeAndPublishBid`` action node for use in the behaviour tree. This node loads the suitability model passed to the tree, and uses
    it to calculate a bid for the active goal, submitting this bid onto the ``/bid`` topic of the agent.

    - Inherits from ``py_trees.behaviour.Behaviour``.
    """
    # constructor for the behaviour:
    def __init__(self, node : Node, model_path : str):
        """
        Constructor for the behaviour.

        :param node: The ROS2 BT node that the tree is attached to.
        :type node: Node

        :param model_path: The path to the suitability model to be used.
        :type model_path: str

        :param model: The suitability model to be used.
        :type model: None

        :param scaler_path: The path to the scaler used by the model.
        :type scaler_path: str

        :param bid_published: A boolean flag to track whether a bid has been published or not.
        :type bid_published: bool

        :param last_goal: A variable to track whether the current goal differs from the last goal.
        :type last_goal: None
        
        """
        # inherit from parent class:
        super().__init__("ComputeAndPublishBid")

        # add to class:
        self.node           = node
        self.model          = None
        self.model_path     = os.path.join(model_path, "ann_model.h5")
        self.scaler_path    = os.path.join(model_path, "ann_scaler.pkl")
        self.bid_published  = False
        self.last_goal      = None

    # define method for setting up the model:
    def setup(self, **kwargs):
        """
        Setup method of the behaviour, which is used to perform a delayed, one-time initialization of properties used
        by the behaviour.

        - Specifically, this method loads the suitability model as well as the scaler.
        """
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
        # if failing to load:
        except Exception as e:
            self.node.get_logger().error(f"Failed to load suitability model: {e}")

    # define method for initializing (MUST USE BRITISH SPELLING):
    def initialise(self):
        """
        Initialise method of the behaviour, which is used to perform initialization before commencing the operation of the behaviour. 

        - It is called on the first tick of the behaviour, and anytime thereafter when the status is not ``py_trees.common.Status.RUNNING``.

        - Specifically, this method resets the ``bid_published`` flag and updates the ``last_goal`` value, only when the goal has changed.
        """
        # reset only if the goal has changed:
        if self.node.goal != self.last_goal:
            self.bid_published  = False
            self.last_goal      = self.node.goal
    
    # define method for updating the bid:
    def update(self):
        """
        Update method of the behaviour. Computes a suitability for the task at hand, and publishes it onto the ``/bid`` message of the agent.
        - A bid is of message type ``Bid``, which consists of a ``suitability`` and a ``capability``.
        - Uses the learned suitability model to assess the objective parameters of the agent, such as its ``distance_to_task``, ``load_history``, and ``distance_history``
        - Output is a scalar ``suitability`` value, indicative of how suitable the agent is for the task at hand.
        
        :returns: ``py_trees.common.Status.FAILURE`` if the suitability model was not loaded, ``py_trees.common.Status.RUNNING`` if still waiting on agent odom or a goal, and
        ``py_trees.common.Status.SUCCESS`` upon publishing a bid.
        """
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
class AllBidsReceived(py_trees.behaviour.Behaviour):
    """
    ``AllBidsReceived`` condition node for use in the behaviour tree. This node checks for the bids of each agent to be submitted, gating the progression of the tree
    by ensuring that each agent has computed a bid for the task at hand.

    - Inherits from ``py_trees.behaviour.Behaviour``.
    """
    # constructor for the behaviour:
    def __init__(self, node):
        """
        Constructor for the behaviour.

        :param node: The ROS2 BT node that the tree is attached to.
        :type node: Node
        
        """
        # inherit from parent:
        super().__init__("AllBidsReceived")

        # add the node to the class:
        self.node = node

    # define update method:
    def update(self):
        """
        Update method of the behaviour. Pulls the number of submitted bids from the parent node, and checks to see if this matches the number of expected bids,
        which is equivalent to the number of agents within the system.
        - If these two numbers match, then the behaviour returns ``py_trees.common.Status.SUCCESS``.
        - Otherwise, return ``py_trees.common.Status.FAILURE``.

        :returns: ``py_trees.common.Status.FAILURE`` if all bids have not been received, ``py_trees.common.Status.SUCCESS`` otherwise.
        
        """
        # get number of submitted bids:
        n_bids = len(self.node.all_bids)

        # if the number of bids matches the number of agents within the system:
        if n_bids == self.node.num_agents:
            return py_trees.common.Status.SUCCESS
        
        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")
        
        # otherwise log that you are waiting and return failure:
        self.node.get_logger().info(f"Waiting for bids: {n_bids}/{self.node.num_agents}")
        return py_trees.common.Status.FAILURE
    
# define an action for simply remaining idle:
class RemainIdle(py_trees.behaviour.Behaviour):
    """
    ``RemainIdle`` action node for use in the behaviour tree. This node suspends the operations of an agent if they are not 
    successful in winning the auction.
    - Inherits from ``py_trees.behaviour.Behaviour``.
    """
    # constructor for the behaviour:
    def __init__(self, node):
        """
        Constructor for the behaviour.

        :param node: The ROS2 BT node that the tree is attached to.
        :type node: Node

        """
        # inherit from parent:
        super().__init__("RemainIdle")

        # add node to the class:
        self.node = node
    
    # define update method:
    def update(self):
        """
        Update method of the behaviour. Causes the agent to hang in an idle state.

        :returns: ``py_trees.common.Status.SUCCESS`` while agent idles.
        
        """
        # log status:
        self.node.get_logger().info(f"{self.node.agent_name} lost bid, idling...")

        # DEBUG:
        # self.node.get_logger().info(f"currently running {self.__class__.__name__}")

        # return success:
        return py_trees.common.Status.SUCCESS
    
# define condition node to check to see if agent wins:
class CheckForWin(py_trees.behaviour.Behaviour):
    """
    ``CheckForWin`` condition for use in the behaviour tree. This node checks whether or not the agent won the active auction, 
    and returns a status accordingly.
    - Inherits from ``py_trees_behaviour.Behaviour``. 
    """
    # constructor for the behaviour:
    def __init__(self, node):
        """
        Constructor for the behaviour.

        :param node: The ROS2 BT node that the tree is attached to.
        :type node: Node

        """
        # inherit from parent:
        super().__init__("CheckForWin")

        # add node to the class:
        self.node = node
    
    # define update method:
    def update(self):
        """
        Update method of the behaviour. Calls the ``is_winner()`` method of the parent node. 
        - If the agent is a winner, return ``py_trees.common.Status.SUCCESS``.
        - Otherwise, return ``py_trees.common.Status.FAILURE``.

        :returns: Either ``py_trees.common.Status.FAILURE`` or ``py_trees.common.Status.SUCCESS`` depending on whether the agent won or not.
        """
        # if the agent wins the bid:
        if self.node.is_winner():
            # self.node.get_logger().info(f"{self.node.agent_name} won the bid.")
            return py_trees.common.Status.SUCCESS
        
        # otherwise:
        # self.node.get_logger().info(f"{self.node.agent_name} did not win the bid.")
        return py_trees.common.Status.FAILURE

# define an action for navigating to the goal:
class NavigateToGoal(py_trees.behaviour.Behaviour):
    """
    ``NavigateToGoal`` action for use in the behaviour tree. This node is responsible for utilizing the developed DRL policy 
    to navigate to the goal location. 
    
    - Inherits from ``py_trees.behaviour.Behaviour``
    """
    # constructor for the behaviour:
    def __init__(self, node, timeout : float = 30.0):
        """
        Constructor for the behaviour. 

        :param node: The ROS2 BT node that the tree is attached to.
        :type node: Node

        :param timeout: Elapsed time before navigation is considered unsuccessful.
        :type timeout: float

        :param _start_time: Empty starting time for tracking timeout.
        :type _start_time: None
        
        """
        # inherit from parent class:
        super().__init__("NavigateToGoal")

        # add node to class:
        self.node = node

        # add timer for timeout tracking:
        self.timeout     = timeout
        self._start_time = None

    # define initialise method for class:
    def initialise(self):
        """
        Initialise method of the behaviour, which is used to perform initialization before commencing the operation of the behaviour. 

        - It is called on the first tick of the behaviour, and anytime thereafter when the status is not ``py_trees.common.Status.RUNNING``.

        - Specifically, this method resets the ``new_goal`` flag of the parent node, starts a timer on ``_start_time``, and calls the 
        ``spin_up_policy()`` method of the parent node.
        """
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
        """
        Update method of the behaviour. Checks to see if the agent has arrived at the goal yet.
        - If the agent has arrived at the goal, return ``py_trees.common.Status.SUCCESS``.
        - If the agent times out on navigation, return ``py_trees.common.Status.FAILURE``.
        - If there is no goal or odometry coming in, or the agent is not yet at the goal, return ``py_trees.common.Status.RUNNING``.

        :returns: ``py_trees.common.Status.FAILURE``, ``py_trees.common.Status.RUNNING``, or ``py_trees.common.Status.SUCCESS``, depending
        on the state of the agent.
        """
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
        """
        Terminate method of the behaviour. 
        - This is called wheneber the behaviour switches into a non-running state, such as ``py_trees.common.Status.FAILURE`` or
        ``py_trees.common.Status.SUCCESS``.
        - Specifically, this calls the ``kill_policy()`` method of the parent node, and if the status has transitioned into
        ``py_trees.common.Status.SUCCESS``, broadcasts that the goal has been cleared using the ``broadcast_goal_clear()`` method 
        of the parent node.

        :param new_status: Terminal status of the behaviour. 
        :type new_status: ``py_trees.common.Status``.
        """
        # kill the policy:
        self.node.kill_policy()

        # clear the goal for the other agents on success:
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.broadcast_goal_clear()

# define class for recalling the auction:
class RecallAuction(py_trees.behaviour.Behaviour):
    """
    ``RecallAuction`` action for use in the behaviour tree. This node recalls the auction procedure if the 
    ``NavigateToGoal`` action fails.
    - Inherits from ``py_trees.behaviour.Behaviour``.
    """
    # constructor for behaviour:
    def __init__(self, node):
        """
        Constructor for the behaviour.

        :param node: The ROS2 BT node that the tree is attached to.
        :type node: Node

        """
        # inherit from parent:
        super().__init__("RecallAuction")

        # add node to class:
        self.node = node

    # define update method:
    def update(self):
        """
        Update method of the behaviour. Clears the ``all_bids`` dict and the current ``goal`` from the parent node. 
        - Returns ``py_trees.common.Status.SUCCESS`` upon clearing these from the parent class.

        :returns: ``py_trees.common.Status.SUCCESS`` once clearing bid and goal related parameters from the parent node.
        """
        # log to user:
        self.node.get_logger().warn(f"{self.node.agent_name} failed navigation, recalling auction")

        # clear bids, and goal:
        self.node.all_bids = {}
        self.node.goal     = None

        # return success on clearing:
        return py_trees.common.Status.SUCCESS

