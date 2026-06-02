# import packages:
import py_trees
from rclpy.node import Node

# allocation behaviours:
from mrs_bt_handler.behaviours import (
    SimulationStarted,          # condition for checking if the simulation has started or not
    ActiveGoal,                 # condition for checking if there is an active goal or not
    SubmitBid,                  # action for computing and publishing a bid
    AllBidsReceived,            # condition for checking to see if all of the bids have arrived
    RemainIdle,                 # action for remaining idle when not selected
    CheckForWin,                # condition for checking to see if a given agent has won the auction
    NavigateToGoal,             # action for navigating to the goal location
    RecallAuction               # action for recalling the auction 
)

# need to define a method for creating trees:
def create_tree(node : Node, model_path: str = "") -> py_trees.trees.BehaviourTree:
    """
    Function for creating a behaviour tree.

    :param node: The ROS2 BT node to register the behaviours to.
    :type node: Node

    :param model_path: The path to the suitability model to be used.
    :type model_path: str
    
    """
    # instantiate the condition nodes:
    sim_started = SimulationStarted(node)
    active_goal    = ActiveGoal(node)
    wait_for_bids  = AllBidsReceived(node)
    check_for_win  = CheckForWin(node)

    # instantiate the action nodes:
    submit_bid      = SubmitBid(node, model_path)
    remain_idle      = RemainIdle(node)
    navigate_to_goal = NavigateToGoal(node)
    recall_auction   = RecallAuction(node)

    # main execution tree:
    execution_tree = py_trees.composites.Selector(
        name = "ExecutionTree",
        memory = False, 
        children = [
            navigate_to_goal,
            recall_auction
        ]
    )

    # add the tree that checks for agent winning:
    check_tree = py_trees.composites.Sequence(
        name = "CheckTree",
        memory = False,
        children = [
            check_for_win, 
            execution_tree
        ]
    )

    # add the main nav tree:
    nav_tree = py_trees.composites.Selector(
        name = "MainNav",
        memory = False,
        children = [
            check_tree,
            remain_idle
        ]
    )

    # define the root:
    root = py_trees.composites.Sequence(
        name = "Root",
        memory = False,
        children = [
            sim_started, 
            active_goal, 
            submit_bid, 
            wait_for_bids, 
            nav_tree
        ]
    )

    # return the completed tree:
    return py_trees.trees.BehaviourTree(root)
