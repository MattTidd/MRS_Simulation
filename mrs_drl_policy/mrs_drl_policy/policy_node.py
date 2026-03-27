import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor 
from mrs_drl_interfaces.action import NavigateToGoal
import numpy as np
import torch, torch.nn as nn
from stable_baselines3 import TD3, SAC
from ament_index_python.packages import get_package_share_directory
import os
import pickle
import time
import copy
import signal

# define the policy node class:
class DRLPolicyNode(Node):
    # define the constructor of the node:
    def __init__(self):
        # inherit from parent class:
        super().__init__("drl_policy_server")   # set node name

        # declare parameters:
        self.declare_parameter("agent_name", "agent1")
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('obstacle_tolerance', 0.20)
        self.declare_parameter('model_name', 'SAC_001')
        self.declare_parameter('max_lin_vel', 0.6)
        self.declare_parameter('max_angular_vel', 0.9)
        self.declare_parameter('goal_timeout', 30.0)

        # add parameters to class:
        self.agent_name                 =   self.get_parameter("agent_name").value
        self.default_goal_tolerance     =   self.get_parameter('goal_tolerance').value
        self.default_obstacle_tolerance =   self.get_parameter('obstacle_tolerance').value
        self.model_name                 =   self.get_parameter('model_name').value
        self.model_type                 =   self.model_name.split('_')[0]
        self.max_lin_vel                =   self.get_parameter('max_lin_vel').value
        self.max_angular_vel            =   self.get_parameter('max_angular_vel').value
        self.goal_timeout               =   self.get_parameter('goal_timeout').value

        # get the paths:
        pkg_dir             =    get_package_share_directory("mrs_drl_policy")
        model_dir           =    os.path.join(pkg_dir, "policy", self.model_name)
        model_path          =    os.path.join(model_dir, "model")
        norm_stat_path      =    os.path.join(model_dir, "norm_stats.pkl")

        # enable the use of cuda if available:
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # load the model, if available:
        if self.model_type == "TD3":
            model = TD3.load(model_path, device = self.device)
        elif self.model_type == "SAC":
            model = SAC.load(model_path, device = self.device)
        
        # get the policy from the model, set to evaluation (inference):
        self.policy = model.policy
        self.policy.eval()

        # load the observation normalization stats:
        with open(norm_stat_path, "rb") as f:
            self.vec_norm = pickle.load(f)
        self.vec_norm.training = False

        # set size of observation space:
        self.n_ray_groups       =   18
        self._obs_space_size    =   27
        self._obs_buffer        =   np.zeros(self._obs_space_size, dtype = np.float32)

        # initialize variables needed for DRL reward calculation:
        self.action_last            =   np.zeros(2)
        self.action                 =   np.zeros(2)
        self.d_goal_last            =   0.0
        self.prev_abs_diff          =   0.0
        self.min_dist_last          =   0.0
        self.d_safe                 =   0.5
        self.lidar_idx_threshold    =   4

        self.rew_head_approach_scaled   =    0      ;   self.rew_head_approach_scale = 200.0
        self.rew_dist_approach_scaled   =    0      ;   self.rew_dist_approach_scale = 200.0
        self.rew_obs_dist_scaled        =    0      ;   self.rew_obs_dist_scale = 0.5
        self.rew_obs_align_scaled       =    0      ;   self.rew_obs_align_scale = 0.5
        self.rew_time                   =  -0.5

        # define variables for storing the state values pulled from simulation:
        self.latest_odom: Odometry      |   None = None
        self.latest_scan: LaserScan     |   None = None

        # instantiate subscribers:
        self.odom_sub = self.create_subscription(Odometry, f"/{self.agent_name}/odom", self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, f"/{self.agent_name}/scan", self.lidar_callback, 10)

        # instantiate publishers:
        self.cmd_pub = self.create_publisher(TwistStamped, f"/{self.agent_name}/cmd_vel", 10)

        # set active goal handle:
        self._current_goal_handle: ServerGoalHandle     |   None = None

        # instantiate action server:
        # self._action_server = ActionServer(
        #     self, 
        #     NavigateToGoal,
        #     "navigate_to_goal",
        #     goal_callback       =   self.goal_callback,
        #     cancel_callback     =   self.cancel_callback,
        #     execute_callback    =   self.execute_callback
        # )

# define main function:
def main():
    # initialize rclpy:
    rclpy.init()

    # instantiate node:
    node = DRLPolicyNode()
    
    # start a multi-threaded executor to prevent blocking code:
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # handle sigterms sent externally (from GUI):
    signal.signal(signal.SIGTERM, lambda *args: executor.shutdown())

    # spinning and shutdown:
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# main:
if __name__ == "__main__":
    main()

