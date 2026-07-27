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
import torch, torch.nn as nn            # type: ignore
from stable_baselines3 import TD3, SAC  # type: ignore
from ament_index_python.packages import get_package_share_directory
import os
import sys
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
        self.declare_parameter("agent_initial_yaw", 0.0)
        self.declare_parameter('max_lin_vel', 0.6)
        self.declare_parameter('max_angular_vel', 0.9)
        self.declare_parameter('goal_timeout', 60.0)

        # add parameters to class:
        self.agent_name                     =   self.get_parameter("agent_name").value
        self.default_goal_tolerance         =   self.get_parameter('goal_tolerance').value
        self.default_obstacle_tolerance     =   self.get_parameter('obstacle_tolerance').value
        self.model_name                     =   self.get_parameter('model_name').value
        self.agent_initial_yaw              =   self.get_parameter("agent_initial_yaw").value
        self.model_type                     =   self.model_name.split('_')[0]
        self.max_lin_vel                    =   self.get_parameter('max_lin_vel').value
        self.max_angular_vel                =   self.get_parameter('max_angular_vel').value
        self.goal_timeout                   =   self.get_parameter('goal_timeout').value

        # get the paths:
        pkg_dir             =    get_package_share_directory("mrs_drl_policy")
        model_dir           =    os.path.join(pkg_dir, "policy", self.model_name)
        model_path          =    os.path.join(model_dir, "model")
        norm_stat_path      =    os.path.join(model_dir, "norm_stats.pkl")

        # enable the use of cuda if available:
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # load the model, if available:
        if self.model_type == "TD3":
            try:
                model = TD3.load(model_path, device = self.device)
            except FileNotFoundError:
                print("No such model!")
                sys.exit(0)
        elif self.model_type == "SAC":
            try:
                model = SAC.load(model_path, device = self.device)
            except FileNotFoundError:
                print("No such model!")
                sys.exit(0)
        
        # get the policy from the model, set to evaluation (inference):
        self.policy = model.policy.actor
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

        self.rew_head_approach_scaled   =    0      ;   self.rew_head_approach_scale    =   200.0
        self.rew_dist_approach_scaled   =    0      ;   self.rew_dist_approach_scale    =   200.0
        self.rew_obs_dist_scaled        =    0      ;   self.rew_obs_dist_scale         =   0.5
        self.rew_obs_align_scaled       =    0      ;   self.rew_obs_align_scale        =   0.5
        self.rew_time                   =  -0.5

        # define variables for storing the state values pulled from simulation:
        self.latest_odom: Odometry      |   None = None
        self.latest_scan: LaserScan     |   None = None

        # instantiate subscribers:
        self.odom_sub   =   self.create_subscription(Odometry, f"/{self.agent_name}/odom", self.odom_callback, 10)
        self.lidar_sub  =   self.create_subscription(LaserScan, f"/{self.agent_name}/scan", self.lidar_callback, 10)

        # instantiate publishers:
        self.cmd_pub    =   self.create_publisher(TwistStamped, f"/{self.agent_name}/cmd_vel", 10)

        # set active goal handle:
        self._current_goal_handle: ServerGoalHandle     |   None = None

        # instantiate action server:
        self._action_server = ActionServer(
            self, 
            NavigateToGoal,
            "navigate_to_goal",
            goal_callback       =   self.goal_callback,
            cancel_callback     =   self.cancel_callback,
            execute_callback    =   self.execute_callback
        )

    # define odometry callback:
    def odom_callback(self, msg : Odometry):
        # apply the latest odom:
        self.latest_odom = msg

    # define lidar callback:
    def lidar_callback(self, msg : LaserScan):
        # pull range data from msg:
        ranges = np.array(msg.ranges)

        # get the index of the zero angle in the (-pi, pi) scan:
        zero_idx = round(-msg.angle_min / msg.angle_increment)

        # shift the lidar data from (-pi, pi) to (0, 2pi), so that it aligns with MuJoCo:
        ranges_rotated = np.roll(ranges, -zero_idx)

        # populate latest scan:
        self.latest_scan            =   copy.deepcopy(msg)
        self.latest_scan.ranges     =   ranges_rotated.tolist()

    # define goal callback:
    def goal_callback(self, goal_request : NavigateToGoal):
        # log that goal has been received:
        self.get_logger().info(
            f"New goal received at: ({goal_request.target_pose.pose.position.x: 5.3f},"
            f"{goal_request.target_pose.pose.position.y: 5.3f})"
        )

        # overwrite any current goal:
        if self._current_goal_handle is not None and self._current_goal_handle.is_active:
            self.get_logger().info('Changing the current goal.')
            self._current_goal_handle.abort()

        # return that the goal has been accepted:
        return GoalResponse.ACCEPT
    
    # define cancellation callback:
    def cancel_callback(self, goal_handle):
        # log that the goal has been cancelled:
        self.get_logger().info("Cancel requested")

        # return a cancel response:
        return CancelResponse.ACCEPT

    # define the main execution callback:
    async def execute_callback(self, goal_handle : ServerGoalHandle):
        # set control frequency and period:
        ctrl_freq       =    50
        ctrl_period     =    1.0 / ctrl_freq

        # wait for data to come in:
        while (self.latest_odom is None) or (self.latest_scan is None):
            self.get_logger().warn('Waiting for odometry and LiDAR...')
            time.sleep(ctrl_period)
            continue

        # get the goal handle:
        self._current_goal_handle = goal_handle

        # pull target pose from goal handle:
        target = goal_handle.request.target_pose

        # apply goal tolerance:
        self.goal_tolerance = (goal_handle.request.goal_tolerance
                               if goal_handle.request.goal_tolerance > 0.0
                               else self.default_goal_tolerance)
        
        # apply obstacle tolerance:
        self.obstacle_tolerance = self.default_obstacle_tolerance

        # start timer for control loop:
        start = time.time()

        # initialize feedback and result messages:
        feedback_msg    =    NavigateToGoal.Feedback()
        result_msg      =    NavigateToGoal.Result()

        # get previous values, total distance travelled:
        x_prev          =    self.latest_odom.pose.pose.position.x
        y_prev          =    self.latest_odom.pose.pose.position.y
        total_distance  =    0.0

        # while spinning:
        while rclpy.ok():
            # timing for control loop:
            ctrl_iter_start     =    time.time()
            elapsed_time        =    time.time() - start

            # check for cancel requests through goal handle:
            if goal_handle.is_cancel_requested:
                # apply cancel:
                goal_handle.canceled()

                # send cmd vel to stop agent:
                cmd                 =    TwistStamped()
                cmd.header.stamp    =    self.get_clock().now().to_msg()

                # try to publish the cmd vel:
                try:
                    self.cmd_pub.publish(cmd)
                except Exception:
                    pass

                # prepare & return a result message:
                result_msg.success = False
                result_msg.message = "Cancelled by client."
                return result_msg
            
            # if goal handle becomes inactive:
            if not goal_handle.is_active:
                # stop the agent and bail cleanly:
                cmd              = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()

                # try to publish the cmd vel:
                try:
                    self.cmd_pub.publish(cmd)
                except Exception:
                    pass

                # formulate a result message:
                result_msg.success = False
                result_msg.message = "Goal preempted."
                return result_msg
            
            # check to see if goal timeout has been hit:
            if elapsed_time >= self.goal_timeout:
                # abort via goal handle:
                goal_handle.abort()
                cmd                 =   TwistStamped()
                cmd.header.stamp    =   self.get_clock().now().to_msg()

                # try to publish the cmd vel:
                try:
                    self.cmd_pub.publish(cmd)
                except Exception:
                    pass

                # prepare & return a result message:
                result_msg.success          =   False
                result_msg.message          =   f"Goal timeout after {elapsed_time:.1f}s"
                result_msg.total_distance   =   float(total_distance)
                self.get_logger().warn(f"Goal timed out after {elapsed_time:.1f}s")
                return result_msg

            ##### MAIN DRL LOOP #####
            # 1) extract and normalize observation:
            obs         =    self._get_obs(self.latest_odom, target, self.latest_scan)
            obs_normed  =    self._normalize_obs(obs)

            # 2) check the termination conditions:
            d_goal      =   obs[2]
            min_lidar   =   np.min(obs[9:])

            # if at the goal:
            if d_goal <= self.goal_tolerance:
                # stop agent via goal handle:
                cmd                 =    TwistStamped()
                cmd.header.stamp    =    self.get_clock().now().to_msg()

                # try to publish the cmd vel:
                try:
                    self.cmd_pub.publish(cmd)
                except Exception:
                    pass

                # report success on goal handle:
                goal_handle.succeed()

                # prepare & return a result message:
                result_msg.success          =   True
                result_msg.message          =   'Goal reached.'
                result_msg.total_distance   =   float(total_distance)
                # self.get_logger().info('Goal reached.')
                return result_msg
            
            # if the agent "hits" an obstacle:
            if min_lidar <= self.obstacle_tolerance:
                # stop agent via goal handle:
                cmd                 =    TwistStamped()
                cmd.header.stamp    =    self.get_clock().now().to_msg()

                # try to publish the cmd vel:
                try:
                    self.cmd_pub.publish(cmd)
                except Exception:
                    pass

                # abort the goal handle:
                goal_handle.abort()

                # prepare & return a result message:
                result_msg.success          =    False
                result_msg.message          =    'Obstacle hit, mission aborted.'
                result_msg.total_distance   =    float(total_distance)
                self.get_logger().info(f'Obstacle hit with min_lidar = {min_lidar: 5.3f}, mission aborted.')
                return result_msg
        
            # 3) DRL policy inference and action selection:
            self.action         =   self._run_policy(obs_normed)        # which gives vx, vyaw in moving agent frame
            self.action[0]      =   np.clip(self.action[0], 0.0, 1.0)
            self.action[1]      =   np.clip(self.action[1], -1.0, 1.0)
            self._get_rewards(obs)

            # prepare the cmd_vel message:
            cmd                         =    TwistStamped()
            cmd.header.stamp            =    self.get_clock().now().to_msg()
            cmd.header.frame_id         =    f"{self.agent_name}_base_link"
            cmd.twist.linear.x          =    float(self.action[0]) * self.max_lin_vel
            cmd.twist.angular.z         =    float(self.action[1]) * self.max_angular_vel

            # try to publish the cmd vel:
            try:
                self.cmd_pub.publish(cmd)
            except Exception:
                result_msg.success = False
                result_msg.message = "Preempted mid-publish."
                return result_msg

            ##### EXTRAS #####
            # 4) publish feedback:
            feedback_msg.distance_to_goal   =    float(d_goal)
            feedback_msg.elapsed_time       =    float(elapsed_time)
            feedback_msg.current_pose       =    self.latest_odom.pose.pose

            try:
                goal_handle.publish_feedback(feedback_msg)
            except Exception:
                result_msg.success = False
                result_msg.message = "Preempted mid-publish."
                return result_msg

            # 5) find total distance travelled over the previous step:
            x               =   self.latest_odom.pose.pose.position.x
            y               =   self.latest_odom.pose.pose.position.y
            step_distance   =   np.sqrt((x - x_prev) ** 2 + (y - y_prev) ** 2)
            total_distance  +=  step_distance
            x_prev, y_prev  =   x, y

            # 6) manage control frequency:
            ctrl_iter_elapsed   =    time.time() - ctrl_iter_start
            ctrl_iter_remain    =    ctrl_period - ctrl_iter_elapsed
            if ctrl_iter_remain > 0:
                time.sleep(ctrl_iter_remain)
      
    # define function for determining yaw from quaternion measurement:
    def _yaw_from_quaternion(self, q: Quaternion) -> float:
        siny_cosp   =    2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp   =    1.0 - 2.0 * (q.y**2 + q.z**2)
        return np.atan2(siny_cosp, cosy_cosp, dtype=np.float32)

    # define function for getting the observation:
    def _get_obs(self, odom: Odometry, target: PoseStamped, scan: LaserScan) -> np.ndarray:
        """
        this method builds the observation from subscribed /odom messages to match the observation
        space of the policy, which contains:

        (dx, dy, dgoal, s_theta, c_theta, s_phi, c_phi, vx, vy, LiDAR scans)
        
        """ 
        # extract raw odometry and sensor data -> IN THE ODOM FRAME:
        agent_pos       =   odom.pose.pose.position             # agent (x, y, z) in initial odom frame
        ori_quat        =   odom.pose.pose.orientation          # orientation quaternion, in the odom frame

        # get the total angle of the agent -> IN THE GLOBAL FRAME:
        agent_yaw       =   self.agent_initial_yaw + self._yaw_from_quaternion(ori_quat)
        agent_vx        =   self.action[0] # agent's velocity in the moving frame
        agent_vyaw      =   self.action[1] # agent's yaw velocity about the Z axis in the moving frame

        # define goal posiiton -> IN THE GLOBAL FRAME:
        goal_pos        =   target.pose.position  # position of the target

        # perform a transform from rotated odom frame -> GLOBAL FRAME:
        cos_spawn = np.cos(self.agent_initial_yaw)
        sin_spawn = np.sin(self.agent_initial_yaw)
        agent_x_global   =   cos_spawn * agent_pos.x - sin_spawn * agent_pos.y
        agent_y_global   =   sin_spawn * agent_pos.x + cos_spawn * agent_pos.y

        # perform the calculations required to form the observation -> IN THE GLOBAL FRAME:
        dx              =   goal_pos.x - agent_x_global
        dy              =   goal_pos.y - agent_y_global
        dgoal           =   np.sqrt(dx**2 + dy**2)

        bearing         =   np.arctan2(dy, dx, dtype = np.float32) % (2 * np.pi)
        heading         =   agent_yaw
        rel_bearing     =   -((bearing - heading + np.pi) % (2*np.pi) - np.pi)

        c_heading       =   np.cos(heading, dtype = np.float32)
        s_heading       =   np.sin(heading, dtype = np.float32) 
        c_bearing       =   np.cos(rel_bearing, dtype = np.float32)
        s_bearing       =   np.sin(rel_bearing, dtype = np.float32)

        # LiDAR min-pooling:
        raw                 =    np.array(scan.ranges, dtype = np.float32)
        raw                 =    np.where(np.isfinite(raw), raw, scan.range_max)    # replace inf/nan with max LiDAR range values
        antennae_mask       =    raw < 0.15
        raw[antennae_mask]  =    scan.range_max                                     # drop anything below the minimum LiDAR scan range 
        raw                 =    np.clip(raw, 0.0, scan.range_max)
        raw                 =    np.flip(raw)
        n_groups            =    18         
        lidar_groups        =    np.array_split(raw, n_groups)
        lidar_obs           =    np.array([g.min() for g in lidar_groups], dtype = np.float32)

        # form observation:
        self._obs_buffer[0:3]   =    dx, dy, dgoal
        self._obs_buffer[3:5]   =    c_heading, s_heading
        self._obs_buffer[5:7]   =    c_bearing, s_bearing
        self._obs_buffer[7:9]   =    agent_vx, agent_vyaw
        self._obs_buffer[9:]    =    lidar_obs
        
        return self._obs_buffer

    # define function for normalizing observation:
    def _normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        # reshape because vecnormalize expects a batch dimension representing parallel environments:
        obs_batch = obs.reshape(1,-1)

        # normalize:
        obs_normalized = self.vec_norm.normalize_obs(obs_batch)

        return obs_normalized.reshape(-1).astype(np.float32)

    # define function for getting rewards:
    def _get_rewards(self, obs):
        # pull values for rewards:
        d_goal          =    obs[2]
        abs_diff        =    np.arctan2(obs[6], obs[5], dtype = np.float32)
        lidar_obs       =    obs[9:]
        min_dist        =    np.min(lidar_obs)
        min_dist_idx    =    np.argmin(lidar_obs)
        v_lin, v_ang    =    obs[7:9]

        # if within the safety range:
        if min_dist >= self.d_safe:
            #--- DISTANCE APPROACH REWARD:
            # this reward term incentivizes approaching the goal, and rewards 0 otherwise:
            rew_dist_approach   =   max((self.d_goal_last - d_goal), 0)

            #--- HEADING APPROACH REWARD:
            # this reward term incentivizes approaching the required heading, and rewards 0 otherwise
            rew_head_approach   =   max((self.prev_abs_diff - abs_diff), 0) if self.action[0] >= 0.05 else 0

            # zero the obstacle terms:
            rew_obs_dist        =   0
            rew_obs_align       =   0

        # when near an obstacle, focus on moving away:
        else:
            #--- OBSTACLE APPROACH PENALTY:
            rew_obs_dist        =   min((min_dist / (self.min_dist_last + 1e-6) - 1), 0)

            # REWARD FOR NOT BEING ALIGNED WITH OBSTACLES:
            if min_dist_idx >= self.lidar_idx_threshold and min_dist_idx < self.n_ray_groups - self.lidar_idx_threshold and v_lin >= 0.05:
                rew_obs_align   =    1
            else:
                if (min_dist_idx < self.lidar_idx_threshold and v_ang > 0) or (min_dist_idx >= self.n_ray_groups - self.lidar_idx_threshold and v_ang < 0):
                    rew_obs_align   =   min(1, np.abs(v_ang))
                else:
                    rew_obs_align   =   0

            # zero the approach terms:
            rew_dist_approach = 0
            rew_head_approach = 0

        #--- PENALIZE ABRUPT CHANGES IN VELOCITY:
        act_diff        =   np.abs(self.action - self.action_last)
        rew_act_diff    =   -0.5 * np.sum(act_diff ** 2)

        #--- PENALIZE STALLING:
        if abs(self.action[0]) <= 0.05:
            rew_time = 2 * self.rew_time
        else:
            rew_time = self.rew_time

        # scaling:
        self.rew_dist_approach_scaled   =   self.rew_dist_approach_scale  * rew_dist_approach
        self.rew_head_approach_scaled   =   self.rew_head_approach_scale  * rew_head_approach
        self.rew_obs_dist_scaled        =   self.rew_obs_dist_scale       * rew_obs_dist
        self.rew_obs_align_scaled       =   self.rew_obs_align_scale      * rew_obs_align

        #--- TOTAL REWARD:
        rew     =       0
        rew     +=      self.rew_dist_approach_scaled + self.rew_head_approach_scaled
        rew     +=      self.rew_obs_dist_scaled + self.rew_obs_align_scaled
        rew     +=      rew_act_diff
        rew     +=      rew_time

        # advance histories:
        self.d_goal_last    =    d_goal
        self.prev_abs_diff  =    abs_diff
        self.min_dist_last  =    min_dist
        self.action_last    =    self.action

    # define function for actually using the policy:
    def _run_policy(self, obs: np.ndarray) -> np.ndarray:
        # turn off gradient updates:
        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs).unsqueeze(0).to(self.device)
            action = self.policy(obs_tensor)
        return action.squeeze(0).cpu().numpy()

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

