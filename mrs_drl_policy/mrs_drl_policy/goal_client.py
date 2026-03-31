import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from drl_interfaces.action import NavigateToGoal
from geometry_msgs.msg import PoseStamped

# define the goal client node class:
class GoalClient(Node):
    # define the constructor for the node:
    def __init__(self, x: float, y: float, goal_tolerance: float):
        # inherit from parent class:
        super().__init__("goal_client")     # set node name

        # set flag for completion:
        self._done = False

        # instantiate client:
        self._client = ActionClient(
            self, 
            NavigateToGoal,
            "navigate_to_goal"
        )

        # wait for action server to become available:
        self.get_logger().info("Waiting for action server...")
        self._client.wait_for_server()

        # define the goal to be sent:
        goal                                =    NavigateToGoal.Goal()
        goal.target_pose                    =    PoseStamped()
        goal.target_pose.header.stamp       =    self.get_clock().now().to_msg()
        goal.target_pose.header.frame_id    =    'odom'
        goal.target_pose.pose.position.x    =    x
        goal.target_pose.pose.position.y    =    y
        goal.goal_tolerance                 =    goal_tolerance

        # send goal to client:
        self.get_logger().info(f'Sending goal: ({x: 5.3f},{y: 5.3f})')
        self._send_future = self._client.send_goal_async(
            goal,
            feedback_callback = self.feedback_callback
        )
        self._send_future.add_done_callback(self.goal_accepted_callback)

    # define callback for goal acceptance:
    def goal_accepted_callback(self, future : Future):
        # get the goal handle:
        self._goal_handle = future.result()

        # if not accepted:
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        # otherwise:
        self.get_logger().info("Goal accepted :)")
        self._goal_handle.get_result_async().add_done_callback(self.result_callback)

    # define callback for feedback:
    def feedback_callback(self, feedback):
        # instantiate feedback:
        f = feedback.feedback

        # print feedback to user:
        self.get_logger().info(
            f'Time: {f.elapsed_time: 5.2f} | ' 
            f'Distance to goal: {f.distance_to_goal: 5.3f} m'
        )

    # define callback for results:
    def result_callback(self, future):
        # get result:
        result = future.result().result

        # print result to user:
        self.get_logger().info(
            f'{result.message} | Total distance travelled: {result.total_distance: 5.3f}')
        
        # set done flag to true:
        self._done = True

    # define method for cancelling requests:
    def cancel(self):
        # log to user:
        self.get_logger().info('Sending cancel request to the server...')

        # cancel the future:
        cancel_future = self._goal_handle.cancel_goal_async()

        # handle rclpy stuff:
        while rclpy.ok() and not cancel_future.done():
            rclpy.spin_once(self, timeout_sec = 0.1)

        # handle cancel response:
        cancel_response = cancel_future.result()
        if cancel_response:
            self.get_logger().info('Cancel acknowledged by the server.')
        else:
            self.get_logger().warn('Cancel request rejected by the server.')

# define main function:
def main():
    # initialize rclpy:
    rclpy.init(signal_handler_options = rclpy.SignalHandlerOptions.NO)

    # usage example -> ros2 run drl_policy goal_client 3.0 2.0 0.5:
    user_args       =    rclpy.utilities.remove_ros_args(sys.argv)[1:]
    x               =    float(user_args[0]) if len(user_args) > 0 else 0.0
    y               =    float(user_args[1]) if len(user_args) > 1 else 0.0
    goal_tolerance  =    float(user_args[2]) if len(user_args) > 2 else 0.2

    # instantiate node:
    node = GoalClient(x, y, goal_tolerance)

    # spinning and shutdown:
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.cancel()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# main:
if __name__ == "__main__":
    main()