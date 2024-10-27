#! /usr/bin/env python3

import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import (
    ServerGoalHandle, 
    GoalResponse, 
    CancelResponse)

from rclpy.callback_groups import ReentrantCallbackGroup
from ros2_custom_interfaces.action import CountUntil

class CountUntilServerNode(Node):
    def __init__(self):
        """
        Initializes the CountUntilServerNode class.

        This constructor sets up the action server for the CountUntil action.
        It creates an instance of the ActionServer class, specifying the action type,
        server name, and callback functions for goal handling, execution, and cancellation.
        It also logs a message indicating that the action server has started and is waiting for goals.

        Parameters:
        None

        Returns:
        None
        """
        super().__init__('count_until_server_node')

        self.count_until_server = ActionServer(
            self,
            CountUntil,
            "count_until",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("Action server started. Waiting for goal.")

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        Executes the action server's main logic.

        This function is called when a new goal is received by the action server.
        It counts up to the target number at the specified period, publishing feedback
        and checking for cancellation requests.

        Parameters:
        goal_handle (ServerGoalHandle): The handle for the received goal. Contains the goal request.

        Returns:
        CountUntil.Result: The result of the action, containing the reached number.
        """
        # Get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Feedback 
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()

        # Action logic
        # Wait for user input to start counting up to the goal number
        self.get_logger().info(f"Received goal: {target_number}")
        counter = 0
        # Execute the action
        for i in range(target_number):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal Cancelled!")
                goal_handle.canceled()
                result.reached_number = counter
                return result

            counter = counter + 1
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Counting up to {target_number} - {counter}")
            time.sleep(period)

        # Once done, Set goal final state
        goal_handle.succeed()
        # Send result
        self.get_logger().info(f"Goal reached: {target_number}")
        result = CountUntil.Result()
        result.reached_number = counter

        return result
    
    def goal_callback(self, goal_request: CountUntil.Goal):
        """
        Callback function for handling incoming goal requests.

        This function is called whenever a new goal is received by the action server.
        It validates the goal request and either accepts or rejects it based on the target number.

        Parameters:
        goal_request (CountUntil.Goal): The incoming goal request containing the target number and period.

        Returns:
        GoalResponse: The response indicating whether the goal was accepted or rejected.
        """
        self.get_logger().info(f"Received goal: {goal_request.target_number}")

        # Validate goal request
        if goal_request.target_number <= 0:
            self.get_logger().warn("Goal Rejected! Target number must be greater than 0") 
            return GoalResponse.REJECT

        self.get_logger().info("Goal Accepted!")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """
        Callback function for handling cancellation requests.

        This function is called whenever a goal cancellation request is received by the action server.
        It simply accepts the cancellation request and returns the appropriate response.

        Parameters:
        goal_handle (ServerGoalHandle): The handle for the goal that is being cancelled.

        Returns:
        CancelResponse: The response indicating whether the cancellation request was accepted.
        In this case, it always returns CancelResponse.ACCEPT.
        """
        return CancelResponse.ACCEPT


def main(args=None):
    """
    The main function initializes the ROS2 environment, creates an instance of the CountUntilServerNode,
    spins the node using a MultiThreadedExecutor, destroys the node, and shuts down the ROS2 environment.

    Parameters:
    args (list, optional): A list of command-line arguments to be passed to rclpy.init().
                           Defaults to None, which means no arguments are passed.

    Returns:
    None
    """
    rclpy.init(args=args)
    server_node = CountUntilServerNode()
    rclpy.spin(server_node, MultiThreadedExecutor())
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()