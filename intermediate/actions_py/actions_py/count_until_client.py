#! /usr/bin/env python3	
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from ros2_custom_interfaces.action import CountUntil

class CountUntilClientNode(Node):
    def __init__(self):
        """
        Initialize the CountUntilClientNode class.

        This class is responsible for creating an action client to communicate with the 
        'count_until' action server. It sets up the necessary components for sending goals, 
        receiving feedback, and handling the response.

        Parameters:
        None

        Returns:
        None
        """
        super().__init__('count_until_client_node')
        self.count_until_client = ActionClient(self, CountUntil, "count_until")

    def send_goal(self, target_number=10, period=1.0):
        """
        Send a goal to the 'count_until' action server.

        This function sends a goal to the action server with the specified target number and period.
        It waits for the server to be ready, creates a goal message, sends the goal, and sets up a 
        timer to cancel the goal after 5 seconds.

        Parameters:
        target_number (int, optional): The target number for the count until action. Defaults to 10.
        period (float, optional): The period for counting. Defaults to 1.0.

        Returns:
        None
        """
        # Wait for server to be ready
        self.count_until_client.wait_for_server()

        # Create goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # Send goal
        self.get_logger().info(f"Sending goal: {target_number}")
        self.count_until_client. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback). \
                add_done_callback(self.goal_response_callback)

        # Send a cancel request 5 seconds after goal has been sent
        self.timer_ = self.create_timer(5.0, self.cancel_goal)
    
    def cancel_goal(self):
        """
        Cancels the currently active goal.

        This function sends a cancel request to the action server for the currently active goal.
        It logs an information message indicating the cancellation, cancels the goal asynchronously,
        and cancels the timer that was set to automatically cancel the goal after 5 seconds.

        Parameters:
        None

        Returns:
        None
        """
        self.get_logger().info("Cancelling goal")
        self.goal_handle.cancel_goal_async()
        self.timer_.cancel()

    def goal_response_callback(self, future):
        """
        Callback function for handling the response from the action server after sending a goal.

        This function is called when the action server responds to a goal sent by the client.
        It retrieves the goal handle from the future result, checks if the goal was accepted,
        logs the appropriate message, and sets up a callback for handling the goal result.

        Parameters:
        future (rclpy.Future): The future object containing the result of the goal.

        Returns:
        None
        """
        self.goal_handle: ClientGoalHandle = future.result()

        if self.goal_handle.accepted:
            self.get_logger().info("Goal accepted")
            self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal rejected")
    
    def goal_result_callback(self, future):
        """
        Callback function for handling the result from the action server after sending a goal.

        This function is called when the action server responds to a goal sent by the client.
        It retrieves the goal handle from the future result, checks the status of the goal,
        logs the appropriate message, and retrieves the goal result.

        Parameters:
        future (rclpy.Future): The future object containing the result of the goal.

        Returns:
        None
        """
        status = future.result().status
        result = future.result().result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Goal aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal canceled")
        self.get_logger().info(f"Result: {result.reached_number}")

    def goal_feedback_callback(self, feedback_msg):
        """
        Callback function for handling feedback from the action server during a goal.

        This function is called when the action server provides feedback during the execution of a goal.
        It retrieves the current number from the feedback message and logs it using the node's logger.

        Parameters:
        feedback_msg (CountUntil.Feedback): The feedback message containing the current number.

        Returns:
        None
        """
        number = feedback_msg.feedback.current_number
        self.get_logger().info(f"Current number: {number}")


def main(args=None):
    """
    The main function initializes the ROS2 environment, creates an instance of the CountUntilClientNode,
    prompts the user for input, sends a goal to the 'count_until' action server, and then spins the node.

    Parameters:
    args (list, optional): Command-line arguments to be passed to rclpy.init(). Defaults to None.

    Returns:
    None
    """
    rclpy.init(args=args)
    client_node = CountUntilClientNode()

    # Ask for input from user
    target_number = input("Enter target number: ")
    period = input("Enter period: ")

    client_node.send_goal(int(target_number), float(period))

    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()