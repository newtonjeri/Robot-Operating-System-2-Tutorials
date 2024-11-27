#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_custom_interfaces/action/count_until.hpp"

class CountUntilServer : public rclcpp::Node {

    public:
      /**
       * @brief Constructor for the CountUntilServer class. Initializes the ROS node and action server.
       *
       * This constructor creates an action server for the "count_until" action. It binds the goal,
       * cancel, and accepted callback functions to handle incoming goals and cancel requests.
       *
       * @param[in] Node::Node("count_until_server_node") The name of the ROS node.
       */
      CountUntilServer(): Node("count_until_server_node"){
        // Create an action server.
        count_until_server_ = rclcpp_action::create_server<ros2_custom_interfaces::action::CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServer::goalCallback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CountUntilServer::cancelCallback, this, std::placeholders::_1),
            std::bind(&CountUntilServer::handleAcceptedCallback, this, std::placeholders::_1)
            );

        RCLCPP_INFO(this->get_logger(), "CountUntil Server is ready to accept goals.");
      }



    private:
      /**
       * @brief Callback function to handle incoming goal requests.
       *
       * This function is called when a new goal is received by the action server.
       * It determines whether the goal should be accepted or rejected.
       *
       * @param[in] uuid The unique identifier for the goal request.
       * @param[in] goal A shared pointer to the goal message containing the target number and period.
       * 
       * @return rclcpp_action::GoalResponse The response indicating whether the goal is accepted or rejected.
       *         In this implementation, the goal is always accepted and executed.
       */
      rclcpp_action::GoalResponse goalCallback(
          const rclcpp_action::GoalUUID &uuid, 
          std::shared_ptr<const ros2_custom_interfaces::action::CountUntil::Goal> goal){
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }


      /**
       * @brief Callback function to handle cancel requests for goals.
       *
       * This function is called when a cancel request is received for a goal.
       * It determines whether the cancel request should be accepted or rejected.
       *
       * @param[in] goal_handle A shared pointer to the goal handle for which the cancel request is made.
       * 
       * @return rclcpp_action::CancelResponse The response indicating whether the cancel request is accepted or rejected.
       *         In this implementation, the cancel request is always accepted.
       */
      rclcpp_action::CancelResponse cancelCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_custom_interfaces::action::CountUntil>> goal_handle
      ){
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
      }

      void handleAcceptedCallback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_custom_interfaces::action::CountUntil>> goal_handle){
        executeGoal(goal_handle);
      }

      void executeGoal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_custom_interfaces::action::CountUntil>> goal_handle
      ){
        int target_number = goal_handle->get_goal()->target_number;

        double period = goal_handle->get_goal()->period;

        RCLCPP_INFO(this->get_logger(), "Executing CountUntil action, target number: %d, period: %.2f seconds", target_number, period);
        int counter = 0;
        rclcpp::Rate loop_rate(1.0 / period);
        for (int i = 0;  i < target_number; i++)
        {
          counter++;
          RCLCPP_INFO(this->get_logger(), "Current count: %d", counter);
          loop_rate.sleep();
        }

        // Set final state and return
        auto result = std::make_shared<ros2_custom_interfaces::action::CountUntil::Result>();
        result->reached_number = counter;
        goal_handle->succeed(result);
      }

      rclcpp_action::Server<ros2_custom_interfaces::action::CountUntil>::SharedPtr count_until_server_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CountUntilServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
