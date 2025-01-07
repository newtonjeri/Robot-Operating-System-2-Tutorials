#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ros2_custom_interfaces/action/count_until.hpp"

using namespace std::placeholders;

class CountUntilClientNode : public rclcpp::Node
{
public:
    CountUntilClientNode() : Node("count_until_client_node")
    {
        count_until_client_ = rclcpp_action::create_client<ros2_custom_interfaces::action::CountUntil>(
            this,
            "count_until");
    }

    /**
     * @brief Sends a goal to the Count Until action server.
     *
     * This function sends a goal to the Count Until action server, specifying
     * a target number to count up to and the period between each count.
     * It waits for the action server to become available before sending the goal.
     *
     * @param target_number The number to count up to.
     * @param period The time interval (in seconds) between each count.
     */
    void sendGoal(int target_number, double period)
    {
        // Wait for Server
        count_until_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Action server is ready to accept goal.");
        auto goal = ros2_custom_interfaces::action::CountUntil::Goal();
        goal.target_number = target_number;
        goal.period = period;

        auto options = rclcpp_action::Client<ros2_custom_interfaces::action::CountUntil>::SendGoalOptions();
        options.result_callback = std::bind(&CountUntilClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = std::bind(&CountUntilClientNode::goal_response_callback, this, _1);

        RCLCPP_INFO(this->get_logger(), "Action client sending goal.");
        count_until_client_->async_send_goal(goal, options);
    }

private:
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<ros2_custom_interfaces::action::CountUntil>::SharedPtr& goal_handle){
        if(!goal_handle){
            RCLCPP_WARN(this->get_logger(), "Goal was REJECTED!!!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal ACCEPTED!!!");
        }
    }

    void goal_result_callback(const rclcpp_action::ClientGoalHandle<ros2_custom_interfaces::action::CountUntil>::WrappedResult &result){
        auto reached_number = result.result->reached_number;
        RCLCPP_INFO(this->get_logger(), "Action client received result: Reached number: %ld", reached_number);
    }

    rclcpp_action::Client<ros2_custom_interfaces::action::CountUntil>::SharedPtr count_until_client_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>();
    node->sendGoal(-10, 1.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}