#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_action/server.hpp>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_start_interfaces/action/fibonacci.hpp>

class MinimalActionServer : public rclcpp::Node {
public:
    using Fibonacci = ros2_start_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    MinimalActionServer(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("minimal_action_server", options)
    {
        using namespace std::placeholders;

        // Create an action server with the name "fibonacci"
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    // callback on goal request from client, check if goal is acceptable
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request with order " << goal->order);

        if (goal->order > 9000) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Goal rejected, order too high: " << goal->order);
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // callback on goal cancel request from client
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // callback on goal accepted by server, execute the goal
    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO_SKIPFIRST(this->get_logger(), "Executing goal");
    }
};

int main(int argc, char** argv) { rclcpp::init(argc, argv); }
