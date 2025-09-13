#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
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

        RCLCPP_INFO_STREAM(this->get_logger(), "Creating Action Server");
        this->action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&MinimalActionServer::handle_goal, this, _1, _2),
            std::bind(&MinimalActionServer::handle_cancel, this, _1),
            std::bind(&MinimalActionServer::handle_accepted, this, _1));
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
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto& sequence = feedback->partial_sequence;
        sequence.push_back(0);
        sequence.push_back(1);
        auto result = std::make_shared<Fibonacci::Result>();

        for (int i = 1; (i < goal->order) && rclcpp::ok(); i++) {
            // check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO_STREAM(this->get_logger(), "Goal canceled");
                return;
            }

            // update sequence
            sequence.push_back(sequence[i] + sequence[i - 1]);
            // publish Feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO_STREAM(this->get_logger(), "Publish feedback: " << sequence.back());

            // sleep for 1 sec
            loop_rate.sleep();
        }

        // check if goal is done
        if (rclcpp::ok()) {
            result->sequence = sequence;
        }
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread { std::bind(&MinimalActionServer::execute, this, std::placeholders::_1), goal_handle }.detach();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<MinimalActionServer>();
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}
