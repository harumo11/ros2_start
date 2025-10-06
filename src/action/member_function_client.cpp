#include "ros2_start_interfaces/action/fibonacci.hpp"
#include <future>
#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_start_interfaces/action/fibonacci.hpp>

class MinimalActionClient : public rclcpp::Node {
public:
    using Fibonacci = ros2_start_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    MinimalActionClient(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
        : Node("minimal_action_client", node_options)
        , goal_done_(false)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Creating action client");
        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MinimalActionClient::send_goal, this));
    }

    bool is_goal_done() const
    {
        return this->goal_done_;
    }

    void send_goal()
    {
        using namespace std::placeholders;
        this->timer_->cancel();
        this->goal_done_ = false;

        if (!this->client_ptr_) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Action client not initialized");
        }

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&MinimalActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&MinimalActionClient::result_callback, this, _1);

        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    bool goal_done_;
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(GoalHandleFibonacci::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleFibonacci::SharedPtr goal_handle, Fibonacci::Feedback::ConstSharedPtr feedback)
    {
        goal_handle->get_goal_id();
        RCLCPP_INFO_STREAM(this->get_logger(), "Next number in sequence received: " << feedback->partial_sequence.back());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult& result)
    {
        this->goal_done_ = true;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO_STREAM(this->get_logger(), "Goal was succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown result code");
            return;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<MinimalActionClient>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(action_client);

    while (rclcpp::ok() && !action_client->is_goal_done()) {
        executor.spin_some();
    }

    return 0;
}
