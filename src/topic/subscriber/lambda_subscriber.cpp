#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>

class LambdaSubscriber : public rclcpp::Node {
  public:

  LambdaSubscriber() : Node("lambda_subscriber") {
    auto on_received = [this](std_msgs::msg::String::UniquePtr) -> void {
        RCLCPP_INFO_STREAM(get_logger(), "Received message");
    };
    this->subscriber = this->create_subscription<std_msgs::msg::String>("topic", 10, on_received);
  }; 

  private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Starting lambda_subscriber node");

  auto node = std::make_shared<LambdaSubscriber>();

  while (rclcpp::ok()) {
      rclcpp::spin_some(node);
  }

  return 0;
}
