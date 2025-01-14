#include <rclcpp/executors.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class LambdaPublisher : public rclcpp::Node {
public:
  LambdaPublisher(): Node("lambda_publisher") {
      this->publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);

      auto on_publish_timer_expierd = [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!" + std::to_string(this->count++);
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.data << "'");
        this->publisher->publish(message);
      };

      this->timer = this->create_wall_timer(500ms, on_publish_timer_expierd);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
  int count = 0;
};


int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    std::cout << "Starting lambda_publisher node" << std::endl;
    auto node = std::make_shared<LambdaPublisher>();


    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        RCLCPP_INFO_STREAM(node->get_logger(), "Spinning");
    }
}

