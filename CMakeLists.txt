cmake_minimum_required(VERSION 3.12)
project(ros2_start)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lambda_publisher ./src/topic/publisher/lambda_publisher.cpp)
ament_target_dependencies(lambda_publisher rclcpp std_msgs)

add_executable(lambda_subscriber ./src/topic/subscriber/lambda_subscriber.cpp)
ament_target_dependencies(lambda_subscriber rclcpp std_msgs)

install(TARGETS
  lambda_publisher
  lambda_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
