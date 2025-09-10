cmake_minimum_required(VERSION 3.12)
project(ros2_start)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(lambda_publisher ./src/topic/publisher/lambda_publisher.cpp)
target_link_libraries(lambda_publisher PUBLIC 
    ${std_msgs_TARGETS} 
    rclcpp::rclcpp
)

add_executable(lambda_subscriber ./src/topic/subscriber/lambda_subscriber.cpp)
target_link_libraries(lambda_subscriber PUBLIC
    ${std_msgs_TARGETS} 
    rclcpp::rclcpp
)

install(TARGETS
  lambda_publisher
  lambda_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
