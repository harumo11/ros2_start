cmake_minimum_required(VERSION 3.12)
project(ros2_start)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_start_interfaces REQUIRED)


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

add_executable(member_function_action_server ./src/action/member_function_server.cpp)
target_link_libraries(member_function_action_server PUBLIC
    ${ros2_start_interfaces_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
)

add_executable(member_function_action_client ./src/action/member_function_client.cpp)
target_link_libraries(member_function_action_client PUBLIC
    ${ros2_start_interfaces_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
)

install(TARGETS
  lambda_publisher
  lambda_subscriber
  member_function_action_server
  member_function_action_client
  DESTINATION lib/${PROJECT_NAME}
)


# for test
if(BUILD_TESTING)
    find_package(ament_cmake_ros REQUIRED)
    FILE(GLOB_RECURSE PY_TESTS test/*.py)
    #ament_add_pytest_test(test_node ${PY_TESTS})

    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_lint_cmake_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
endif()


# copy compile_commands.json to workspace root
file(CREATE_LINK 
    ${CMAKE_BINARY_DIR}/compile_commands.json 
    ${CMAKE_SOURCE_DIR}/compile_command.json 
    SYMBOLIC
)

ament_package()
