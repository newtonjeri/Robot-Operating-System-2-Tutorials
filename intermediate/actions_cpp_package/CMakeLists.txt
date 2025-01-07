cmake_minimum_required(VERSION 3.8)
project(actions_cpp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_custom_interfaces REQUIRED)


add_executable(count_until_server_node src/count_until_server.cpp)
ament_target_dependencies(
  count_until_server_node
  "rclcpp"
  "rclcpp_action"
  "ros2_custom_interfaces"
)

add_executable(count_until_client_node src/count_until_client.cpp)
ament_target_dependencies(
  count_until_client_node
  "rclcpp"
  "rclcpp_action"
  "ros2_custom_interfaces"
)

install(
  TARGETS
    count_until_server_node
    count_until_client_node
  DESTINATION 
    lib/${PROJECT_NAME}
)

ament_package()
