cmake_minimum_required(VERSION 3.8)
project(bridge_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(connection_handlers REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bridge_node src/bridge_node.cpp)
target_compile_features(bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(bridge_node
  connection_handlers
  pluginlib
  rclcpp
  std_msgs
)

install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
