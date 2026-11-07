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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(sensor_conversion STATIC src/sensor_conversion.cpp)
target_compile_features(sensor_conversion PUBLIC cxx_std_17)
target_include_directories(sensor_conversion PUBLIC include)
ament_target_dependencies(sensor_conversion connection_handlers std_msgs sensor_msgs geometry_msgs)
target_link_libraries(sensor_conversion nlohmann_json::nlohmann_json)

add_executable(bridge_node src/bridge_node.cpp)
target_compile_features(bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(bridge_node
  connection_handlers
  pluginlib
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(bridge_node sensor_conversion)

install(TARGETS bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sensor_conversion test/test_sensor_conversion.cpp)
  target_link_libraries(test_sensor_conversion sensor_conversion)
endif()
ament_package()
