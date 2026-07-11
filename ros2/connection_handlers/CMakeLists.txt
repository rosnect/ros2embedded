cmake_minimum_required(VERSION 3.8)
project(connection_handlers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(connection_handlers SHARED
  src/basic_connection_handler.cpp
  src/serial_connection_handler.cpp
  src/mqtt_connection_handler.cpp
  src/redis_connection_handler.cpp
)

target_compile_features(connection_handlers PUBLIC cxx_std_17)
target_include_directories(connection_handlers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(connection_handlers pluginlib rclcpp)

pluginlib_export_plugin_description_file(connection_handlers connection_handler_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS connection_handlers
  EXPORT export_connection_handlers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES connection_handler_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(connection_handlers)
ament_export_targets(export_connection_handlers)
ament_export_dependencies(pluginlib rclcpp)
ament_package()
