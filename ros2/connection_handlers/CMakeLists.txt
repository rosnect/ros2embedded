cmake_minimum_required(VERSION 3.8)
project(connection_handlers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Protobuf REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED IMPORTED_TARGET libmosquitto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  ../../proto/bridge_config.proto ../../proto/bridge_payload.proto)

add_library(connection_handlers SHARED
  ${PROTO_SRCS}
  src/payload.cpp
  src/basic_connection_handler.cpp
  src/serial_connection_handler.cpp
  src/mqtt_connection_handler.cpp
  src/redis_connection_handler.cpp
)

target_compile_features(connection_handlers PUBLIC cxx_std_17)
target_include_directories(connection_handlers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(connection_handlers pluginlib rclcpp)
target_include_directories(connection_handlers PRIVATE ${MOSQUITTO_INCLUDE_DIRS})
target_link_libraries(connection_handlers protobuf::libprotobuf nlohmann_json::nlohmann_json ${MOSQUITTO_LINK_LIBRARIES})

pluginlib_export_plugin_description_file(connection_handlers connection_handler_plugins.xml)

install(FILES ${PROTO_HDRS} DESTINATION include)
install(DIRECTORY ../../proto/ DESTINATION share/${PROJECT_NAME}/proto)

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
ament_export_dependencies(pluginlib rclcpp Protobuf nlohmann_json)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_payload test/test_payload.cpp)
  target_link_libraries(test_payload connection_handlers util)
  ament_add_gtest(test_arduino
    ../../embedded/arduino/test/test_arduino.cpp
    ../../embedded/arduino/src/Ros2EmbeddedBridge.cpp)
  target_include_directories(test_arduino PRIVATE
    ../../embedded/arduino/test ../../embedded/arduino/src)
  target_link_libraries(test_arduino connection_handlers)
endif()
ament_package()
