#pragma once

#include "ros2_embedded/payload.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ros2_embedded
{
constexpr std::size_t kMaxCompressedImageBytes = 256 * 1024;
bool supported_sensor_type(const std::string & type);

// Explicit supported schemas, selected by ros_message_type, never device identity.
// JSON values carry ROS-shaped fields; missing measurements remain unknown.
template<typename Message> Message sensor_from_payload(const TransportPayload & payload);
template<typename Message> std::string sensor_to_json(const Message & message);

#define ROS2_EMBEDDED_SENSOR_CODEC(Message) \
  template<> Message sensor_from_payload<Message>(const TransportPayload & payload); \
  template<> std::string sensor_to_json<Message>(const Message & message);
ROS2_EMBEDDED_SENSOR_CODEC(sensor_msgs::msg::Imu)
ROS2_EMBEDDED_SENSOR_CODEC(sensor_msgs::msg::MagneticField)
ROS2_EMBEDDED_SENSOR_CODEC(sensor_msgs::msg::NavSatFix)
ROS2_EMBEDDED_SENSOR_CODEC(sensor_msgs::msg::Illuminance)
ROS2_EMBEDDED_SENSOR_CODEC(sensor_msgs::msg::FluidPressure)
ROS2_EMBEDDED_SENSOR_CODEC(sensor_msgs::msg::BatteryState)
ROS2_EMBEDDED_SENSOR_CODEC(sensor_msgs::msg::CompressedImage)
#undef ROS2_EMBEDDED_SENSOR_CODEC
}  // namespace ros2_embedded
