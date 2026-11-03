#pragma once

#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace bridge_sensor_json
{
using Json = nlohmann::json;
inline bool supported(const std::string & name)
{
  return name == "sensor_msgs/msg/Imu" || name == "sensor_msgs/msg/MagneticField" ||
    name == "sensor_msgs/msg/NavSatFix" || name == "sensor_msgs/msg/Illuminance" ||
    name == "sensor_msgs/msg/FluidPressure" || name == "sensor_msgs/msg/CompressedImage" ||
    name == "sensor_msgs/msg/BatteryState";
}
inline double number(const Json & j, const char * key, double fallback = 0)
{
  if (!j.contains(key)) {return fallback;}
  if (!j.at(key).is_number()) {throw std::invalid_argument(std::string("Expected number: ") + key);}
  const auto value = j.at(key).get<double>();
  if (!std::isfinite(value)) {throw std::invalid_argument("Non-finite sensor value");}
  return value;
}
inline double required_number(const Json & j, const char * key)
{
  (void)j.at(key); return number(j, key);
}
template<typename Vector>
void vector(Vector & out, const Json & j)
{
  out.x = required_number(j, "x"); out.y = required_number(j, "y"); out.z = required_number(j, "z");
}
template<typename Array>
void covariance(Array & out, const Json & j, const char * key)
{
  if (!j.contains(key)) {return;}
  const auto & a = j.at(key);
  if (!a.is_array() || a.size() != out.size()) {throw std::invalid_argument("Invalid covariance length");}
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!a[i].is_number() || !std::isfinite(a[i].get<double>())) {throw std::invalid_argument("Invalid covariance value");}
    out[i] = a[i].get<double>();
  }
}
inline std::vector<std::uint8_t> decode_base64(const std::string & input)
{
  if (input.empty() || input.size() % 4 != 0 || input.size() > 349528) {throw std::invalid_argument("Invalid or oversized base64 image");}
  const std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::vector<std::uint8_t> result;
  result.reserve(input.size() / 4 * 3);
  for (std::size_t i = 0; i < input.size(); i += 4) {
    std::uint32_t word = 0;
    int padding = 0;
    for (int k = 0; k < 4; ++k) {
      const char c = input[i + k];
      if (c == '=') {
        if (i + 4 != input.size() || k < 2) {throw std::invalid_argument("Invalid base64 padding");}
        ++padding; word <<= 6;
      } else {
        const auto value = alphabet.find(c);
        if (padding || value == std::string::npos) {throw std::invalid_argument("Invalid base64 character");}
        word = (word << 6) | static_cast<std::uint32_t>(value);
      }
    }
    if (padding > 2 || (padding == 2 && (word & 0x00ffff)) || (padding == 1 && (word & 0x0000ff))) {
      throw std::invalid_argument("Non-canonical base64 padding");
    }
    result.push_back((word >> 16) & 255);
    if (padding < 2) {result.push_back((word >> 8) & 255);}
    if (padding < 1) {result.push_back(word & 255);}
  }
  if (result.size() > 256 * 1024) {throw std::invalid_argument("Image exceeds 256 KiB");}
  return result;
}
template<typename Message>
Message decode(const std::string & text, std::uint64_t timestamp_ns)
{
  const auto j = Json::parse(text);
  if (!j.is_object()) {throw std::invalid_argument("Sensor JSON must be an object");}
  Message out;
  if (timestamp_ns / 1000000000 > INT32_MAX) {throw std::invalid_argument("Timestamp out of ROS range");}
  out.header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000);
  out.header.stamp.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1000000000);
  if (j.contains("header")) {
    const auto & h = j.at("header");
    out.header.frame_id = h.value("frame_id", std::string{});
    if (out.header.frame_id.size() > 96) {throw std::invalid_argument("frame_id exceeds 96 characters");}
    if (h.contains("stamp")) {
      const auto & stamp = h.at("stamp");
      if (!stamp.at("sec").is_number_integer() || !stamp.at("nanosec").is_number_integer()) {throw std::invalid_argument("Invalid timestamp");}
      const auto sec = stamp.at("sec").get<std::int64_t>();
      const auto ns = stamp.at("nanosec").get<std::int64_t>();
      if (sec < 0 || sec > INT32_MAX || ns < 0 || ns >= 1000000000) {throw std::invalid_argument("Timestamp out of ROS range");}
      out.header.stamp.sec = static_cast<std::int32_t>(sec); out.header.stamp.nanosec = static_cast<std::uint32_t>(ns);
    }
  }
  if constexpr (std::is_same_v<Message, sensor_msgs::msg::CompressedImage>) {
    out.format = j.value("format", std::string("jpeg"));
    if (out.format != "jpeg" && out.format != "png") {throw std::invalid_argument("Only jpeg/png images supported");}
    if (j.contains("data_base64")) {out.data = decode_base64(j.at("data_base64").get<std::string>());}
    else {
      const auto & bytes = j.at("data");
      if (!bytes.is_array() || bytes.empty() || bytes.size() > 256 * 1024) {throw std::invalid_argument("Invalid image bytes");}
      for (const auto & v : bytes) {
        if (!v.is_number_integer() || v.get<std::int64_t>() < 0 || v.get<std::int64_t>() > 255) {throw std::invalid_argument("Invalid image byte");}
        out.data.push_back(v.get<std::uint8_t>());
      }
    }
    if (out.data.empty()) {throw std::invalid_argument("Empty image");}
  } else if constexpr (std::is_same_v<Message, sensor_msgs::msg::Imu>) {
    if (!j.contains("orientation")) {out.orientation_covariance[0] = -1;}
    else {vector(out.orientation, j.at("orientation")); out.orientation.w = required_number(j.at("orientation"), "w");}
    if (j.contains("angular_velocity")) {vector(out.angular_velocity, j.at("angular_velocity"));}
    else {out.angular_velocity_covariance[0] = -1;}
    if (j.contains("linear_acceleration")) {vector(out.linear_acceleration, j.at("linear_acceleration"));}
    else {out.linear_acceleration_covariance[0] = -1;}
    covariance(out.orientation_covariance, j, "orientation_covariance");
    covariance(out.angular_velocity_covariance, j, "angular_velocity_covariance");
    covariance(out.linear_acceleration_covariance, j, "linear_acceleration_covariance");
  } else if constexpr (std::is_same_v<Message, sensor_msgs::msg::MagneticField>) {
    vector(out.magnetic_field, j.at("magnetic_field")); covariance(out.magnetic_field_covariance, j, "magnetic_field_covariance");
  } else if constexpr (std::is_same_v<Message, sensor_msgs::msg::NavSatFix>) {
    out.latitude = required_number(j, "latitude"); out.longitude = required_number(j, "longitude");
    if (out.latitude < -90 || out.latitude > 90 || out.longitude < -180 || out.longitude > 180) {throw std::invalid_argument("GPS coordinates out of range");}
    out.altitude = number(j, "altitude", std::numeric_limits<double>::quiet_NaN());
    const auto status = j.value("status", Json::object());
    const auto fix = status.value("status", 0); const auto service = status.value("service", 0);
    if (fix < -1 || fix > 2 || service < 0 || service > 15) {throw std::invalid_argument("Invalid NavSat status");}
    out.status.status = fix; out.status.service = service;
    covariance(out.position_covariance, j, "position_covariance");
    const auto covariance_type = j.value("position_covariance_type", 0);
    if (covariance_type < 0 || covariance_type > 3) {throw std::invalid_argument("Invalid GPS covariance type");}
    out.position_covariance_type = covariance_type;
  } else if constexpr (std::is_same_v<Message, sensor_msgs::msg::Illuminance>) {
    out.illuminance = required_number(j, "illuminance"); out.variance = number(j, "variance");
    if (out.illuminance < 0 || out.variance < 0) {throw std::invalid_argument("Invalid illuminance");}
  } else if constexpr (std::is_same_v<Message, sensor_msgs::msg::FluidPressure>) {
    out.fluid_pressure = required_number(j, "fluid_pressure"); out.variance = number(j, "variance");
    if (out.fluid_pressure < 0 || out.variance < 0) {throw std::invalid_argument("Invalid pressure");}
  } else if constexpr (std::is_same_v<Message, sensor_msgs::msg::BatteryState>) {
    const auto unknown = std::numeric_limits<float>::quiet_NaN();
    out.voltage = number(j, "voltage", unknown); out.temperature = number(j, "temperature", unknown);
    out.current = number(j, "current", unknown); out.charge = number(j, "charge", unknown);
    out.capacity = number(j, "capacity", unknown); out.design_capacity = number(j, "design_capacity", unknown);
    out.percentage = number(j, "percentage", unknown);
    if (std::isfinite(out.percentage) && (out.percentage < 0 || out.percentage > 1)) {throw std::invalid_argument("Battery percentage must be 0..1");}
    out.present = j.value("present", true);
    const auto state = j.value("power_supply_status", 0);
    if (state < 0 || state > 4) {throw std::invalid_argument("Invalid battery state");}
    out.power_supply_status = state;
  }
  return out;
}
}  // namespace bridge_sensor_json
