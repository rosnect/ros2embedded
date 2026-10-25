#include "ros2_embedded/sensor_conversion.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace ros2_embedded
{
namespace
{
using Json = nlohmann::json;
constexpr double unknown = std::numeric_limits<double>::quiet_NaN();

void fields(const Json & value, std::initializer_list<const char *> allowed)
{
  if (!value.is_object()) {throw std::invalid_argument("Sensor value/field must be a JSON object");}
  for (auto it = value.begin(); it != value.end(); ++it) {
    if (std::none_of(allowed.begin(), allowed.end(), [&](const char * key) {return it.key() == key;})) {
      throw std::invalid_argument("Unknown sensor field: " + it.key());
    }
  }
}
Json parse(const TransportPayload & payload)
{
  validate_payload(payload);
  if (payload.type != ValueType::Json) {throw std::invalid_argument("Native sensor messages require value_type=JSON");}
  auto value = Json::parse(std::get<std::string>(payload.value));
  if (!value.is_object()) {throw std::invalid_argument("Native sensor JSON must be an object");}
  return value;
}
double number(const Json & value, bool nullable = false)
{
  if (nullable && value.is_null()) {return unknown;}
  if (!value.is_number()) {throw std::invalid_argument("Sensor scalar must be numeric");}
  const double result = value.get<double>();
  if (!std::isfinite(result)) {throw std::invalid_argument("Non-finite sensor number");}
  return result;
}
std::int64_t integer(const Json & value, std::int64_t minimum, std::int64_t maximum)
{
  if (!value.is_number_integer() || (value.is_number_unsigned() && value.get<std::uint64_t>() > INT64_MAX)) {
    throw std::invalid_argument("Sensor field requires an integer");
  }
  const auto result = value.get<std::int64_t>();
  if (result < minimum || result > maximum) {throw std::invalid_argument("Sensor integer out of range");}
  return result;
}
std::string text(const Json & value, std::size_t limit = 256)
{
  if (!value.is_string()) {throw std::invalid_argument("Sensor field requires a string");}
  const auto result = value.get<std::string>();
  if (result.size() > limit || result.find('\0') != std::string::npos) {throw std::invalid_argument("Invalid sensor string");}
  return result;
}
std_msgs::msg::Header header(const Json & value, std::uint64_t timestamp)
{
  std_msgs::msg::Header result;
  const Json * stamp = nullptr;
  if (value.contains("header")) {
    const auto & h = value.at("header");
    fields(h, {"stamp", "frame_id"});
    if (h.contains("frame_id")) {result.frame_id = text(h.at("frame_id"));}
    if (h.contains("stamp")) {stamp = &h.at("stamp");}
  }
  if (stamp) {
    fields(*stamp, {"sec", "nanosec"});
    result.stamp.sec = static_cast<std::int32_t>(integer(stamp->at("sec"), INT32_MIN, INT32_MAX));
    result.stamp.nanosec = static_cast<std::uint32_t>(integer(stamp->at("nanosec"), 0, 999999999));
  } else {
    if (timestamp / 1000000000ULL > INT32_MAX) {throw std::invalid_argument("Sensor timestamp exceeds ROS Time range");}
    result.stamp.sec = static_cast<std::int32_t>(timestamp / 1000000000ULL);
    result.stamp.nanosec = static_cast<std::uint32_t>(timestamp % 1000000000ULL);
  }
  return result;
}
geometry_msgs::msg::Vector3 vector(const Json & value, bool nullable = false)
{
  fields(value, {"x", "y", "z"});
  geometry_msgs::msg::Vector3 result;
  result.x = number(value.at("x"), nullable);
  result.y = number(value.at("y"), nullable);
  result.z = number(value.at("z"), nullable);
  return result;
}
template<std::size_t N>
std::array<double, N> covariance(const Json & value, bool unavailable_allowed = false)
{
  static_assert(N == 9);
  if (!value.is_array() || value.size() != N) {throw std::invalid_argument("Covariance requires exactly 9 numbers");}
  std::array<double, N> result{};
  for (std::size_t i = 0; i < N; ++i) {result[i] = number(value[i]);}
  if (unavailable_allowed && result[0] == -1) {return result;}
  if (result[0] < 0 || result[4] < 0 || result[8] < 0) {throw std::invalid_argument("Covariance variances cannot be negative");}
  return result;
}
std::array<double, 9> imu_covariance(const Json & value, const char * name, bool present)
{
  std::array<double, 9> result{};
  if (value.contains(name)) {result = covariance<9>(value.at(name), true);}
  if (!present) {
    if (value.contains(name) && result[0] != -1) {throw std::invalid_argument("Missing IMU measurement must have covariance[0]=-1");}
    result[0] = -1;
  }
  return result;
}
Json numeric_json(double value, bool nullable = false)
{
  if (nullable && std::isnan(value)) {return nullptr;}
  if (!std::isfinite(value)) {throw std::invalid_argument("Cannot encode non-finite sensor field");}
  return value;
}
Json header_json(const std_msgs::msg::Header & value)
{
  return {{"stamp", {{"sec", value.stamp.sec}, {"nanosec", value.stamp.nanosec}}}, {"frame_id", value.frame_id}};
}
Json vector_json(const geometry_msgs::msg::Vector3 & value, bool nullable = false)
{
  return {{"x", numeric_json(value.x, nullable)}, {"y", numeric_json(value.y, nullable)}, {"z", numeric_json(value.z, nullable)}};
}
template<typename Message>
std::string checked_json(const Json & value)
{
  const auto encoded = value.dump();
  // Run the same schema/range validation in both directions.
  sensor_from_payload<Message>(TransportPayload{"validation", 0, ValueType::Json, encoded, 0});
  return encoded;
}
float optional_float(const Json & value, const char * key, bool nonnegative = false)
{
  if (!value.contains(key) || value.at(key).is_null()) {return std::numeric_limits<float>::quiet_NaN();}
  const auto n = number(value.at(key));
  if (std::abs(n) > std::numeric_limits<float>::max() || (nonnegative && n < 0)) {
    throw std::invalid_argument(std::string("Invalid battery ") + key);
  }
  return static_cast<float>(n);
}
std::vector<float> cells(const Json & value, bool nonnegative)
{
  if (!value.is_array() || value.size() > 256) {throw std::invalid_argument("Battery cells require an array of at most 256 entries");}
  std::vector<float> result;
  for (const auto & entry : value) {
    const auto n = number(entry, true);
    if ((!std::isnan(n) && nonnegative && n < 0) || std::abs(n) > std::numeric_limits<float>::max()) {
      throw std::invalid_argument("Invalid battery cell value");
    }
    result.push_back(static_cast<float>(n));
  }
  return result;
}
Json cells_json(const std::vector<float> & values)
{
  Json result = Json::array();
  for (auto value : values) {result.push_back(numeric_json(value, true));}
  return result;
}
int base64_digit(char c)
{
  if (c >= 'A' && c <= 'Z') {return c - 'A';}
  if (c >= 'a' && c <= 'z') {return c - 'a' + 26;}
  if (c >= '0' && c <= '9') {return c - '0' + 52;}
  if (c == '+') {return 62;}
  if (c == '/') {return 63;}
  throw std::invalid_argument("Invalid image base64 character");
}
Bytes decode_base64(const Json & value)
{
  if (!value.is_string()) {throw std::invalid_argument("data_base64 requires a string");}
  const auto & wire = value.get_ref<const std::string &>();
  if (wire.empty() || wire.size() % 4 || wire.size() > 4 * ((kMaxCompressedImageBytes + 2) / 3)) {
    throw std::invalid_argument("Invalid image base64 size");
  }
  Bytes result;
  result.reserve(wire.size() / 4 * 3);
  for (std::size_t i = 0; i < wire.size(); i += 4) {
    const int a = base64_digit(wire[i]), b = base64_digit(wire[i + 1]);
    const bool pad2 = wire[i + 2] == '=', pad3 = wire[i + 3] == '=';
    if ((pad2 && !pad3) || ((pad2 || pad3) && i + 4 != wire.size())) {throw std::invalid_argument("Invalid image base64 padding");}
    const int c = pad2 ? 0 : base64_digit(wire[i + 2]);
    const int d = pad3 ? 0 : base64_digit(wire[i + 3]);
    if ((pad2 && (b & 15)) || (!pad2 && pad3 && (c & 3))) {throw std::invalid_argument("Noncanonical image base64 padding bits");}
    result.push_back(static_cast<std::uint8_t>((a << 2) | (b >> 4)));
    if (!pad2) {result.push_back(static_cast<std::uint8_t>((b << 4) | (c >> 2)));}
    if (!pad3) {result.push_back(static_cast<std::uint8_t>((c << 6) | d));}
  }
  if (result.size() > kMaxCompressedImageBytes) {throw std::invalid_argument("Compressed image exceeds 256 KiB");}
  return result;
}
std::string encode_base64(const Bytes & value)
{
  const char * alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string result;
  for (std::size_t i = 0; i < value.size(); i += 3) {
    const std::uint32_t a = value[i], b = i + 1 < value.size() ? value[i + 1] : 0, c = i + 2 < value.size() ? value[i + 2] : 0;
    result += alphabet[a >> 2]; result += alphabet[((a & 3) << 4) | (b >> 4)];
    result += i + 1 < value.size() ? alphabet[((b & 15) << 2) | (c >> 6)] : '=';
    result += i + 2 < value.size() ? alphabet[c & 63] : '=';
  }
  return result;
}
}  // namespace

bool supported_sensor_type(const std::string & type)
{
  return type == "sensor_msgs/msg/Imu" || type == "sensor_msgs/msg/MagneticField" ||
    type == "sensor_msgs/msg/NavSatFix" || type == "sensor_msgs/msg/Illuminance" ||
    type == "sensor_msgs/msg/FluidPressure" || type == "sensor_msgs/msg/BatteryState" ||
    type == "sensor_msgs/msg/CompressedImage";
}

template<> sensor_msgs::msg::Imu sensor_from_payload(const TransportPayload & payload)
{
  const auto j = parse(payload);
  fields(j, {"header", "orientation", "orientation_covariance", "angular_velocity", "angular_velocity_covariance", "linear_acceleration", "linear_acceleration_covariance"});
  const bool orientation = j.contains("orientation"), angular = j.contains("angular_velocity"), acceleration = j.contains("linear_acceleration");
  if (!orientation && !angular && !acceleration) {throw std::invalid_argument("IMU requires a measurement");}
  sensor_msgs::msg::Imu m;
  m.header = header(j, payload.timestamp_unix_ns);
  m.orientation.w = 1;
  m.orientation_covariance = imu_covariance(j, "orientation_covariance", orientation);
  m.angular_velocity_covariance = imu_covariance(j, "angular_velocity_covariance", angular);
  m.linear_acceleration_covariance = imu_covariance(j, "linear_acceleration_covariance", acceleration);
  if (orientation) {
    const auto & q = j.at("orientation"); fields(q, {"x", "y", "z", "w"});
    m.orientation.x = number(q.at("x")); m.orientation.y = number(q.at("y"));
    m.orientation.z = number(q.at("z")); m.orientation.w = number(q.at("w"));
    const double norm = std::hypot(std::hypot(m.orientation.x, m.orientation.y), std::hypot(m.orientation.z, m.orientation.w));
    if (m.orientation_covariance[0] != -1 && std::abs(norm - 1) > 1e-3) {throw std::invalid_argument("Available IMU orientation must be a unit quaternion");}
  }
  if (angular) {m.angular_velocity = vector(j.at("angular_velocity"));}
  if (acceleration) {m.linear_acceleration = vector(j.at("linear_acceleration"));}
  return m;
}
template<> std::string sensor_to_json(const sensor_msgs::msg::Imu & m)
{
  return checked_json<sensor_msgs::msg::Imu>({{"header", header_json(m.header)},
    {"orientation", {{"x", numeric_json(m.orientation.x)}, {"y", numeric_json(m.orientation.y)}, {"z", numeric_json(m.orientation.z)}, {"w", numeric_json(m.orientation.w)}}},
    {"orientation_covariance", m.orientation_covariance}, {"angular_velocity", vector_json(m.angular_velocity)},
    {"angular_velocity_covariance", m.angular_velocity_covariance}, {"linear_acceleration", vector_json(m.linear_acceleration)},
    {"linear_acceleration_covariance", m.linear_acceleration_covariance}});
}
template<> sensor_msgs::msg::MagneticField sensor_from_payload(const TransportPayload & payload)
{
  const auto j = parse(payload); fields(j, {"header", "magnetic_field", "magnetic_field_covariance"});
  sensor_msgs::msg::MagneticField m;
  m.header = header(j, payload.timestamp_unix_ns); m.magnetic_field = vector(j.at("magnetic_field"), true);
  if (j.contains("magnetic_field_covariance")) {m.magnetic_field_covariance = covariance<9>(j.at("magnetic_field_covariance"));}
  return m;
}
template<> std::string sensor_to_json(const sensor_msgs::msg::MagneticField & m)
{
  return checked_json<sensor_msgs::msg::MagneticField>({{"header", header_json(m.header)}, {"magnetic_field", vector_json(m.magnetic_field, true)}, {"magnetic_field_covariance", m.magnetic_field_covariance}});
}
template<> sensor_msgs::msg::NavSatFix sensor_from_payload(const TransportPayload & payload)
{
  const auto j = parse(payload); fields(j, {"header", "status", "latitude", "longitude", "altitude", "position_covariance", "position_covariance_type"});
  sensor_msgs::msg::NavSatFix m;
  m.header = header(j, payload.timestamp_unix_ns); m.status.status = -1;
  if (j.contains("status")) {
    const auto & status = j.at("status"); fields(status, {"status", "service"});
    if (status.contains("status")) {m.status.status = static_cast<std::int8_t>(integer(status.at("status"), -1, 2));}
    if (status.contains("service")) {m.status.service = static_cast<std::uint16_t>(integer(status.at("service"), 0, 15));}
  }
  m.latitude = number(j.at("latitude"), m.status.status == -1);
  m.longitude = number(j.at("longitude"), m.status.status == -1);
  if ((!std::isnan(m.latitude) && std::abs(m.latitude) > 90) || (!std::isnan(m.longitude) && std::abs(m.longitude) > 180)) {throw std::invalid_argument("Latitude/longitude outside WGS84 ranges");}
  m.altitude = j.contains("altitude") ? number(j.at("altitude"), true) : unknown;
  if (j.contains("position_covariance")) {m.position_covariance = covariance<9>(j.at("position_covariance"));}
  if (j.contains("position_covariance_type")) {m.position_covariance_type = static_cast<std::uint8_t>(integer(j.at("position_covariance_type"), 0, 3));}
  if (m.position_covariance_type && !j.contains("position_covariance")) {throw std::invalid_argument("Known position covariance requires its values");}
  return m;
}
template<> std::string sensor_to_json(const sensor_msgs::msg::NavSatFix & m)
{
  return checked_json<sensor_msgs::msg::NavSatFix>({{"header", header_json(m.header)}, {"status", {{"status", m.status.status}, {"service", m.status.service}}},
    {"latitude", numeric_json(m.latitude, true)}, {"longitude", numeric_json(m.longitude, true)}, {"altitude", numeric_json(m.altitude, true)},
    {"position_covariance", m.position_covariance}, {"position_covariance_type", m.position_covariance_type}});
}
template<> sensor_msgs::msg::Illuminance sensor_from_payload(const TransportPayload & payload)
{
  const auto j = parse(payload); fields(j, {"header", "illuminance", "variance"});
  sensor_msgs::msg::Illuminance m;
  m.header = header(j, payload.timestamp_unix_ns); m.illuminance = number(j.at("illuminance"));
  if (j.contains("variance")) {m.variance = number(j.at("variance"));}
  if (m.illuminance < 0 || m.variance < 0) {throw std::invalid_argument("Illuminance and variance cannot be negative");}
  return m;
}
template<> std::string sensor_to_json(const sensor_msgs::msg::Illuminance & m)
{
  return checked_json<sensor_msgs::msg::Illuminance>({{"header", header_json(m.header)}, {"illuminance", numeric_json(m.illuminance)}, {"variance", numeric_json(m.variance)}});
}
template<> sensor_msgs::msg::FluidPressure sensor_from_payload(const TransportPayload & payload)
{
  const auto j = parse(payload); fields(j, {"header", "fluid_pressure", "variance"});
  sensor_msgs::msg::FluidPressure m;
  m.header = header(j, payload.timestamp_unix_ns); m.fluid_pressure = number(j.at("fluid_pressure"));
  if (j.contains("variance")) {m.variance = number(j.at("variance"));}
  if (m.fluid_pressure < 0 || m.variance < 0) {throw std::invalid_argument("Absolute pressure and variance cannot be negative");}
  return m;
}
template<> std::string sensor_to_json(const sensor_msgs::msg::FluidPressure & m)
{
  return checked_json<sensor_msgs::msg::FluidPressure>({{"header", header_json(m.header)}, {"fluid_pressure", numeric_json(m.fluid_pressure)}, {"variance", numeric_json(m.variance)}});
}
template<> sensor_msgs::msg::BatteryState sensor_from_payload(const TransportPayload & payload)
{
  const auto j = parse(payload);
  fields(j, {"header", "voltage", "temperature", "current", "charge", "capacity", "design_capacity", "percentage", "power_supply_status", "power_supply_health", "power_supply_technology", "present", "cell_voltage", "cell_temperature", "location", "serial_number"});
  sensor_msgs::msg::BatteryState m; m.header = header(j, payload.timestamp_unix_ns);
  if (!j.contains("voltage") || j.at("voltage").is_null()) {throw std::invalid_argument("Battery voltage is required");}
  m.voltage = optional_float(j, "voltage", true); m.temperature = optional_float(j, "temperature");
  m.current = optional_float(j, "current"); m.charge = optional_float(j, "charge", true);
  m.capacity = optional_float(j, "capacity", true); m.design_capacity = optional_float(j, "design_capacity", true);
  m.percentage = optional_float(j, "percentage");
  // Check the original number as well, before float32 rounding can hide a bad fraction.
  if (j.contains("percentage") && !j.at("percentage").is_null()) {
    const double percentage = number(j.at("percentage"));
    if (percentage < 0 || percentage > 1) {throw std::invalid_argument("Battery percentage must be a 0..1 fraction");}
  }
  if (j.contains("power_supply_status")) {m.power_supply_status = static_cast<std::uint8_t>(integer(j.at("power_supply_status"), 0, 4));}
  if (j.contains("power_supply_health")) {m.power_supply_health = static_cast<std::uint8_t>(integer(j.at("power_supply_health"), 0, 8));}
  if (j.contains("power_supply_technology")) {m.power_supply_technology = static_cast<std::uint8_t>(integer(j.at("power_supply_technology"), 0, 6));}
  if (j.contains("present")) {
    if (!j.at("present").is_boolean()) {throw std::invalid_argument("Battery present must be boolean");}
    m.present = j.at("present").get<bool>();
  }
  if (j.contains("cell_voltage")) {m.cell_voltage = cells(j.at("cell_voltage"), true);}
  if (j.contains("cell_temperature")) {m.cell_temperature = cells(j.at("cell_temperature"), false);}
  if (j.contains("location")) {m.location = text(j.at("location"));}
  if (j.contains("serial_number")) {m.serial_number = text(j.at("serial_number"));}
  return m;
}
template<> std::string sensor_to_json(const sensor_msgs::msg::BatteryState & m)
{
  return checked_json<sensor_msgs::msg::BatteryState>({{"header", header_json(m.header)},
    {"voltage", numeric_json(m.voltage)}, {"temperature", numeric_json(m.temperature, true)}, {"current", numeric_json(m.current, true)},
    {"charge", numeric_json(m.charge, true)}, {"capacity", numeric_json(m.capacity, true)}, {"design_capacity", numeric_json(m.design_capacity, true)},
    {"percentage", numeric_json(m.percentage, true)}, {"power_supply_status", m.power_supply_status}, {"power_supply_health", m.power_supply_health},
    {"power_supply_technology", m.power_supply_technology}, {"present", m.present}, {"cell_voltage", cells_json(m.cell_voltage)},
    {"cell_temperature", cells_json(m.cell_temperature)}, {"location", m.location}, {"serial_number", m.serial_number}});
}
template<> sensor_msgs::msg::CompressedImage sensor_from_payload(const TransportPayload & payload)
{
  const auto j = parse(payload); fields(j, {"header", "format", "data_base64", "data"});
  sensor_msgs::msg::CompressedImage m;
  m.header = header(j, payload.timestamp_unix_ns); m.format = text(j.at("format"), 128);
  if (m.format.empty() || j.contains("data_base64") == j.contains("data")) {throw std::invalid_argument("Image requires a format and exactly one of data_base64/data");}
  if (j.contains("data_base64")) {m.data = decode_base64(j.at("data_base64"));}
  else {
    const auto & bytes = j.at("data");
    if (!bytes.is_array() || bytes.empty() || bytes.size() > kMaxCompressedImageBytes) {throw std::invalid_argument("Invalid compressed image byte array");}
    for (const auto & byte : bytes) {m.data.push_back(static_cast<std::uint8_t>(integer(byte, 0, 255)));}
  }
  return m;
}
template<> std::string sensor_to_json(const sensor_msgs::msg::CompressedImage & m)
{
  if (m.data.empty() || m.data.size() > kMaxCompressedImageBytes) {throw std::invalid_argument("Compressed image must contain 1..256 KiB");}
  return checked_json<sensor_msgs::msg::CompressedImage>({{"header", header_json(m.header)}, {"format", m.format}, {"data_base64", encode_base64(m.data)}});
}
}  // namespace ros2_embedded
