#include "ros2_embedded/payload.hpp"
#include "bridge_payload.pb.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace ros2_embedded
{
namespace
{
using Json = nlohmann::json;
const char * type_name(ValueType type)
{
  switch (type) {
    case ValueType::Bool: return "boolValue";  // Existing Flutter contract.
    case ValueType::Int32: return "int32";
    case ValueType::UInt32: return "uint32";
    case ValueType::Int64: return "int64";
    case ValueType::UInt64: return "uint64";
    case ValueType::Float32: return "float32";
    case ValueType::Float64: return "float64";
    case ValueType::String: return "string";
    case ValueType::Bytes: return "bytes";
    case ValueType::Json: return "json";
  }
  throw std::invalid_argument("Unsupported value type");
}
ValueType parse_type(const std::string & name)
{
  for (int code = 1; code <= 10; ++code) {
    auto type = static_cast<ValueType>(code);
    if (name == type_name(type)) {return type;}
  }
  throw std::invalid_argument("Unknown value_type");
}
std::uint64_t unsigned_number(const Json & value)
{
  if (value.is_number_unsigned()) {return value.get<std::uint64_t>();}
  if (value.is_number_integer() && value.get<std::int64_t>() >= 0) {
    return static_cast<std::uint64_t>(value.get<std::int64_t>());
  }
  throw std::invalid_argument("Expected unsigned integer");
}
std::int64_t signed_number(const Json & value)
{
  if (!value.is_number_integer() ||
    (value.is_number_unsigned() && value.get<std::uint64_t>() > INT64_MAX))
  {
    throw std::invalid_argument("Expected signed integer");
  }
  return value.get<std::int64_t>();
}
void append_le(Bytes & bytes, std::uint64_t bits, std::size_t count)
{
  for (std::size_t i = 0; i < count; ++i) {bytes.push_back((bits >> (8 * i)) & 0xff);}
}
std::uint64_t read_le(const Bytes & bytes)
{
  std::uint64_t bits = 0;
  for (std::size_t i = 0; i < bytes.size(); ++i) {bits |= std::uint64_t(bytes[i]) << (8 * i);}
  return bits;
}
unsigned hex_digit(char c)
{
  if (c >= '0' && c <= '9') {return c - '0';}
  if (c >= 'a' && c <= 'f') {return c - 'a' + 10;}
  if (c >= 'A' && c <= 'F') {return c - 'A' + 10;}
  throw std::invalid_argument("Invalid serial hex");
}
std::uint32_t decimal(const std::string & text)
{
  if (text.empty() || text.size() > 10) {throw std::invalid_argument("Invalid serial integer");}
  std::uint64_t value = 0;
  for (char c : text) {
    if (c < '0' || c > '9') {throw std::invalid_argument("Invalid serial integer");}
    value = value * 10 + (c - '0');
  }
  if (value > UINT32_MAX) {throw std::invalid_argument("Serial integer overflow");}
  return static_cast<std::uint32_t>(value);
}
}  // namespace

bool valid_identifier(const std::string & identifier)
{
  return !identifier.empty() && identifier.size() <= kMaxIdentifierBytes &&
         std::all_of(identifier.begin(), identifier.end(), [](unsigned char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-' || c == '/';
  });
}
void validate_payload(const TransportPayload & p)
{
  if (!valid_identifier(p.identifier)) {throw std::invalid_argument("Invalid identifier");}
  bool valid = false;
  switch (p.type) {
    case ValueType::Bool: valid = std::holds_alternative<bool>(p.value); break;
    case ValueType::Int32:
    case ValueType::Int64:
      if (auto v = std::get_if<std::int64_t>(&p.value)) {
        valid = p.type == ValueType::Int64 || (*v >= INT32_MIN && *v <= INT32_MAX);
      }
      break;
    case ValueType::UInt32:
    case ValueType::UInt64:
      if (auto v = std::get_if<std::uint64_t>(&p.value)) {
        valid = p.type == ValueType::UInt64 || *v <= UINT32_MAX;
      }
      break;
    case ValueType::Float32:
    case ValueType::Float64:
      if (auto v = std::get_if<double>(&p.value)) {
        valid = std::isfinite(*v) &&
          (p.type == ValueType::Float64 || std::abs(*v) <= std::numeric_limits<float>::max());
      }
      break;
    case ValueType::String:
    case ValueType::Json:
      if (auto v = std::get_if<std::string>(&p.value)) {
        // Parsing also rejects invalid UTF-8 when generating the JSON envelope.
        valid = v->size() <= (p.type == ValueType::Json ? kMaxJsonValueBytes : kMaxValueBytes);
        if (p.type == ValueType::Json) {
          valid = valid && Json::accept(*v);
          if (valid) {valid = !Json::parse(*v).is_null();}
        }
        else {Json(*v).dump();}
      }
      break;
    case ValueType::Bytes:
      if (auto v = std::get_if<Bytes>(&p.value)) {valid = v->size() <= (p.type == ValueType::Json ? kMaxJsonValueBytes : kMaxValueBytes);}
      break;
  }
  if (!valid) {throw std::invalid_argument("Value does not match type or exceeds limit");}
}
std::string encode_json(const TransportPayload & p)
{
  validate_payload(p);
  Json value;
  std::visit([&value](const auto & v) {value = v;}, p.value);
  if (p.type == ValueType::Json) {value = Json::parse(std::get<std::string>(p.value));}
  Json envelope{{"identifier", p.identifier}, {"domain_id", p.domain_id},
    {"value_type", type_name(p.type)}, {"value", value}, {"timestamp_unix_ms", nullptr}};
  if (p.timestamp_unix_ns) {envelope["timestamp_unix_ms"] = p.timestamp_unix_ns / 1000000;}
  auto wire = envelope.dump();
  if (wire.size() > kMaxWireBytes) {throw std::invalid_argument("Envelope too large");}
  return wire;
}
TransportPayload decode_json(const std::string & wire)
{
  if (wire.size() > kMaxWireBytes) {throw std::invalid_argument("Envelope too large");}
  auto json = Json::parse(wire);
  TransportPayload p;
  p.identifier = json.at("identifier").get<std::string>();
  auto domain = unsigned_number(json.at("domain_id"));
  if (domain > UINT32_MAX) {throw std::invalid_argument("Domain out of range");}
  p.domain_id = static_cast<std::uint32_t>(domain);
  p.type = parse_type(json.at("value_type").get<std::string>());
  const auto & v = json.at("value");
  switch (p.type) {
    case ValueType::Bool:
      if (!v.is_boolean()) {throw std::invalid_argument("Expected boolean");}
      p.value = v.get<bool>(); break;
    case ValueType::Int32: case ValueType::Int64: p.value = signed_number(v); break;
    case ValueType::UInt32: case ValueType::UInt64: p.value = unsigned_number(v); break;
    case ValueType::Float32: case ValueType::Float64:
      if (!v.is_number()) {throw std::invalid_argument("Expected number");}
      p.value = v.get<double>(); break;
    case ValueType::String: p.value = v.get<std::string>(); break;
    case ValueType::Json:
      if (v.is_null()) {throw std::invalid_argument("Flutter values cannot be null");}
      p.value = v.dump(); break;
    case ValueType::Bytes: {
      if (!v.is_array() || v.size() > kMaxValueBytes) {throw std::invalid_argument("Expected byte array");}
      Bytes bytes;
      for (const auto & entry : v) {
        auto number = unsigned_number(entry);
        if (number > 255) {throw std::invalid_argument("Byte out of range");}
        bytes.push_back(static_cast<std::uint8_t>(number));
      }
      p.value = std::move(bytes); break;
    }
  }
  if (json.contains("timestamp_unix_ms") && !json["timestamp_unix_ms"].is_null()) {
    auto ms = unsigned_number(json["timestamp_unix_ms"]);
    if (ms > UINT64_MAX / 1000000) {throw std::invalid_argument("Timestamp overflow");}
    p.timestamp_unix_ns = ms * 1000000;
  }
  validate_payload(p);
  return p;
}
std::string encode_protobuf(const TransportPayload & p)
{
  validate_payload(p);
  ros2embedded::v1::BridgePayload proto;
  proto.set_identifier(p.identifier); proto.set_domain_id(p.domain_id);
  proto.set_timestamp_unix_ns(p.timestamp_unix_ns);
  switch (p.type) {
    case ValueType::Bool: proto.set_bool_value(std::get<bool>(p.value)); break;
    case ValueType::Int32: proto.set_int32_value(std::get<std::int64_t>(p.value)); break;
    case ValueType::UInt32: proto.set_uint32_value(std::get<std::uint64_t>(p.value)); break;
    case ValueType::Int64: proto.set_int64_value(std::get<std::int64_t>(p.value)); break;
    case ValueType::UInt64: proto.set_uint64_value(std::get<std::uint64_t>(p.value)); break;
    case ValueType::Float32: proto.set_float32_value(std::get<double>(p.value)); break;
    case ValueType::Float64: proto.set_float64_value(std::get<double>(p.value)); break;
    case ValueType::String: proto.set_string_value(std::get<std::string>(p.value)); break;
    case ValueType::Json: proto.set_json_value(std::get<std::string>(p.value)); break;
    case ValueType::Bytes: {
      const auto & bytes = std::get<Bytes>(p.value);
      proto.set_bytes_value(std::string(bytes.begin(), bytes.end())); break;
    }
  }
  return proto.SerializeAsString();
}
TransportPayload decode_protobuf(const std::string & wire)
{
  ros2embedded::v1::BridgePayload proto;
  if (wire.size() > kMaxWireBytes || !proto.ParseFromString(wire)) {
    throw std::invalid_argument("Invalid protobuf envelope");
  }
  TransportPayload p;
  p.identifier = proto.identifier(); p.domain_id = proto.domain_id();
  p.timestamp_unix_ns = proto.timestamp_unix_ns();
  switch (proto.value_case()) {
    case ros2embedded::v1::BridgePayload::kBoolValue:
      p.type = ValueType::Bool; p.value = proto.bool_value(); break;
    case ros2embedded::v1::BridgePayload::kInt32Value:
      p.type = ValueType::Int32; p.value = std::int64_t(proto.int32_value()); break;
    case ros2embedded::v1::BridgePayload::kUint32Value:
      p.type = ValueType::UInt32; p.value = std::uint64_t(proto.uint32_value()); break;
    case ros2embedded::v1::BridgePayload::kInt64Value:
      p.type = ValueType::Int64; p.value = proto.int64_value(); break;
    case ros2embedded::v1::BridgePayload::kUint64Value:
      p.type = ValueType::UInt64; p.value = proto.uint64_value(); break;
    case ros2embedded::v1::BridgePayload::kFloat32Value:
      p.type = ValueType::Float32; p.value = double(proto.float32_value()); break;
    case ros2embedded::v1::BridgePayload::kFloat64Value:
      p.type = ValueType::Float64; p.value = proto.float64_value(); break;
    case ros2embedded::v1::BridgePayload::kStringValue:
      p.type = ValueType::String; p.value = proto.string_value(); break;
    case ros2embedded::v1::BridgePayload::kJsonValue:
      p.type = ValueType::Json; p.value = proto.json_value(); break;
    case ros2embedded::v1::BridgePayload::kBytesValue:
      p.type = ValueType::Bytes; p.value = Bytes(proto.bytes_value().begin(), proto.bytes_value().end()); break;
    default: throw std::invalid_argument("Missing protobuf value");
  }
  validate_payload(p); return p;
}
std::string encode_serial(const TransportPayload & p)
{
  validate_payload(p);
  Bytes bytes;
  switch (p.type) {
    case ValueType::Bool: bytes.push_back(std::get<bool>(p.value) ? 1 : 0); break;
    case ValueType::Int32: case ValueType::Int64:
      append_le(bytes, static_cast<std::uint64_t>(std::get<std::int64_t>(p.value)), p.type == ValueType::Int32 ? 4 : 8); break;
    case ValueType::UInt32: case ValueType::UInt64:
      append_le(bytes, std::get<std::uint64_t>(p.value), p.type == ValueType::UInt32 ? 4 : 8); break;
    case ValueType::Float32: {
      float value = std::get<double>(p.value); std::uint32_t bits;
      std::memcpy(&bits, &value, sizeof(bits)); append_le(bytes, bits, 4); break;
    }
    case ValueType::Float64: {
      double value = std::get<double>(p.value); std::uint64_t bits;
      std::memcpy(&bits, &value, sizeof(bits)); append_le(bytes, bits, 8); break;
    }
    case ValueType::String: case ValueType::Json: {
      const auto & value = std::get<std::string>(p.value);
      bytes.assign(value.begin(), value.end()); break;
    }
    case ValueType::Bytes: bytes = std::get<Bytes>(p.value); break;
  }
  if (bytes.size() > kMaxSerialValueBytes) {throw std::invalid_argument("Serial value exceeds 256 bytes");}
  std::string line = "R2E," + std::to_string(p.domain_id) + "," +
    std::to_string(static_cast<unsigned>(p.type)) + "," + p.identifier + ",";
  const char * hex = "0123456789ABCDEF";
  for (auto byte : bytes) {line += hex[byte >> 4]; line += hex[byte & 15];}
  return line + "\n";
}
TransportPayload decode_serial(const std::string & input)
{
  if (input.size() > 640) {throw std::invalid_argument("Serial frame too large");}
  auto line = input;
  if (!line.empty() && line.back() == '\n') {line.pop_back();}
  if (!line.empty() && line.back() == '\r') {line.pop_back();}
  std::vector<std::string> parts;
  std::size_t start = 0;
  for (std::size_t i = 0; i < line.size(); ++i) {
    if (line[i] == ',') {parts.push_back(line.substr(start, i - start)); start = i + 1;}
  }
  parts.push_back(line.substr(start));
  if (parts.size() != 5 || parts[0] != "R2E") {throw std::invalid_argument("Invalid serial frame");}
  TransportPayload p;
  p.domain_id = decimal(parts[1]);
  auto code = decimal(parts[2]);
  if (code < 1 || code > 10) {throw std::invalid_argument("Invalid serial type");}
  p.type = static_cast<ValueType>(code); p.identifier = parts[3];
  const auto & hex = parts[4];
  if (hex.size() % 2 || hex.size() > kMaxSerialValueBytes * 2) {throw std::invalid_argument("Invalid serial length");}
  Bytes bytes;
  for (std::size_t i = 0; i < hex.size(); i += 2) {bytes.push_back((hex_digit(hex[i]) << 4) | hex_digit(hex[i + 1]));}
  auto require_size = [&bytes](std::size_t size) {
    if (bytes.size() != size) {throw std::invalid_argument("Wrong serial scalar size");}
  };
  switch (p.type) {
    case ValueType::Bool:
      require_size(1); if (bytes[0] > 1) {throw std::invalid_argument("Invalid boolean");}
      p.value = bytes[0] != 0; break;
    case ValueType::Int32: {
      require_size(4); auto bits = static_cast<std::uint32_t>(read_le(bytes)); std::int32_t value;
      std::memcpy(&value, &bits, 4); p.value = std::int64_t(value); break;
    }
    case ValueType::Int64: {
      require_size(8); auto bits = read_le(bytes); std::int64_t value;
      std::memcpy(&value, &bits, 8); p.value = value; break;
    }
    case ValueType::UInt32: case ValueType::UInt64:
      require_size(p.type == ValueType::UInt32 ? 4 : 8); p.value = read_le(bytes); break;
    case ValueType::Float32: {
      require_size(4); auto bits = static_cast<std::uint32_t>(read_le(bytes)); float value;
      std::memcpy(&value, &bits, 4); p.value = double(value); break;
    }
    case ValueType::Float64: {
      require_size(8); auto bits = read_le(bytes); double value;
      std::memcpy(&value, &bits, 8); p.value = value; break;
    }
    case ValueType::String: case ValueType::Json: p.value = std::string(bytes.begin(), bytes.end()); break;
    case ValueType::Bytes: p.value = bytes; break;
  }
  validate_payload(p); return p;
}
}  // namespace ros2_embedded
