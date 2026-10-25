#pragma once

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace ros2_embedded
{
// Numeric codes match proto/bridge_config.proto and the serial device API.
enum class ValueType : std::uint8_t {
  Bool = 1, Int32 = 2, UInt32 = 3, Int64 = 4, UInt64 = 5,
  Float32 = 6, Float64 = 7, String = 8, Bytes = 9, Json = 10
};
using Bytes = std::vector<std::uint8_t>;
using Value = std::variant<bool, std::int64_t, std::uint64_t, double, std::string, Bytes>;
struct TransportPayload
{
  std::string identifier;
  std::uint32_t domain_id{0};
  ValueType type{ValueType::String};
  Value value{std::string{}};
  // Zero means unknown. Device uptime must not be presented as Unix time.
  std::uint64_t timestamp_unix_ns{0};
};

constexpr std::size_t kMaxIdentifierBytes = 96;
constexpr std::size_t kMaxValueBytes = 4096;
constexpr std::size_t kMaxWireBytes = 65536;
constexpr std::size_t kMaxSerialValueBytes = 256;

bool valid_identifier(const std::string & identifier);
void validate_payload(const TransportPayload & payload);
std::string encode_json(const TransportPayload & payload);
TransportPayload decode_json(const std::string & wire);
std::string encode_protobuf(const TransportPayload & payload);
TransportPayload decode_protobuf(const std::string & wire);
std::string encode_serial(const TransportPayload & payload);
TransportPayload decode_serial(const std::string & line);
}  // namespace ros2_embedded
