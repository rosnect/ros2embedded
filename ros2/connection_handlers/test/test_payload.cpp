#include "ros2_embedded/payload.hpp"
#include "ros2_embedded/serial_connection_handler.hpp"
#include <gtest/gtest.h>
#include <fcntl.h>
#include <pty.h>
#include <unistd.h>
#include <limits>
#include <thread>

using namespace ros2_embedded;
TEST(Payload, AllTypesRoundTripAcrossCodecs)
{
  const std::vector<std::pair<ValueType, Value>> values{
    {ValueType::Bool, true}, {ValueType::Int32, std::int64_t(INT32_MIN)},
    {ValueType::UInt32, std::uint64_t(UINT32_MAX)}, {ValueType::Int64, std::int64_t(INT64_MIN)},
    {ValueType::UInt64, std::uint64_t(UINT64_MAX)}, {ValueType::Float32, 24.5},
    {ValueType::Float64, 1.25}, {ValueType::String, std::string("hello,\nµ")},
    {ValueType::String, std::string()}, {ValueType::Bytes, Bytes{0, 255, 42}},
    {ValueType::Bytes, Bytes{}}, {ValueType::Json, std::string("{\"x\":1}")}};
  for (const auto & entry : values) {
    TransportPayload p{"sensor.value", 7, entry.first, entry.second, 1700000000000000000ULL};
    for (const auto & q : {decode_json(encode_json(p)), decode_protobuf(encode_protobuf(p)), decode_serial(encode_serial(p))}) {
      EXPECT_EQ(q.identifier, p.identifier); EXPECT_EQ(q.domain_id, p.domain_id);
      EXPECT_EQ(q.type, p.type); EXPECT_EQ(q.value, p.value);
    }
    EXPECT_EQ(decode_json(encode_json(p)).timestamp_unix_ns, p.timestamp_unix_ns);
    EXPECT_EQ(decode_protobuf(encode_protobuf(p)).timestamp_unix_ns, p.timestamp_unix_ns);
  }
}
TEST(Payload, RejectsMalformedTypesRangesAndLimits)
{
  for (const auto & wire : {
    R"({"identifier":"x","domain_id":4294967296,"value_type":"string","value":"x"})",
    R"({"identifier":"x","domain_id":-1,"value_type":"string","value":"x"})",
    R"({"identifier":"x","domain_id":7,"value_type":"int32","value":2147483648})",
    R"({"identifier":"x","domain_id":7,"value_type":"uint64","value":-1})",
    R"({"identifier":"x","domain_id":7,"value_type":"boolValue","value":1})",
    R"({"identifier":"x","domain_id":7,"value_type":"bytes","value":[256]})",
    R"({"identifier":"x","domain_id":7,"value_type":"bytes","value":[-1]})",
    R"({"identifier":"x","domain_id":7,"value_type":"int64","value":18446744073709551615})",
    R"({"identifier":"x","domain_id":7,"value_type":"float32","value":1e100})",
    R"({"identifier":"x","domain_id":7,"value_type":"json","value":null})",
    R"({"identifier":"x","domain_id":7,"value_type":"string","value":"x","timestamp_unix_ms":18446744073709551615})",
    "[]", "garbage"}) {EXPECT_ANY_THROW(decode_json(wire)) << wire;}
  for (const auto & wire : {"R2E,7,1,x,02", "R2E,7,2,x,01", "R2E,7,8,x,GG",
    "R2E,4294967296,8,x,", "R2E,7,256,x,", "R2E,7,8,,", "R2E,7,8,x,0"})
  {EXPECT_ANY_THROW(decode_serial(wire)) << wire;}
  EXPECT_ANY_THROW(decode_protobuf(""));
  EXPECT_ANY_THROW(decode_json(std::string(kMaxWireBytes + 1, 'x')));
  EXPECT_ANY_THROW(encode_json(TransportPayload{"x", 7, ValueType::Float64, std::numeric_limits<double>::infinity(), 0}));
  EXPECT_ANY_THROW(encode_serial(TransportPayload{"x", 7, ValueType::Bytes, Bytes(257), 0}));
  EXPECT_ANY_THROW(encode_json(TransportPayload{"bad,id", 7, ValueType::String, std::string("x"), 0}));
}
TEST(Serial, FragmentationDomainFilteringRecoveryAndCommands)
{
  int master, slave; char name[128];
  ASSERT_EQ(openpty(&master, &slave, name, nullptr, nullptr), 0);
  close(slave); fcntl(master, F_SETFL, O_NONBLOCK);
  SerialConnectionHandler handler;
  handler.configure(ConnectionParams{"test", 7, {{"serial_port", name}, {"baudrate", "115200"}, {"payload_encoding", "serial"}}});
  ASSERT_TRUE(handler.connect());
  std::vector<TransportPayload> received;
  handler.set_receive_callback([&](const auto & p) {received.push_back(p);});
  std::string first = "R2E,7,6,sensor.temperature_c,0000";
  ASSERT_EQ(write(master, first.data(), first.size()), static_cast<ssize_t>(first.size()));
  handler.poll(); EXPECT_TRUE(received.empty());
  const std::string rest = "C441\r\nR2E,8,8,x,61\ninvalid\n" + std::string(700, 'x') + "\nR2E,7,8,status,\n";
  ASSERT_EQ(write(master, rest.data(), rest.size()), static_cast<ssize_t>(rest.size()));
  for (int i = 0; i < 20 && received.size() < 2; ++i) {handler.poll(); std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  ASSERT_EQ(received.size(), 2u); EXPECT_DOUBLE_EQ(std::get<double>(received[0].value), 24.5);
  EXPECT_EQ(std::get<std::string>(received[1].value), "");
  EXPECT_FALSE(handler.send(TransportPayload{"motor.target", 8, ValueType::Int32, std::int64_t(42), 0}));
  ASSERT_TRUE(handler.send(TransportPayload{"motor.target", 7, ValueType::Int32, std::int64_t(-42), 0}));
  handler.poll();
  char buffer[1024]; ssize_t count = -1;
  for (int i = 0; i < 20 && count <= 0; ++i) {count = read(master, buffer, sizeof(buffer)); std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  ASSERT_GT(count, 0); EXPECT_EQ(std::string(buffer, count), "R2E,7,2,motor.target,D6FFFFFF\n");
  close(master); handler.poll(); EXPECT_FALSE(handler.is_connected());
}
