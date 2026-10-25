#include "Ros2EmbeddedBridge.h"
#include "ros2_embedded/payload.hpp"
#include <gtest/gtest.h>
#include <cstring>
HardwareSerial Serial;
TEST(Arduino, HostAndDeviceWireCompatibility)
{
  HardwareSerial serial;
  SerialBridgeConnectionHandler handler(serial);
  Ros2EmbeddedBridge device(handler, 7);
  EXPECT_FALSE(device.connected()); ASSERT_TRUE(device.begin());
  ASSERT_TRUE(device.writeData("sensor.temperature_c", 24.5f));
  auto p = ros2_embedded::decode_serial(serial.outgoing);
  EXPECT_EQ(p.type, ros2_embedded::ValueType::Float32); EXPECT_DOUBLE_EQ(std::get<double>(p.value), 24.5);
  serial.outgoing.clear(); ASSERT_TRUE(device.writeData("label", ""));
  EXPECT_EQ(std::get<std::string>(ros2_embedded::decode_serial(serial.outgoing).value), "");
  serial.outgoing.clear(); ASSERT_TRUE(device.writeBytes("blob", nullptr, 0));
  EXPECT_TRUE(std::get<ros2_embedded::Bytes>(ros2_embedded::decode_serial(serial.outgoing).value).empty());
  serial.outgoing.clear(); ASSERT_TRUE(device.writeData("integer", int64_t(INT64_MIN)));
  EXPECT_EQ(std::get<int64_t>(ros2_embedded::decode_serial(serial.outgoing).value), INT64_MIN);
  serial.outgoing.clear(); ASSERT_TRUE(device.writeData("integer", uint64_t(UINT64_MAX)));
  EXPECT_EQ(std::get<uint64_t>(ros2_embedded::decode_serial(serial.outgoing).value), UINT64_MAX);
  serial.outgoing.clear(); ASSERT_TRUE(device.writeData("double", 1.25));
  EXPECT_DOUBLE_EQ(std::get<double>(ros2_embedded::decode_serial(serial.outgoing).value), 1.25);
  serial.write_limit = 0; EXPECT_FALSE(device.writeData("label", "hello"));
  EXPECT_FALSE(device.writeData("bad,id", "hello"));
  serial.incoming = "bad\nR2E,8,2,motor.target,00000000\n" + std::string(700, 'x') + "\nR2E,7,2,motor.target,D6FF";
  BridgePayload command;
  EXPECT_FALSE(device.readData(command));
  serial.incoming += "FFFF\n";
  ASSERT_TRUE(device.readData(command)); EXPECT_STREQ(command.identifier, "motor.target");
  EXPECT_EQ(command.type, BridgeValueType::Int32); ASSERT_EQ(command.size, 4u);
  EXPECT_EQ(command.data[0], 0xd6); EXPECT_EQ(command.data[3], 0xff);
  serial.incoming = "R2E,7,8,label,\n";
  ASSERT_TRUE(device.readData(command)); EXPECT_EQ(command.size, 0u);
}
TEST(Arduino, RejectsMalformedFramesAndRecovers)
{
  HardwareSerial serial; SerialBridgeConnectionHandler handler(serial); handler.begin(7);
  serial.incoming = "R2E,7,1,x,02\nR2E,7,2,x,00\nR2E,7,8,x,GG\nR2E,4294967296,8,x,\nR2E,7,8,bad id,\nR2E,7,8,x,\n";
  BridgePayload p; ASSERT_TRUE(handler.receive(p)); EXPECT_STREQ(p.identifier, "x"); EXPECT_EQ(p.size, 0u);
}
