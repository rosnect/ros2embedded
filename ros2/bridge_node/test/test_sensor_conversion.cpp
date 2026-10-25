#include "ros2_embedded/sensor_conversion.hpp"
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <limits>

using namespace ros2_embedded;
using Json = nlohmann::json;
TransportPayload payload(const Json & value, std::uint64_t timestamp = 1700000000123456789ULL)
{
  return {"sensor.value", 7, ValueType::Json, value.dump(), timestamp};
}
TEST(SensorConversion, AppAccelerometerAndGyroscopeHaveHonestUnknownFields)
{
  const Json stamp = {{"stamp", {{"sec", 1700000000}, {"nanosec", 123000000}}}, {"frame_id", "phone_link"}};
  const auto accel = sensor_from_payload<sensor_msgs::msg::Imu>(payload({
    {"header", stamp}, {"orientation", {{"x", 0}, {"y", 0}, {"z", 0}, {"w", 1}}},
    {"orientation_covariance", {-1, 0, 0, 0, 0, 0, 0, 0, 0}},
    {"linear_acceleration", {{"x", 1.25}, {"y", -2}, {"z", 9.81}}},
    {"angular_velocity_covariance", {-1, 0, 0, 0, 0, 0, 0, 0, 0}}}));
  EXPECT_DOUBLE_EQ(accel.linear_acceleration.z, 9.81);
  EXPECT_DOUBLE_EQ(accel.linear_acceleration.x, 1.25);
  EXPECT_DOUBLE_EQ(accel.angular_velocity_covariance[0], -1);
  EXPECT_DOUBLE_EQ(accel.orientation_covariance[0], -1);
  EXPECT_DOUBLE_EQ(accel.linear_acceleration_covariance[0], 0);
  EXPECT_EQ(accel.header.stamp.sec, 1700000000); EXPECT_EQ(accel.header.stamp.nanosec, 123000000u);
  EXPECT_EQ(accel.header.frame_id, "phone_link");
  auto roundtrip = sensor_from_payload<sensor_msgs::msg::Imu>(payload(Json::parse(sensor_to_json(accel))));
  EXPECT_EQ(roundtrip, accel);
  const auto gyro = sensor_from_payload<sensor_msgs::msg::Imu>(payload({{"angular_velocity", {{"x", 0.1}, {"y", 0.2}, {"z", -0.3}}}}));
  EXPECT_DOUBLE_EQ(gyro.angular_velocity.z, -0.3);
  EXPECT_DOUBLE_EQ(gyro.linear_acceleration_covariance[0], -1);
  EXPECT_DOUBLE_EQ(gyro.orientation_covariance[0], -1);
  EXPECT_DOUBLE_EQ(gyro.orientation.w, 1);
}
TEST(SensorConversion, TimestampsUseExplicitHeaderThenEnvelopeAndNeverHostTime)
{
  auto p = payload({{"illuminance", 20}});
  auto m = sensor_from_payload<sensor_msgs::msg::Illuminance>(p);
  EXPECT_EQ(m.header.stamp.sec, 1700000000); EXPECT_EQ(m.header.stamp.nanosec, 123456789u);
  p.timestamp_unix_ns = 0;
  m = sensor_from_payload<sensor_msgs::msg::Illuminance>(p);
  EXPECT_EQ(m.header.stamp.sec, 0); EXPECT_EQ(m.header.stamp.nanosec, 0u);
  m = sensor_from_payload<sensor_msgs::msg::Illuminance>(payload({{"illuminance", 20}, {"header", {{"stamp", {{"sec", 0}, {"nanosec", 0}}}}}}));
  EXPECT_EQ(m.header.stamp.sec, 0);
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::Illuminance>(payload({{"illuminance", 20}}, UINT64_MAX)));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::Illuminance>(payload({{"illuminance", 20}, {"header", {{"stamp", {{"sec", 1}, {"nanosec", 1000000000}}}}}})));
}
TEST(SensorConversion, PhysicalUnitsArePreserved)
{
  auto magnetic = sensor_from_payload<sensor_msgs::msg::MagneticField>(payload({{"magnetic_field", {{"x", 45e-6}, {"y", -12e-6}, {"z", nullptr}}}}));
  EXPECT_DOUBLE_EQ(magnetic.magnetic_field.x, 45e-6); EXPECT_TRUE(std::isnan(magnetic.magnetic_field.z));
  auto magnetic_roundtrip = sensor_from_payload<sensor_msgs::msg::MagneticField>(payload(Json::parse(sensor_to_json(magnetic))));
  EXPECT_DOUBLE_EQ(magnetic_roundtrip.magnetic_field.y, -12e-6);
  EXPECT_TRUE(std::isnan(magnetic_roundtrip.magnetic_field.z));
  auto light = sensor_from_payload<sensor_msgs::msg::Illuminance>(payload({{"illuminance", 123.5}, {"variance", 0}}));
  EXPECT_DOUBLE_EQ(light.illuminance, 123.5);
  EXPECT_EQ(sensor_from_payload<sensor_msgs::msg::Illuminance>(payload(Json::parse(sensor_to_json(light)))), light);
  auto pressure = sensor_from_payload<sensor_msgs::msg::FluidPressure>(payload({{"fluid_pressure", 101325}, {"variance", 2.5}}));
  EXPECT_DOUBLE_EQ(pressure.fluid_pressure, 101325);
  EXPECT_EQ(sensor_from_payload<sensor_msgs::msg::FluidPressure>(payload(Json::parse(sensor_to_json(pressure)))), pressure);
}
TEST(SensorConversion, LocationDoesNotInventAnAltitudeOrFix)
{
  auto fix = sensor_from_payload<sensor_msgs::msg::NavSatFix>(payload({{"latitude", 52.5}, {"longitude", 13.4},
    {"status", {{"status", 0}, {"service", 1}}}, {"position_covariance", {4, 0, 0, 0, 4, 0, 0, 0, 0}}, {"position_covariance_type", 0}}));
  EXPECT_DOUBLE_EQ(fix.latitude, 52.5); EXPECT_TRUE(std::isnan(fix.altitude)); EXPECT_EQ(fix.status.service, 1u);
  auto roundtrip = sensor_from_payload<sensor_msgs::msg::NavSatFix>(payload(Json::parse(sensor_to_json(fix))));
  EXPECT_TRUE(std::isnan(roundtrip.altitude)); EXPECT_EQ(roundtrip.position_covariance_type, 0u);
  auto nofix = sensor_from_payload<sensor_msgs::msg::NavSatFix>(payload({{"latitude", nullptr}, {"longitude", nullptr}}));
  EXPECT_EQ(nofix.status.status, -1); EXPECT_TRUE(std::isnan(nofix.latitude));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::NavSatFix>(payload({{"latitude", nullptr}, {"longitude", 0}, {"status", {{"status", 0}}}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::NavSatFix>(payload({{"latitude", 91}, {"longitude", 0}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::NavSatFix>(payload({{"latitude", 0}, {"longitude", -181}})));
}
TEST(SensorConversion, BatteryMissingValuesAreNanAndPercentageIsAFraction)
{
  const auto battery = sensor_from_payload<sensor_msgs::msg::BatteryState>(payload({{"voltage", 3.9}, {"temperature", 25.5},
    {"percentage", 0.75}, {"present", true}, {"power_supply_status", 2}}));
  EXPECT_FLOAT_EQ(battery.voltage, 3.9f); EXPECT_FLOAT_EQ(battery.percentage, 0.75f);
  EXPECT_FLOAT_EQ(battery.temperature, 25.5f); EXPECT_TRUE(battery.present);
  EXPECT_TRUE(std::isnan(battery.current)); EXPECT_TRUE(std::isnan(battery.charge)); EXPECT_TRUE(std::isnan(battery.capacity));
  auto json = Json::parse(sensor_to_json(battery));
  EXPECT_TRUE(json["current"].is_null());
  auto roundtrip = sensor_from_payload<sensor_msgs::msg::BatteryState>(payload(json));
  EXPECT_TRUE(std::isnan(roundtrip.current)); EXPECT_FLOAT_EQ(roundtrip.percentage, 0.75f);
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::BatteryState>(payload({{"voltage", 3.9}, {"percentage", 75}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::BatteryState>(payload({{"voltage", 3.9}, {"percentage", 1.00000001}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::BatteryState>(payload({{"voltage", 3.9}, {"power_supply_status", 255}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::BatteryState>(payload({{"voltage", nullptr}})));
}
TEST(SensorConversion, CompressedImagesDecodeBase64AndRemainBounded)
{
  for (auto size : {1u, 2u, 3u, 4u, 10000u, static_cast<unsigned>(kMaxCompressedImageBytes)}) {
    sensor_msgs::msg::CompressedImage image;
    image.format = "jpeg"; image.data.resize(size);
    for (std::size_t i = 0; i < size; ++i) {image.data[i] = static_cast<std::uint8_t>(i % 256);}
    const auto json = sensor_to_json(image);
    auto original = payload(Json::parse(json));
    // Real JSON and protobuf envelopes preserve the structured large image value.
    auto decoded = sensor_from_payload<sensor_msgs::msg::CompressedImage>(decode_json(encode_json(original)));
    EXPECT_EQ(decoded.data, image.data); EXPECT_EQ(decoded.format, "jpeg");
    EXPECT_EQ(sensor_from_payload<sensor_msgs::msg::CompressedImage>(decode_protobuf(encode_protobuf(original))).data, image.data);
  }
  const auto raw = sensor_from_payload<sensor_msgs::msg::CompressedImage>(payload({{"format", "png"}, {"data", {0, 255, 42}}}));
  EXPECT_EQ(raw.data, (Bytes{0, 255, 42}));
  for (const auto & bad : {"", "A", "A===", "AA=A", "AA==AAAA", "AB==", "AAB=", "AA!A", "AA\nA"}) {
    EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::CompressedImage>(payload({{"format", "jpeg"}, {"data_base64", bad}}))) << bad;
  }
  sensor_msgs::msg::CompressedImage too_big; too_big.format = "jpeg"; too_big.data.resize(kMaxCompressedImageBytes + 1);
  EXPECT_ANY_THROW(sensor_to_json(too_big));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::CompressedImage>(payload({{"format", "jpeg"}, {"data", {-1}}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::CompressedImage>(payload({{"format", "jpeg"}, {"data", {256}}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::CompressedImage>(payload({{"format", "jpeg"}, {"data", {1}}, {"data_base64", "AA=="}})));
}
TEST(SensorConversion, RejectsWrongSchemasRatherThanCoercingOrDroppingFields)
{
  for (const auto & value : {Json::parse("{\"linear_acceleration\":{\"x\":\"1\",\"y\":0,\"z\":0}}"),
    Json::parse("{\"linear_acceleration\":{\"x\":1,\"y\":0}}"),
    Json::parse("{\"linear_acceleration\":{\"x\":1,\"y\":0,\"z\":0},\"linear_acceleration_covariance\":[0]}"),
    Json::parse("{\"angular_velocity_covariance\":[0,0,0,0,0,0,0,0,0],\"linear_acceleration\":{\"x\":1,\"y\":0,\"z\":0}}"),
    Json::parse("{\"orientation\":{\"x\":0,\"y\":0,\"z\":0,\"w\":0}}"),
    Json::parse("{\"unknown\":1}"), Json::array()}) {
    EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::Imu>(payload(value)));
  }
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::Illuminance>(payload({{"illuminance", true}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::Illuminance>(payload({{"illuminance", -1}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::FluidPressure>(payload({{"fluid_pressure", 100000}, {"variance", -1}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::MagneticField>(payload({{"magnetic_field", {{"x", 1}, {"y", 2}, {"z", 3}}},
    {"magnetic_field_covariance", {-1, 0, 0, 0, 0, 0, 0, 0, 0}}})));
  EXPECT_ANY_THROW(sensor_from_payload<sensor_msgs::msg::Illuminance>(TransportPayload{"x", 7, ValueType::String, std::string("{}"), 0}));
  sensor_msgs::msg::Illuminance bad; bad.illuminance = std::numeric_limits<double>::infinity();
  EXPECT_ANY_THROW(sensor_to_json(bad));
  EXPECT_TRUE(supported_sensor_type("sensor_msgs/msg/Imu"));
  EXPECT_FALSE(supported_sensor_type("sensor_msgs/msg/PointCloud2"));
}
