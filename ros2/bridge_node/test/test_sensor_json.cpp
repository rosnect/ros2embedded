#include "bridge_node/sensor_json.hpp"
#include <gtest/gtest.h>
using namespace bridge_sensor_json;
TEST(SensorJson, ImageBytesAndHeader) {
  const auto image = decode<sensor_msgs::msg::CompressedImage>(
    R"({"header":{"stamp":{"sec":123,"nanosec":456},"frame_id":"camera_optical"},"format":"jpeg","data_base64":"/9gAf//Z"})", 0);
  EXPECT_EQ(image.header.stamp.sec, 123);
  EXPECT_EQ(image.header.stamp.nanosec, 456u);
  EXPECT_EQ(image.data, (std::vector<std::uint8_t>{255,216,0,127,255,217}));
}
TEST(SensorJson, RejectsMalformedAndOversizedImages) {
  EXPECT_ANY_THROW(decode_base64("AB=C"));
  EXPECT_ANY_THROW(decode_base64("!!!!"));
  EXPECT_ANY_THROW(decode_base64("AB=="));
  EXPECT_ANY_THROW(decode_base64(std::string(349532, 'A')));
}
TEST(SensorJson, ImuUnknownComponents) {
  const auto imu = decode<sensor_msgs::msg::Imu>(R"({"linear_acceleration":{"x":0,"y":0,"z":9.81}})", 123000000456);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 9.81);
  EXPECT_DOUBLE_EQ(imu.orientation_covariance[0], -1);
  EXPECT_DOUBLE_EQ(imu.angular_velocity_covariance[0], -1);
  EXPECT_EQ(imu.header.stamp.sec, 123);
}
TEST(SensorJson, RejectsInvalidCoordinatesAndCovariances) {
  EXPECT_ANY_THROW(decode<sensor_msgs::msg::NavSatFix>(R"({"latitude":91,"longitude":0})", 0));
  EXPECT_ANY_THROW(decode<sensor_msgs::msg::Imu>(R"({"orientation_covariance":[1,2]})", 0));
  EXPECT_ANY_THROW(decode<sensor_msgs::msg::FluidPressure>(R"({"fluid_pressure":-1})", 0));
}
TEST(SensorJson, MissingMeasurementsStayUnknown) {
  auto gps = decode<sensor_msgs::msg::NavSatFix>(R"({"latitude":52,"longitude":13})", 0);
  EXPECT_TRUE(std::isnan(gps.altitude));
  auto battery = decode<sensor_msgs::msg::BatteryState>(R"({"percentage":0.75})", 0);
  EXPECT_FLOAT_EQ(battery.percentage, 0.75f);
  EXPECT_TRUE(std::isnan(battery.current));
}
