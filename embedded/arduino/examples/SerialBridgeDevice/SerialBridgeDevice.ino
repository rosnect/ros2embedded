#include <Ros2EmbeddedBridge.h>

static constexpr uint32_t kBridgeDomainId = 7;

SerialBridgeConnectionHandler serial_handler(Serial);
Ros2EmbeddedBridge bridge(serial_handler, kBridgeDomainId);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  bridge.begin();
}

void loop()
{
  const float fake_temperature_c = 24.5f;
  bridge.writeData("sensor.temperature_c", fake_temperature_c);
  delay(1000);
}
