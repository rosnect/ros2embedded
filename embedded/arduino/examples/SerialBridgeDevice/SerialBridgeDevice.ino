#include <Ros2EmbeddedBridge.h>
#include <string.h>

static constexpr uint32_t kBridgeDomainId = 7;
SerialBridgeConnectionHandler serial_handler(Serial);
Ros2EmbeddedBridge bridge(serial_handler, kBridgeDomainId);
int32_t motor_target = 0;
unsigned long last_publish = 0;

void setup()
{
  Serial.begin(115200);
  bridge.begin();
}

void loop()
{
  BridgePayload command;
  if (bridge.readData(command) && strcmp(command.identifier, "motor.target") == 0 &&
      command.type == BridgeValueType::Int32 && command.size == 4) {
    // The wire uses little-endian integers; use memcpy rather than an unaligned pointer cast.
    uint32_t bits = uint32_t(command.data[0]) | (uint32_t(command.data[1]) << 8) |
      (uint32_t(command.data[2]) << 16) | (uint32_t(command.data[3]) << 24);
    memcpy(&motor_target, &bits, sizeof(motor_target));
    // Apply motor_target to your hardware here. Do not print debug text on this UART.
  }
  const unsigned long now = millis();
  if (now - last_publish >= 1000) {
    last_publish = now;
    bridge.writeData("sensor.temperature_c", 24.5f);
  }
}
