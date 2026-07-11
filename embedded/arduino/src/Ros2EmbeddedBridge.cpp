#include "Ros2EmbeddedBridge.h"

#include <string.h>

SerialBridgeConnectionHandler::SerialBridgeConnectionHandler(HardwareSerial & serial)
: serial_(serial)
{
}

bool SerialBridgeConnectionHandler::begin(uint32_t domain_id)
{
  domain_id_ = domain_id;
  return true;
}

bool SerialBridgeConnectionHandler::connected() const
{
  return true;
}

bool SerialBridgeConnectionHandler::send(const BridgePayload & payload)
{
  if (payload.domain_id != domain_id_ || payload.identifier == nullptr || payload.data == nullptr) {
    return false;
  }

  serial_.print("R2E,");
  serial_.print(payload.domain_id);
  serial_.print(",");
  serial_.print(static_cast<uint8_t>(payload.type));
  serial_.print(",");
  serial_.print(payload.identifier);
  serial_.print(",");
  for (size_t i = 0; i < payload.size; ++i) {
    if (payload.data[i] < 16) {
      serial_.print("0");
    }
    serial_.print(payload.data[i], HEX);
  }
  serial_.println();
  return true;
}

bool SerialBridgeConnectionHandler::receive(BridgePayload & payload)
{
  (void)payload;
  return false;
}

Ros2EmbeddedBridge::Ros2EmbeddedBridge(BridgeConnectionHandler & handler, uint32_t domain_id)
: handler_(handler), domain_id_(domain_id)
{
}

bool Ros2EmbeddedBridge::begin()
{
  return handler_.begin(domain_id_);
}

bool Ros2EmbeddedBridge::connected() const
{
  return handler_.connected();
}

uint32_t Ros2EmbeddedBridge::domainId() const
{
  return domain_id_;
}

bool Ros2EmbeddedBridge::writeData(const char * identifier, bool value)
{
  const uint8_t encoded = value ? 1 : 0;
  return writeRaw(identifier, BridgeValueType::Bool, &encoded, sizeof(encoded));
}

bool Ros2EmbeddedBridge::writeData(const char * identifier, int32_t value)
{
  return writeRaw(
    identifier,
    BridgeValueType::Int32,
    reinterpret_cast<const uint8_t *>(&value),
    sizeof(value));
}

bool Ros2EmbeddedBridge::writeData(const char * identifier, uint32_t value)
{
  return writeRaw(
    identifier,
    BridgeValueType::UInt32,
    reinterpret_cast<const uint8_t *>(&value),
    sizeof(value));
}

bool Ros2EmbeddedBridge::writeData(const char * identifier, float value)
{
  return writeRaw(
    identifier,
    BridgeValueType::Float32,
    reinterpret_cast<const uint8_t *>(&value),
    sizeof(value));
}

bool Ros2EmbeddedBridge::writeData(const char * identifier, const char * value)
{
  if (value == nullptr) {
    return false;
  }
  return writeRaw(
    identifier,
    BridgeValueType::String,
    reinterpret_cast<const uint8_t *>(value),
    strlen(value));
}

bool Ros2EmbeddedBridge::writeBytes(const char * identifier, const uint8_t * data, size_t size)
{
  return writeRaw(identifier, BridgeValueType::Bytes, data, size);
}

bool Ros2EmbeddedBridge::readData(BridgePayload & payload)
{
  if (!handler_.receive(payload)) {
    return false;
  }
  return payload.domain_id == domain_id_;
}

bool Ros2EmbeddedBridge::writeRaw(
  const char * identifier,
  BridgeValueType type,
  const uint8_t * data,
  size_t size)
{
  if (!connected() || identifier == nullptr || data == nullptr || size == 0) {
    return false;
  }

  BridgePayload payload;
  payload.identifier = identifier;
  payload.domain_id = domain_id_;
  payload.type = type;
  payload.data = data;
  payload.size = size;
  return handler_.send(payload);
}
