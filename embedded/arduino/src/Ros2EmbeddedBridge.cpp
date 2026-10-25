#include "Ros2EmbeddedBridge.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

namespace {
bool validIdentifier(const char * text) {
  if (!text) return false;
  size_t count = 0;
  while (*text) {
    const unsigned char c = *text++;
    if (++count > 96 || !((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-' || c == '/')) return false;
  }
  return count != 0;
}
bool decimal(const char * text, uint32_t & value) {
  value = 0;
  size_t digits = 0;
  if (!*text) return false;
  while (*text) {
    const char c = *text++;
    if (c < '0' || c > '9' || ++digits > 10) return false;
    const uint32_t digit = static_cast<uint32_t>(c - '0');
    if (value > (UINT32_MAX - digit) / 10) return false;
    value = value * 10 + digit;
  }
  return true;
}
int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
bool validSize(BridgeValueType type, size_t size) {
  switch (type) {
    case BridgeValueType::Bool: return size == 1;
    case BridgeValueType::Int32: case BridgeValueType::UInt32: case BridgeValueType::Float32: return size == 4;
    case BridgeValueType::Int64: case BridgeValueType::UInt64: case BridgeValueType::Float64: return size == 8;
    case BridgeValueType::String: case BridgeValueType::Bytes: case BridgeValueType::Json: return size <= 256;
    default: return false;
  }
}
void encodeLE(uint64_t value, uint8_t * bytes, size_t count) {
  for (size_t i = 0; i < count; ++i) bytes[i] = static_cast<uint8_t>(value >> (i * 8));
}
}  // namespace

SerialBridgeConnectionHandler::SerialBridgeConnectionHandler(HardwareSerial & serial) : serial_(serial) {}
bool SerialBridgeConnectionHandler::begin(uint32_t domain_id) {
  domain_id_ = domain_id;
  frame_size_ = 0; discard_ = false; started_ = true;
  return true;  // Caller configures HardwareSerial.begin(baudrate) first.
}
bool SerialBridgeConnectionHandler::connected() const {return started_;}
bool SerialBridgeConnectionHandler::send(const BridgePayload & payload) {
  if (!started_ || payload.domain_id != domain_id_ || !validIdentifier(payload.identifier) ||
      (!payload.data && payload.size != 0) || !validSize(payload.type, payload.size)) return false;
  if (payload.type == BridgeValueType::Bool && payload.data[0] > 1) return false;
  char frame[640];
  int header = snprintf(frame, sizeof(frame), "R2E,%lu,%u,%s,",
    static_cast<unsigned long>(payload.domain_id), static_cast<unsigned>(payload.type), payload.identifier);
  if (header < 0 || static_cast<size_t>(header) + payload.size * 2 + 1 > sizeof(frame)) return false;
  size_t size = static_cast<size_t>(header);
  const char * hex = "0123456789ABCDEF";
  for (size_t i = 0; i < payload.size; ++i) {
    frame[size++] = hex[payload.data[i] >> 4]; frame[size++] = hex[payload.data[i] & 15];
  }
  frame[size++] = '\n';
  return serial_.write(reinterpret_cast<const uint8_t *>(frame), size) == size;
}
bool SerialBridgeConnectionHandler::parseFrame(BridgePayload & payload) {
  if (frame_size_ && frame_[frame_size_ - 1] == '\r') --frame_size_;
  frame_[frame_size_] = 0;
  char * fields[5] = {frame_, nullptr, nullptr, nullptr, nullptr};
  size_t field = 1;
  for (size_t i = 0; i < frame_size_; ++i) {
    if (frame_[i] == ',') {
      if (field == 5) return false;
      frame_[i] = 0; fields[field++] = frame_ + i + 1;
    } else if (frame_[i] == 0) return false;
  }
  if (field != 5 || strcmp(fields[0], "R2E") != 0 || !validIdentifier(fields[3])) return false;
  uint32_t domain, type;
  if (!decimal(fields[1], domain) || domain != domain_id_ || !decimal(fields[2], type) || type < 1 || type > 10) return false;
  const size_t hex_size = strlen(fields[4]);
  if (hex_size % 2 || hex_size > 512 || !validSize(static_cast<BridgeValueType>(type), hex_size / 2)) return false;
  for (size_t i = 0; i < hex_size; i += 2) {
    int high = hexDigit(fields[4][i]), low = hexDigit(fields[4][i + 1]);
    if (high < 0 || low < 0) return false;
    data_[i / 2] = static_cast<uint8_t>((high << 4) | low);
  }
  const size_t size = hex_size / 2;
  if (type == 1 && data_[0] > 1) return false;
  if (type == 6 || type == 7) {
    uint64_t bits = 0;
    for (size_t i = 0; i < size; ++i) bits |= uint64_t(data_[i]) << (i * 8);
    if (type == 6) {
      const uint32_t short_bits = static_cast<uint32_t>(bits);
      float value; memcpy(&value, &short_bits, 4); if (!isfinite(value)) return false;
    } else {
      double value; memcpy(&value, &bits, 8); if (!isfinite(value)) return false;
    }
  }
  data_[size] = 0;  // Convenience terminator for text; payload.size remains authoritative.
  strcpy(identifier_, fields[3]);
  payload.identifier = identifier_; payload.domain_id = domain;
  payload.type = static_cast<BridgeValueType>(type); payload.data = data_; payload.size = size;
  return true;
}
bool SerialBridgeConnectionHandler::receive(BridgePayload & payload) {
  if (!started_) return false;
  // Bounded work even if a noisy peer continuously feeds the UART.
  for (size_t budget = 0; budget < 1024 && serial_.available() > 0; ++budget) {
    int next = serial_.read();
    if (next < 0) break;
    const char c = static_cast<char>(next);
    if (c == '\n') {
      bool valid = !discard_ && parseFrame(payload);
      frame_size_ = 0; discard_ = false;
      if (valid) return true;
    } else if (!discard_) {
      if (frame_size_ >= sizeof(frame_) - 1) {discard_ = true; frame_size_ = 0;}
      else frame_[frame_size_++] = c;
    }
  }
  return false;
}
Ros2EmbeddedBridge::Ros2EmbeddedBridge(BridgeConnectionHandler & handler, uint32_t domain_id)
: handler_(handler), domain_id_(domain_id) {}
bool Ros2EmbeddedBridge::begin() {return handler_.begin(domain_id_);}
bool Ros2EmbeddedBridge::connected() const {return handler_.connected();}
uint32_t Ros2EmbeddedBridge::domainId() const {return domain_id_;}
bool Ros2EmbeddedBridge::writeData(const char * identifier, bool value) {
  const uint8_t encoded = value ? 1 : 0;
  return writeRaw(identifier, BridgeValueType::Bool, &encoded, 1);
}
bool Ros2EmbeddedBridge::writeData(const char * identifier, int32_t value) {
  uint8_t bytes[4]; encodeLE(static_cast<uint32_t>(value), bytes, 4);
  return writeRaw(identifier, BridgeValueType::Int32, bytes, 4);
}
bool Ros2EmbeddedBridge::writeData(const char * identifier, uint32_t value) {
  uint8_t bytes[4]; encodeLE(value, bytes, 4);
  return writeRaw(identifier, BridgeValueType::UInt32, bytes, 4);
}
bool Ros2EmbeddedBridge::writeData(const char * identifier, int64_t value) {
  uint8_t bytes[8]; encodeLE(static_cast<uint64_t>(value), bytes, 8);
  return writeRaw(identifier, BridgeValueType::Int64, bytes, 8);
}
bool Ros2EmbeddedBridge::writeData(const char * identifier, uint64_t value) {
  uint8_t bytes[8]; encodeLE(value, bytes, 8);
  return writeRaw(identifier, BridgeValueType::UInt64, bytes, 8);
}
bool Ros2EmbeddedBridge::writeData(const char * identifier, float value) {
  if (!isfinite(value)) return false;
  static_assert(sizeof(float) == 4, "float32 required");
  uint32_t bits; memcpy(&bits, &value, 4); uint8_t bytes[4]; encodeLE(bits, bytes, 4);
  return writeRaw(identifier, BridgeValueType::Float32, bytes, 4);
}
bool Ros2EmbeddedBridge::writeData(const char * identifier, double value) {
  if (!isfinite(value)) return false;
  static_assert(sizeof(double) == 8, "float64 required (ESP32/ESP8266)");
  uint64_t bits; memcpy(&bits, &value, 8); uint8_t bytes[8]; encodeLE(bits, bytes, 8);
  return writeRaw(identifier, BridgeValueType::Float64, bytes, 8);
}
bool Ros2EmbeddedBridge::writeData(const char * identifier, const char * value) {
  if (!value) return false;
  const size_t size = strnlen(value, 257);
  return writeRaw(identifier, BridgeValueType::String, reinterpret_cast<const uint8_t *>(value), size);
}
bool Ros2EmbeddedBridge::writeBytes(const char * identifier, const uint8_t * data, size_t size) {
  return writeRaw(identifier, BridgeValueType::Bytes, data, size);
}
bool Ros2EmbeddedBridge::readData(BridgePayload & payload) {
  return handler_.receive(payload) && payload.domain_id == domain_id_;
}
bool Ros2EmbeddedBridge::writeRaw(const char * identifier, BridgeValueType type, const uint8_t * data, size_t size) {
  if (!connected() || !validIdentifier(identifier) || (!data && size) || !validSize(type, size)) return false;
  BridgePayload payload{identifier, domain_id_, type, data, size};
  return handler_.send(payload);
}
