#pragma once

#include <Arduino.h>

enum class BridgeValueType : uint8_t {
  Bool = 1,
  Int32 = 2,
  UInt32 = 3,
  Int64 = 4,
  UInt64 = 5,
  Float32 = 6,
  Float64 = 7,
  String = 8,
  Bytes = 9,
  Json = 10,
};

struct BridgePayload {
  const char * identifier;
  uint32_t domain_id;
  BridgeValueType type;
  const uint8_t * data;
  size_t size;
};

class BridgeConnectionHandler {
public:
  virtual ~BridgeConnectionHandler() = default;

  virtual bool begin(uint32_t domain_id) = 0;
  virtual bool connected() const = 0;
  virtual bool send(const BridgePayload & payload) = 0;
  virtual bool receive(BridgePayload & payload) = 0;
};

class SerialBridgeConnectionHandler : public BridgeConnectionHandler {
public:
  explicit SerialBridgeConnectionHandler(HardwareSerial & serial = Serial);

  bool begin(uint32_t domain_id) override;
  bool connected() const override;
  bool send(const BridgePayload & payload) override;
  bool receive(BridgePayload & payload) override;

private:
  HardwareSerial & serial_;
  bool parseFrame(BridgePayload & payload);
  uint32_t domain_id_{0};
  bool started_{false};
  bool discard_{false};
  size_t frame_size_{0};
  char frame_[640];
  char identifier_[97];
  uint8_t data_[257];
};

class Ros2EmbeddedBridge {
public:
  Ros2EmbeddedBridge(BridgeConnectionHandler & handler, uint32_t domain_id);

  bool begin();
  bool connected() const;
  uint32_t domainId() const;

  bool writeData(const char * identifier, bool value);
  bool writeData(const char * identifier, int32_t value);
  bool writeData(const char * identifier, uint32_t value);
  bool writeData(const char * identifier, int64_t value);
  bool writeData(const char * identifier, uint64_t value);
  bool writeData(const char * identifier, float value);
  bool writeData(const char * identifier, double value);
  bool writeData(const char * identifier, const char * value);
  bool writeBytes(const char * identifier, const uint8_t * data, size_t size);

  // Returned identifier/data belong to the handler and remain valid until
  // the next readData() or begin(). Copy them if they must be retained.
  bool readData(BridgePayload & payload);

private:
  bool writeRaw(
    const char * identifier,
    BridgeValueType type,
    const uint8_t * data,
    size_t size);

  BridgeConnectionHandler & handler_;
  uint32_t domain_id_;
};
