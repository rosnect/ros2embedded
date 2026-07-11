# ROS2 Embedded Bridge Architecture

## Goal

Build a communication-agnostic bridge between embedded hardware and the ROS 2 ecosystem. Devices can connect over serial, Redis, MQTT, or future transports while ROS 2 nodes see normal publishers, subscribers, services, and actions.

The first target devices are Arduino-style ESP32 and ESP8266 boards, but the architecture should also support non-embedded producers later.

## High-Level Shape

```text
Manager
  |
  +-- launches/configures bridge node for connection A
  +-- launches/configures bridge node for connection B
  +-- tracks lifecycle, status, restart policy, and discovery

Bridge Node
  |
  +-- ConnectionHandler interface
  |     +-- SerialConnectionHandler
  |     +-- RedisConnectionHandler
  |     +-- MqttConnectionHandler
  |
  +-- receives protobuf interface description
  +-- creates ROS 2 publishers/subscribers dynamically
  +-- translates device messages <-> ROS 2 messages

Embedded Client Library
  |
  +-- write_data(identifier, value)
  +-- read_data(identifier)
  +-- protocol-specific handler implementation
```

## Monorepo Layout

Proposed package layout:

```text
ros2embedded/
  docs/
    architecture.md
  proto/
    bridge_config.proto
    bridge_payload.proto
  ros2/
    bridge_manager/
    bridge_node/
    connection_handlers/
  embedded/
    arduino/
      src/
      examples/
  tools/
```

## Core Concepts

### Manager Level

The manager owns process and lifecycle control. It should:

- Accept connection requests with protocol-specific parameters.
- Launch a bridge node with those parameters.
- Track active bridge nodes by connection id, protocol, health, and lifecycle state.
- Stop, restart, and reconfigure bridge nodes.
- Expose status over ROS 2 services/actions or a management topic.

Example connection request:

```yaml
connection_id: motor_controller_left
protocol: serial
params:
  port: /dev/ttyUSB0
  baudrate: 115200
config_source:
  type: device_bootstrap
```

### Bridge Node

Each bridge node handles one external connection. It receives a protobuf configuration that describes:

- External identifiers used by the hardware or external system.
- ROS 2 topics to publish.
- ROS 2 topics to subscribe to.
- Message encoding and value type.
- Direction: device-to-ROS, ROS-to-device, or bidirectional.

The bridge node should dynamically create ROS 2 publishers and subscribers from this configuration. That keeps the transport protocol independent from ROS 2 graph wiring.

### ConnectionHandler Interface

The bridge node should depend on a stable transport interface instead of protocol-specific code.

Expected responsibilities:

- `connect(params)`
- `disconnect()`
- `is_connected()`
- `send(identifier, payload)`
- `poll()` or async receive callback
- `set_receive_callback(callback)`

Initial implementations:

- `SerialConnectionHandler`
- `RedisConnectionHandler`
- `MqttConnectionHandler`

For efficiency and clean extensibility, protocol handlers can be built as shared libraries/plugins. In C++ ROS 2 this likely maps well to `pluginlib`, where each handler is compiled as a `.so` and loaded by protocol name.

### Protobuf Contracts

Use protobuf for two layers:

- Bridge configuration: declares identifiers, ROS topics, directions, and value types.
- Runtime payloads: carries identifier, timestamp, type information, and encoded value.
- Every config and payload includes a bridge `domain_id`. The bridge node and device client must agree on it before traffic is accepted.

The bridge configuration should be validated before any publishers/subscribers are created. Invalid identifiers, unsupported types, missing topics, or protocol mismatch should fail early.

### Embedded Client Library

The embedded side should feel like a small Blynk-style API:

```cpp
bridge.write_data("imu.accel_x", accel_x);
bridge.write_data("motor.left.rpm", rpm);

auto target = bridge.read_data<float>("motor.left.target_rpm");
```

The embedded library should:

- Expose connection-agnostic read/write APIs.
- Use explicit protocol handlers for serial, Redis, and MQTT where supported.
- Keep configuration values visible and strongly typed.
- Avoid hidden defaults that can cause hard-to-debug transport mismatches.
- Support ESP32 and ESP8266 first.

## First Implementation Milestones

1. Define protobuf schemas for bridge configuration and runtime payloads.
2. Create the C++ `ConnectionHandler` interface.
3. Implement serial, Redis, and MQTT handler skeletons.
4. Build a ROS 2 bridge node that loads one handler and creates publishers/subscribers from config.
5. Build a manager node that launches and tracks bridge nodes.
6. Add an Arduino client library with serial first, then MQTT and Redis as optional transports.
7. Add examples for ESP32 sensor publishing and ROS 2 command subscription.

## Design Notes

- Prefer one bridge node per physical or logical connection.
- Prefer handler plugins for protocol-specific performance and dependency isolation.
- Keep protobuf schemas stable and versioned.
- Treat identifiers as the cross-transport device API.
- Keep ROS 2 topic names configurable, not hard-coded into firmware.
- Validate bridge configs before starting runtime IO.
