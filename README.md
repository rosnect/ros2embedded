# ros2embedded

Communication-agnostic bridge between embedded devices and ROS 2.

This repository targets ROS 2 Humble on the ROS machine. The code can be edited on macOS without ROS 2 installed, then built and tested on a Linux device with Humble.

## Layout

```text
proto/                  Protobuf contracts for bridge config and payloads
ros2/connection_handlers C++ pluginlib transport handler package
ros2/bridge_node         C++ bridge node package
ros2/bridge_manager      Python manager package
embedded/arduino         Arduino ESP32/ESP8266 client library
docs/architecture.md     Reference architecture
```

## Domain ID

The bridge has its own `domain_id` field in protobuf payloads and client configuration. It is not a replacement for `ROS_DOMAIN_ID`; it is a device-bridge guard that makes the embedded client and launched bridge node agree before traffic is accepted.

Example host launch:

```bash
ros2 launch bridge_node bridge.launch.py \
  connection_id:=esp32_lab_1 \
  protocol:=serial \
  domain_id:=7 \
  serial_port:=/dev/ttyUSB0 \
  baudrate:=115200
```

Equivalent direct node run:

```bash
ros2 run bridge_node bridge_node --ros-args \
  -p connection_id:=esp32_lab_1 \
  -p protocol:=serial \
  -p domain_id:=7 \
  -p serial_port:=/dev/ttyUSB0 \
  -p baudrate:=115200
```

Example Arduino client:

```cpp
static constexpr uint32_t kBridgeDomainId = 7;
SerialBridgeConnectionHandler serial_handler(Serial);
Ros2EmbeddedBridge bridge(serial_handler, kBridgeDomainId);
```

## Build on ROS 2 Humble

From a Humble workspace:

```bash
mkdir -p ~/ros2embedded_ws/src
cd ~/ros2embedded_ws/src
git clone <this-repo-url> ros2embedded
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Current Status

This is the first implementation scaffold:

- Protobuf config and payload schemas exist.
- ROS 2 handler plugins exist for serial, MQTT, and Redis.
- Bridge node loads handlers by protocol and enforces bridge `domain_id`.
- Manager node can auto-start one bridge process and publish status.
- Arduino library exposes `writeData`, `writeBytes`, and `readData` with domain validation.

Next work should replace the placeholder transport internals with real serial framing, MQTT client integration, Redis pub/sub integration, and generated protobuf serialization.
