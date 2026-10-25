# ros2embedded

A ROS 2 Humble bridge for embedded devices and other external clients. The architecture remains **manager → one bridge node per connection → transport plugin**. MQTT and serial share typed payload validation and configurable ROS topic bindings.

The Flutter library and Rosnect app live in the separate sibling repository `../flutter_bridge_client`; this repository does not build or modify them.

## Implemented

- MQTT 3.1.1 through libmosquitto: JSON compatible with Rosnect, or protobuf for other devices; QoS 1, credentials, verified TLS, client certificates, and reconnect with 1–30 second backoff.
- Linux serial: nonblocking termios I/O, bounded queues, partial frames/writes, malformed-frame recovery, and reconnect after an established device disconnects.
- Arduino ESP32/ESP8266 library: bidirectional serial, typed writes, and command reads.
- Automatic MQTT device interface exchange (JSON/protobuf): live binding updates, acknowledgements, and retained-config restart bootstrap.
- Configured ROS publishers/subscribers for Bool, Int32, UInt32, Int64, UInt64, Float32, Float64, String, and ByteMultiArray. Structured JSON maps to String.
- Manager child cleanup, restart of exited connections, private temporary parameter files, and process status.

Redis, automatic manager discovery of new connection IDs, arbitrary ROS message definitions, services, and actions are not implemented. Phone sensor capture belongs to Rosnect; its current Sensors page sends sample data.

## Build

On Ubuntu 22.04 with ROS 2 Humble, install dependencies and build from this repository:

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install libmosquitto-dev nlohmann-json3-dev libprotobuf-dev protobuf-compiler
rosdep install --from-paths ros2 --ignore-src -r -y
colcon build --symlink-install --base-paths ros2
source install/setup.bash
```

Protobuf sources are generated during the build. Installed schemas are under `share/connection_handlers/proto`.

## Automatic MQTT interfaces / Rosnect

The device sends a complete `BridgeConfig` on its MQTT `config` topic. The bridge creates ROS publishers/subscribers automatically, acknowledges their creation, and applies future interface changes without restarting. This works for any MQTT client implementing the contract.

**The current Flutter app needs the small control-channel update described in [Flutter handoff](docs/device_configuration.md).** Its existing data messages remain compatible, but the checked-out app does not yet publish its interface description. This repository does not edit Flutter.

Use a broker reachable by the ROS machine and phone. From a built, sourced repository terminal, start the manager without a local binding file:

```bash
ros2 run bridge_manager bridge_manager --ros-args \
  -p auto_start:=true \
  -p connection_id:=phone_1 -p protocol:=mqtt -p domain_id:=7 \
  -p mqtt_broker:=192.168.0.81 -p mqtt_port:=1883 \
  -p config_source:=device
```

The phone publishes its snapshot to `ros2embedded/7/phone_1/config` (QoS 1, retained) and listens for `config_status` and `config_request` alongside the existing data topics. Until a valid snapshot arrives, the bridge waits without creating application topics. Enabling/disabling sensors or changing subscriptions should publish a new complete snapshot, with a new `configuration_id`; the bridge adds/removes the corresponding ROS interfaces on acceptance. A snapshot with zero bindings removes all application interfaces.

To simulate the configuration exchange before updating Flutter, run in another sourced repository terminal:

```bash
/usr/bin/python3 tools/publish_device_config.py \
  ros2/bridge_node/config/phone_device_config.json \
  --broker 192.168.0.81 --port 1883
```

This helper sends a demo description containing accelerometer, gyroscope, and robot status bindings and prints the bridge ACK. It is a device simulator, not a replacement for the Flutter update. After acceptance, configure Rosnect with the same broker, connection ID `phone_1`, and bridge domain `7`. Enable Accelerometer and select **Publish sample**, then inspect:

```bash
ros2 topic echo /ros2embedded/domain_7/connection_phone_5f1/accelerometer
```

In Rosnect's Subscribe tab add `robot.status`, then send a reply:

```bash
ros2 topic pub --once \
  /ros2embedded/domain_7/connection_phone_5f1/robot_status \
  std_msgs/msg/String '{data: ready}'
```

The host uses a distinct MQTT client ID so it can coexist with the phone. Broker connectivity does not prove device presence or interface readiness. `/ros2embedded/domain_7/connection_phone_5f1/status` reports broker/subscription state; `/ros2embedded/domain_7/connection_phone_5f1/interface_status` contains configuration results as JSON, with transient-local durability.

For the previous static demo, use `config_source:=file` and supply `config_file:=.../phone_mqtt.json` to the launch file or manager. Static file mode ignores device configuration messages. `config_source=auto` (default) selects file mode when a file is supplied, and MQTT device mode otherwise.

For TLS, set `mqtt_tls:=true`, `mqtt_port:=8883`, and optionally `mqtt_ca_file:=/path/to/ca.pem`. System trust is used when no CA file is supplied. Client authentication uses `mqtt_client_cert` and `mqtt_client_key` paths (unencrypted PEM key). The broker certificate must match its hostname/IP. Use a ROS parameter YAML file for credentials rather than placing secrets in shell commands. The manager writes its child parameters into a mode-0600 temporary file and deletes it when the child exits or stops.

## Arduino / serial

Install `embedded/arduino` as an Arduino library and flash `examples/SerialBridgeDevice`. It publishes a sample temperature once per second and receives the `motor.target` Int32 command. Configure the serial port before `bridge.begin()`; do not mix debug prints with bridge frames on that UART.

```bash
BRIDGE_CONFIG_DIR="$(ros2 pkg prefix bridge_node)/share/bridge_node/config"
ros2 launch bridge_node bridge.launch.py \
  connection_id:=esp32_lab_1 protocol:=serial domain_id:=7 \
  config_file:="$BRIDGE_CONFIG_DIR/arduino_serial.json" \
  serial_port:=/dev/ttyUSB0 baudrate:=115200

# In another sourced terminal:
ros2 topic echo /ros2embedded/domain_7/connection_esp32_5flab_5f1/temperature
ros2 topic pub --once \
  /ros2embedded/domain_7/connection_esp32_5flab_5f1/motor_target \
  std_msgs/msg/Int32 '{data: 42}'
```

The port must exist and open successfully at startup. After a disconnect, the running handler retries every two seconds using the configured path; use a stable `/dev/serial/by-id/...` path for hardware. Commands queued when the port fails are discarded.

## Configuration and interfaces

See [communication contract](docs/communication.md) for wire formats, parameter names, limits, type mappings, and adding other devices. See [architecture](docs/architecture.md) for component responsibilities.

Both the device description and optional host config file use protobuf `BridgeConfig`, represented as JSON by default. Their connection ID, domain, and protocol must match node parameters. Device descriptions require `schema_version=1` and a unique `configuration_id` for each changed snapshot. Device descriptions must use relative topics; host files can opt into absolute topics. Relative ROS topics are scoped under `/ros2embedded/domain_<id>/connection_<escaped-id>/`; absolute topics explicitly opt out of this isolation. To make names legal and collision-free, non-alphanumeric connection characters are escaped as `_` plus their two-digit ASCII hex code (`phone_1` becomes `connection_phone_5f1`). The launch file and manager also give each bridge a unique ROS node name. For direct executable runs, supply a unique `__node` remapping when running multiple instances.

`domain_id` is a uint32 bridge guard, separate from DDS `ROS_DOMAIN_ID`. It is not authentication.

## Manager

For example:

```bash
ros2 run bridge_manager bridge_manager --ros-args \
  -p auto_start:=true -p connection_id:=phone_1 -p protocol:=mqtt \
  -p domain_id:=7 -p mqtt_broker:=192.168.0.81 \
  -p config_source:=device
```

The manager launches the bridge executable directly, reports whether the process is running/exited, and terminates/reaps children on shutdown. Its `start_bridge`/`stop_bridge` Python methods support explicit management; there is no remote management service or automatic process restart policy yet. Transport status belongs to each bridge's `/status` topic.

## Tests

```bash
# Unit/codec/PTY tests, including the Arduino source compiled against a host serial stub:
colcon test --base-paths ros2
colcon test-result --verbose

# Full local MQTT + protobuf + ROS + serial pipeline (does not use your real broker):
sudo apt-get install mosquitto python3-paho-mqtt python3-protobuf
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173 python3 -m unittest discover -s tests -v
```

The integration suite starts temporary loopback brokers and multiple bridges, including device-advertised interface bootstrap and live reconfiguration. It checks both directions, domain/type rejection, connection isolation, broker restart, virtual serial framing, launch arguments, manager shutdown, TLS/mTLS, and authentication failures. Hardware and the actual Flutter app still require a separate on-device check.
