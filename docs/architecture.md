# ROS 2 Embedded Bridge Architecture

## Goal

Connect embedded hardware and other clients to ROS 2 through configurable interfaces, keeping transport choice independent from device platform and ROS topic wiring. ROS 2 Humble is the host target; Arduino ESP32/ESP8266 and the separately maintained Rosnect Flutter app are initial clients.

## Components

```text
Manager
  └── Bridge node for each connection
        ├── Validated BridgeConfig
        ├── Typed ROS publishers/subscribers
        └── ConnectionHandler plugin
              ├── MQTT + JSON or protobuf
              ├── Serial + bounded line framing
              └── Redis (reserved; explicitly unavailable)
```

The manager owns child processes and their parameter files. It starts the installed bridge executable directly, tracks running/exited state, supports stop/restart through its Python API, and terminates/reaps children at shutdown. Automatic process restart policies, remote management services, and manager discovery of new connection IDs remain future work.

Each bridge obtains a protobuf `BridgeConfig` (JSON or binary) from its device over a transport control channel, or from an explicit host file. It validates all bindings, creates the configured ROS interfaces, and runs one transport plugin. MQTT device mode accepts complete interface snapshots, stages replacements, commits them on the ROS executor, and acknowledges results; invalid updates preserve the active interfaces. Retained snapshots allow bootstrap after a bridge restart. The config identity must match its node parameters. ROS topics are scoped per domain/connection unless explicitly absolute. Bidirectional identifiers use distinct ROS output and command topics to prevent feedback. Standard scalar, string, byte-array, structured JSON, and seven explicit native sensor schemas are supported; services, actions, and arbitrary ROS message layouts remain extension work.

The `ConnectionHandler` interface retains `configure`, `connect`, `disconnect`, `is_connected`, `send`, `poll`, and receive callbacks. An optional configuration callback/request/status interface lets transports deliver descriptions without moving ROS binding policy into the transport. Serial retains file configuration for now. Its data messages are common typed `TransportPayload` objects, independent of wire representation. `connect()` means transport startup was accepted; MQTT becomes connected after CONNACK and SUBACK. `send()` accepts a message for transport, not an application-level delivery acknowledgement.

Codecs own wire encoding and strict type/range validation. JSON preserves compatibility with the existing Flutter MQTT contract; protobuf implements the versioned binary payload schema; serial framing preserves the Arduino protocol while implementing both directions. A new device can use any implemented transport/codec combination without changing ROS routing. Domain guards apply both at the transport boundary and before ROS publication. Native sensor converters are selected by advertised ROS message type, never by client/device identity; JSON remains the structured wire value and String fallback. [Native sensor conversion](native_sensors.md) documents the schema and resource limits.

MQTT uses a network worker and bounded queues. Receive callbacks are drained by `poll()` on the ROS executor so I/O callbacks never publish through an expired node. Serial uses nonblocking Linux I/O, bounded frame/output buffers, and bounded work each tick. Transport destruction closes resources; MQTT destruction stops/joins its worker before unloading the plugin library.

## Contracts and configuration

- `proto/bridge_config.proto`: connection identity, protocol, identifiers, ROS bindings, types, directions.
- `proto/bridge_payload.proto`: typed values and optional Unix timestamp (zero means unknown).
- `ros2/connection_handlers/include/ros2_embedded/payload.hpp`: transport-independent host representation and codec API.
- [Device configuration / Flutter handoff](device_configuration.md): configuration topics, handshake, snapshots, and client changes.
- [Communication contract](communication.md): supported mappings, wire formats, limits, and extension rules.

The bridge `domain_id` is separate from DDS `ROS_DOMAIN_ID`. It prevents accidental cross-domain acceptance but is not authentication. Broker credentials/TLS are transport settings and remain outside device binding configuration.

## Validation

Codec tests cover scalar boundaries, malformed envelopes, all supported value types, and JSON/protobuf/serial round trips. The actual Arduino library source is built with a host serial stub and checked against host codecs. PTY tests exercise Linux serial framing and disconnects. Manager tests cover process cleanup and replacement. The integration scripts run temporary MQTT brokers, multiple ROS bridges, JSON and protobuf devices, and a serial device; they verify traffic isolation, both directions, broker restart, device configuration bootstrap, live interface replacement/removal, rejection, idempotent retries, retained-config recovery, native sensor units/unknown fields, malformed measurements, and bounded camera payloads in both directions.

Physical device checks and running the actual Flutter app are separate validation steps. This repository does not modify the sibling Flutter project.
