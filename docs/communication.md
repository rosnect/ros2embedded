# Communication contract (v1)

A device needs a transport and a payload codec. Neither determines the device platform. Any MQTT device can use JSON or protobuf; any serial device can implement the line framing below. All paths decode to the same C++ `TransportPayload`, validate it, then route by identifier and expected value type.

## Device configuration exchange

MQTT devices can send their complete interface description to the connection's `/config` topic and receive `/config_status` acknowledgements; `/config_request` asks a device to resend it. See [device configuration and Flutter handoff](device_configuration.md) for the full handshake, update rules, wire examples, and changes needed in the separate app. These control messages are separate from data payload envelopes. An MQTT bridge without a local file defaults to device mode and waits for the description.

## Binding configuration

The schema is `proto/bridge_config.proto`. In file mode, load JSON with `config_file`; use `config_encoding=protobuf` for binary protobuf. In device mode the same setting selects the encoding of incoming `/config` messages. Transport credentials remain ROS parameters, not part of the device binding schema.

```json
{
  "connection_id": "controller",
  "domain_id": 7,
  "protocol": "PROTOCOL_MQTT",
  "bindings": [
    {
      "identifier": "motor.target",
      "ros_topic": "motor_state",
      "ros_command_topic": "motor_command",
      "direction": "BIDIRECTIONAL",
      "value_type": "INT32",
      "ros_message_type": "std_msgs/msg/Int32"
    }
  ]
}
```

Directions are `DEVICE_TO_ROS`, `ROS_TO_DEVICE`, and `BIDIRECTIONAL`. For one-way bindings, `ros_topic` is the respective output or command topic. For bidirectional bindings, `ros_topic` is device output and `ros_command_topic` is the command input. Host files require those topics explicitly; device descriptions can omit them for automatic names. Output/command topics cannot overlap anywhere within one bridge configuration, including after ROS remapping. Explicit absolute topics shared across different bridge configurations remain the operator's responsibility.

Supported mappings:

| Config type | JSON `value_type` | ROS message | Serial code / value bytes |
|---|---|---|---|
| BOOL | `boolValue` | std_msgs/msg/Bool | 1 / one byte, 0 or 1 |
| INT32 | `int32` | std_msgs/msg/Int32 | 2 / four bytes |
| UINT32 | `uint32` | std_msgs/msg/UInt32 | 3 / four bytes |
| INT64 | `int64` | std_msgs/msg/Int64 | 4 / eight bytes |
| UINT64 | `uint64` | std_msgs/msg/UInt64 | 5 / eight bytes |
| FLOAT32 | `float32` | std_msgs/msg/Float32 | 6 / four IEEE 754 bytes |
| FLOAT64 | `float64` | std_msgs/msg/Float64 | 7 / eight IEEE 754 bytes |
| STRING | `string` | std_msgs/msg/String | 8 / UTF-8 bytes |
| BYTES | `bytes` | std_msgs/msg/ByteMultiArray | 9 / raw bytes |
| JSON | `json` | std_msgs/msg/String | 10 / UTF-8 JSON text |

`ros_message_type` may be omitted; the table gives the defaults. With `value_type=JSON`, it may instead name one of the supported native sensor schemas (Imu, MagneticField, NavSatFix, Illuminance, FluidPressure, BatteryState, CompressedImage), all in `sensor_msgs/msg`. See [native sensor conversion](native_sensors.md) for field validation, units, timestamps, and both-direction support. Other incompatible/unsupported combinations are rejected. ByteMultiArray layout metadata is not transmitted. JSON strings on ROS must contain valid non-null JSON. Scalar std_msgs output omits envelope identifiers/timestamps because the binding determines its topic/type and those messages have no header. Native sensors preserve a JSON header, or use the envelope timestamp if the header stamp is absent.

Identifiers are 1–96 ASCII characters: letters, digits, `_`, `-`, `.`, `/`. Connection IDs are 1–96 characters and exclude `/`. Domains range from 0 through 4294967295. Each configuration has 0–256 unique identifiers; an empty device snapshot unregisters all application bindings. Device snapshots require `schema_version=1` and `configuration_id` (1–96 ASCII letters/digits, `_`, `-`, `.`); legacy host files may omit these fields. Numeric values must match the declared type/range; NaN and infinity are rejected. Unsupported/mismatched types are dropped, never coerced into another binding's type.

## MQTT

Parameters: `mqtt_broker` (hostname/IP), `mqtt_port` (1883 by default, 8883 with TLS), `mqtt_base_topic` (default `ros2embedded`), `mqtt_username`, `mqtt_password`, `mqtt_tls`, `mqtt_ca_file`, `mqtt_client_cert`, `mqtt_client_key`, and `payload_encoding` (`json` default or `protobuf`). TLS paths refer to files on the ROS machine. Password-protected private keys are not supported by this host implementation.

For connection `phone_1`, domain 7, device-to-host traffic goes to:

```text
ros2embedded/7/phone_1/to_ros
```

Host-to-device traffic goes to:

```text
ros2embedded/7/phone_1/to_device
```

All topics in a connection use the same selected codec. The host subscribes/publishes at QoS 1, without retaining outgoing commands, and uses clean sessions. Host connection state becomes ready after the broker accepts the input subscription. The network worker reconnects with exponential backoff capped at 30 seconds and subscribes again after reconnecting.

The JSON envelope matches the existing Flutter library:

```json
{"identifier":"robot.status","domain_id":7,"value_type":"string","value":"ready","timestamp_unix_ms":1700000000000}
```

Bytes are JSON arrays of integers 0–255. Structured JSON is carried directly in `value`, not as a doubly encoded string. Timestamp is optional/null; received milliseconds convert to nanoseconds internally. JSON encoding truncates outgoing nanoseconds to milliseconds. The `boolValue` spelling intentionally matches Flutter. Full-range uint64 values are supported by the host/protobuf but are not portable through every JSON consumer; use a device-specific supported range or protobuf when full-range integers are needed.

The protobuf codec uses the binary `ros2embedded.v1.BridgePayload` message. A value must be set in its `oneof`; `json_value` is JSON text. These messages retain nanosecond timestamps. Existing protobuf field numbers remain unchanged.

Incoming data envelopes are capped at 1 MiB, structured JSON values at 512 KiB, and scalar string/byte values at 4096 bytes. CompressedImage decoders separately cap image bytes at 256 KiB. The application receive queue holds at most 128 ordered data/config events, with at most eight pending config messages. Data is limited to 120 queued events to reserve control space; incoming config can evict a data event at capacity. New overflow events are dropped. Config wire messages and normalized acknowledgements are capped at 64 KiB (with error headroom). Each direction also has a 4 MiB queued-content budget (metadata and decoder temporaries are additional). Receive polling drains at most 16 events / approximately 1 MiB per tick. At most 120 data publishes may await acknowledgement, with eight extra slots and 512 KiB reserved for control, up to 128 total. Backpressure drops/rejections are reported; buffers do not grow without limit. `send()` success means accepted into the transport, not consumed by the remote device. QoS 1 can deliver duplicates and unacknowledged messages may be retransmitted after a reconnect; actuator protocols requiring expiry/deduplication must supply that at the application level. Neither device presence nor application-level acknowledgement is inferred from broker connectivity.

## Serial

Parameters: `serial_port`, `baudrate`, `payload_encoding=serial`. Baud rates supported here: 9600, 19200, 38400, 57600, 115200, 230400. UART settings are 8N1 without flow control. Use a dedicated data UART.

Each frame is ASCII:

```text
R2E,<domain decimal>,<type code decimal>,<identifier>,<hex value>\n
```

CRLF is also accepted. All scalar value bytes are little-endian; signed integers use two's complement. Floats use IEEE 754. String/JSON/byte values are hex-encoded too, so embedded newlines and commas in values are safe. Empty string and byte values have an empty hex field. Identifiers cannot contain commas, whitespace, or newlines. Frames contain no timestamp; the internal timestamp is unknown/zero rather than inventing a Unix timestamp from device uptime.

Examples:

```text
R2E,7,6,sensor.temperature_c,0000C441
R2E,7,2,motor.target,2A000000
R2E,7,8,label,
```

Values are capped at 256 bytes, frames at 640 bytes, and the host output queue at 64 KiB. Oversized/malformed frames are discarded through the next newline; partial valid frames persist across reads. The host performs bounded work per timer tick. Serial has framing and validation but no checksum or delivery acknowledgement. On link loss, buffered output is discarded, and the host retries opening the configured path every two seconds. Failure to open at initial startup fails the bridge explicitly.

The Arduino handler owns received identifier/data buffers. They remain valid only until the next `readData()` or `begin()`; copy them if retaining a command. The `BridgePayload.data` bytes use the wire's little-endian format; do not cast them to potentially unaligned typed pointers. `connected()` indicates that `begin()` ran, not that the USB peer is online. Serial sends report short writes as failures.

## Extending the system

Implement `ConnectionHandler` for a new transport and reuse a codec, or implement a codec that produces/consumes validated `TransportPayload` objects. Its optional device-configuration callbacks/request/status methods can carry the shared BridgeConfig control exchange; otherwise use host file configuration. Callbacks into the ROS node must run from `poll()` on its executor, rather than an I/O worker. Keep buffers and per-tick work bounded. Device code does not need ROS/DDS or knowledge of ROS topic names; it sends stable identifiers. Add explicit supported ROS type mappings for additional message types instead of introducing platform-specific branches into the bridge node.
