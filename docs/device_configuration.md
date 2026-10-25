# Device-advertised interfaces / Flutter handoff

The bridge accepts a complete device interface description, creates ROS publishers/subscribers automatically, and acknowledges the result. The device controls which identifiers exist; no phone sensor catalog is hard-coded into the ROS node. A single bridge still owns one configured connection ID/domain. This implements interface discovery within that connection, not automatic creation of manager processes for unknown devices.

This repository keeps the sibling Flutter checkout read-only. Its updated Rosnect implementation now sends interface snapshots and live Android sensor data. The exchange below is also the contract for other clients. Existing data messages remain compatible; native sensor declarations are supported as described in [native sensor conversion](native_sensors.md).

## Start the host

From the built, sourced `ros2embedded` repository:

```bash
ros2 run bridge_manager bridge_manager --ros-args \
  -p auto_start:=true \
  -p connection_id:=phone_1 -p protocol:=mqtt -p domain_id:=7 \
  -p mqtt_broker:=192.168.0.81 -p mqtt_port:=1883 \
  -p config_source:=device
```

Do not pass `config_file` in device mode. `config_source=auto` (default) selects device mode for MQTT without a file; supplying a file selects the old static file mode. Explicit device mode plus a file is rejected so that the source of truth is unambiguous. Serial currently retains file-based configuration; the transport interface has an optional configuration channel for future transports.

The broker connection status remains on `/ros2embedded/domain_7/connection_phone_5f1/status`. The separate transient-local `/ros2embedded/domain_7/connection_phone_5f1/interface_status` contains the most recent configuration result as a JSON String. Until a valid snapshot arrives, only internal status interfaces exist; unknown telemetry does not create topics implicitly.

## MQTT control topics

For connection `phone_1`, domain `7`, default base topic:

| Topic | Sender → receiver | Format | QoS / retain |
|---|---|---|---|
| `ros2embedded/7/phone_1/config` | Device → bridge | Raw BridgeConfig JSON or binary protobuf | 1 / true recommended |
| `ros2embedded/7/phone_1/config_status` | Bridge → device | BridgeConfigStatus JSON | 1 / false |
| `ros2embedded/7/phone_1/config_request` | Bridge → device | BridgeConfigRequest JSON | 1 / false |
| `ros2embedded/7/phone_1/to_ros` | Device → bridge | Existing data envelope | 1 / false recommended |
| `ros2embedded/7/phone_1/to_device` | Bridge → device | Existing data envelope | 1 / false |

Configuration messages are **not** wrapped in `BridgePayload`. Route incoming MQTT messages by their topic before attempting to decode data envelopes. Clients need a separate parser/stream for control results rather than passing them to `BridgePayload.fromJson`.

`config_encoding=json` is the host default. To use binary BridgeConfig messages, start with `config_encoding=protobuf`. This is independent of `payload_encoding`, which still selects JSON/protobuf for data. Requests and status responses always use JSON (protobuf JSON representation with snake_case field names). Schemas, including request/status messages, are in `proto/bridge_config.proto`.

## Snapshot sent by the app

```json
{
  "schema_version": 1,
  "configuration_id": "a-new-uuid-for-this-interface-change",
  "connection_id": "phone_1",
  "domain_id": 7,
  "protocol": "PROTOCOL_MQTT",
  "bindings": [
    {
      "identifier": "phone.accelerometer",
      "direction": "DEVICE_TO_ROS",
      "value_type": "JSON",
      "ros_topic": "accelerometer"
    },
    {
      "identifier": "phone.gyroscope",
      "direction": "DEVICE_TO_ROS",
      "value_type": "JSON",
      "ros_topic": "gyroscope"
    },
    {
      "identifier": "robot.status",
      "direction": "ROS_TO_DEVICE",
      "value_type": "STRING",
      "ros_topic": "robot_status"
    }
  ]
}
```

The snapshot is the **entire desired interface set**, not a patch. Build it from enabled sensors and active subscriptions. Removing an entry removes that ROS binding on acceptance. An empty `bindings` array unregisters all device interfaces without disconnecting. Do not send an empty MQTT payload to unregister interfaces: that deletes the broker's retained message and is not a valid BridgeConfig.

Directions are from the bridge/device perspective:

- App publishing a sensor → `DEVICE_TO_ROS`: bridge creates a ROS publisher.
- App subscribing to an identifier → `ROS_TO_DEVICE`: bridge creates a ROS subscription.
- Both directions → `BIDIRECTIONAL`: uses distinct ROS output and command topics to avoid feedback.

`value_type` is required and uses the uppercase protobuf enum names, **not** the lower-case data envelope spellings. Match the actual values sent: Rosnect's structured sensor values use `JSON`; a numeric Dart double inferred as `float64` requires config `FLOAT64`. Subscriptions must declare the expected value type as well; an identifier alone cannot determine a ROS message type. If the app both publishes and subscribes to the same identifier, merge those entries into one `BIDIRECTIONAL` binding with the same value type; duplicate identifiers or conflicting types are rejected. Existing text subscriptions can default to `STRING`, with a type selector added for other kinds.

`ros_topic` and `ros_message_type` are optional in device mode. The host infers the default ROS type from `value_type`. For native sensors, explicitly set `ros_message_type` to a supported `sensor_msgs/msg/...` type and keep `value_type=JSON`; the host then validates/converts the structured fields. If no topic is supplied, it generates a unique legal relative name:

- `phone.accelerometer`, device → ROS: `data/id_phone_2eaccelerometer`
- `robot.status`, ROS → device: `commands/id_robot_2estatus`

Non-alphanumeric identifier bytes are escaped as `_` plus two ASCII hex digits. A bidirectional binding defaults its command topic to `commands/id_<escaped-identifier>`. All device-supplied topics must remain relative and within that connection's ROS namespace. Absolute/private topics, reserved status topics, and output/command feedback loops are rejected. Supplying friendly relative topics, as in the example, gives `/ros2embedded/domain_7/connection_phone_5f1/accelerometer`, etc.

The ACK echoes the normalized bindings, including inferred types/topics and `ros_namespace`. Use that result to display the actual ROS interfaces rather than duplicating name generation in Flutter.

## App lifecycle

1. Connect to the broker, then subscribe to `to_device`, `config_status`, and `config_request`. Wait for successful SUBACKs.
2. Create the current full snapshot and publish it on `config`, QoS 1, retained. Generate a new UUID for a changed snapshot; retries of the same snapshot reuse its ID and contents.
3. Wait for `config_status` with matching connection ID, domain ID, and `configuration_id`, and `accepted=true`. Only then mark these interfaces ready and start publishing their data.
4. Republish the current snapshot after every broker connection/reconnection and whenever a matching `config_request` arrives. The bridge requests it after reconnecting and every two seconds while unconfigured. Retention also lets a restarted bridge bootstrap before the app explicitly resends.
5. When sensors are enabled/disabled, identifiers change, subscriptions are added/removed, or value types change, publish a new full snapshot with a new ID. Serialize updates: keep one snapshot awaiting acknowledgement; coalesce newer UI edits into the next snapshot.
6. Retry an unacknowledged snapshot on a bounded timer (for example every two seconds); show a waiting/error state after an application timeout. QoS 1 PUBACK only proves broker receipt, not bridge configuration.
7. On rejection, keep the last accepted snapshot active in the UI and report `error`. Correct the candidate and send a new ID. A rejected retained snapshot replaces the broker's previous retained value, even though the running bridge preserves its old interfaces. Republish the last accepted snapshot or a corrected one to restore a valid retained description before a host restart.

An acknowledgement looks like this (some fields abbreviated):

```json
{
  "schema_version": 1,
  "connection_id": "phone_1",
  "domain_id": 7,
  "configuration_id": "a-new-uuid-for-this-interface-change",
  "active_configuration_id": "a-new-uuid-for-this-interface-change",
  "accepted": true,
  "error": "",
  "state": "ready",
  "bindings_count": 3,
  "payload_encoding": "json",
  "ros_namespace": "/ros2embedded/domain_7/connection_phone_5f1",
  "bindings": []
}
```

The actual `bindings` array contains all active normalized bindings; it is abbreviated above. `accepted=false` means the requested update failed. `state=ready` can still accompany a rejection when a previous configuration remains active; readiness for the requested snapshot requires both `accepted=true` and a matching ID. A malformed message may get a response with an empty `configuration_id` because its ID could not be parsed. Status is deliberately not retained on MQTT, avoiding stale readiness acknowledgements from a stopped bridge.

Repeated identical snapshots are acknowledged without rebuilding. Changed content with an active ID is rejected. The host remembers the last 128 superseded IDs to reject recent out-of-order retries. Configuration IDs are opaque correlation IDs, not a persistent revision history. Use a new ID even when reverting to an older set of bindings.

Only one device owner should publish a connection's config topic. Bindings survive broker disconnects in the running host; broker status still reports disconnected. Configuration readiness does not prove the device is online. In-flight data/commands are not cancelled by a configuration ACK; applications needing expiry/acknowledgement must implement it at the payload level. Devices should not change the type on a ROS topic already used by other nodes; publish the new type under a new topic. If the ROS middleware cannot stage an interface replacement, the update is rejected and the previous interfaces remain active.

## Client implementation checklist (including Flutter)

- Maintain enabled sensors and subscriptions in shared interface state accessible to the connection/session, rather than only local widget lists.
- Add a type to subscription entries; preserve current JSON data envelopes. Merge publish/subscribe entries sharing an identifier into a single bidirectional binding, and validate identifiers before sending.
- Add raw config publishing and config request/status parsing in the MQTT handler, separated by topic from data delivery.
- Send a full snapshot after connecting and whenever that shared interface state changes. Editing an identifier must update the snapshot and the sender together.
- Expose broker connectivity and interface readiness separately. Do not treat MQTT connection acceptance as a successful ROS configuration.
- Wait for the matching accepted result before enabling data publishing for the new interfaces; use the normalized ACK for topic display.

The configuration exchange itself does not capture sensors. The updated Rosnect Android app implements live capture separately and waits for interface readiness before streaming; native message converters are transport-independent bridge functionality.

## Simulate a device without Flutter

Start the host command above, then from a sourced repository terminal:

```bash
/usr/bin/python3 tools/publish_device_config.py \
  ros2/bridge_node/config/phone_device_config.json \
  --broker 192.168.0.81 --port 1883
```

This helper simulates a device's retained configuration message and waits for its ACK. The file above demonstrates JSON-in-String interfaces; use `phone_native_config.json` instead for native sensor declarations. Do not have the helper compete with a running app that owns the same config topic. The updated Rosnect app already publishes its own snapshots. For TLS use `--tls`, optionally `--ca-file`, `--client-cert`, and `--client-key`. Broker credentials use `--username` and `--password-env NAME`.

For a new demo binding set, edit the JSON and change `configuration_id` before rerunning. Retained configs persist in the broker until replaced/deleted; replace a test description with the app's own snapshot when resuming app-owned operation.

The local integration tests in `tests/test_device_configuration.py` cover JSON and binary protobuf configs, bidirectional data, defaults, validation, idempotency, removal, restart bootstrap, and manager startup without a file. They never modify Flutter or contact the real broker.
