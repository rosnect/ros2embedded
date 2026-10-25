# Native ROS sensor messages

The bridge can translate structured JSON values into supported `sensor_msgs` types. The converter is selected by a binding's `ros_message_type`, independent of client platform, transport, identifier spelling, or topic name. The same path works with MQTT JSON envelopes, protobuf `json_value` envelopes, and small JSON-over-serial frames within serial's existing limits.

Keep `value_type=JSON` in the interface description and `value_type=json` in JSON data envelopes. `ros_message_type` selects the ROS schema. Omitting it retains the existing JSON-in-`std_msgs/msg/String` behavior.

## Supported types

| ROS message type | Required measurement fields | Units / notes |
|---|---|---|
| `sensor_msgs/msg/Imu` | At least one of `orientation`, `angular_velocity`, `linear_acceleration` | Quaternion; rad/s; m/s². Covariances are optional 9-element arrays. |
| `sensor_msgs/msg/MagneticField` | `magnetic_field` with x/y/z | Tesla, not microtesla. Optional `magnetic_field_covariance`. A null axis represents an unreported component (NaN in ROS). |
| `sensor_msgs/msg/NavSatFix` | `latitude`, `longitude` | WGS84 degrees; optional altitude is metres above the ellipsoid. Optional `status`, `position_covariance`, `position_covariance_type`. |
| `sensor_msgs/msg/Illuminance` | `illuminance` | Nonnegative lux; optional nonnegative `variance`. |
| `sensor_msgs/msg/FluidPressure` | `fluid_pressure` | Nonnegative absolute pascals; optional nonnegative `variance`. |
| `sensor_msgs/msg/BatteryState` | `voltage` | Nonnegative volts; percentage is a 0..1 fraction. Other standard fields are optional. |
| `sensor_msgs/msg/CompressedImage` | `format` and exactly one of `data_base64` / `data` | Base64 text or byte array; 1..256 KiB decoded image. |

These are explicit supported schemas, not arbitrary ROS-message deserialization. Unknown fields, wrong JSON scalar types, unsupported ROS types, incorrect covariance lengths, invalid ranges, and malformed base64 are rejected. Bad telemetry is dropped without destroying the configured interfaces. Invalid configurations retain the previously active bindings.

The current Rosnect Android app already shapes its values this way, including microtesla → tesla, hPa → Pa, and battery fraction conversion. The bridge does **not** rescale them again. Sensor axes and frame IDs are preserved; the bridge does not publish TF or rotate phone coordinates into a robot frame. Other devices must supply the same ROS units and meaningful frame IDs.

## Example: accelerometer

Advertise the native type in a device snapshot:

```json
{
  "schema_version": 1,
  "configuration_id": "native-example-1",
  "connection_id": "phone_1",
  "domain_id": 7,
  "protocol": "PROTOCOL_MQTT",
  "bindings": [
    {
      "identifier": "phone.accelerometer",
      "direction": "DEVICE_TO_ROS",
      "value_type": "JSON",
      "ros_message_type": "sensor_msgs/msg/Imu",
      "ros_topic": "accelerometer"
    }
  ]
}
```

After its matching accepted ACK, publish the existing envelope to `ros2embedded/7/phone_1/to_ros`:

```json
{
  "identifier": "phone.accelerometer",
  "domain_id": 7,
  "value_type": "json",
  "timestamp_unix_ms": 1700000000123,
  "value": {
    "header": {
      "stamp": {"sec": 1700000000, "nanosec": 123000000},
      "frame_id": "phone_link"
    },
    "linear_acceleration": {"x": 1.25, "y": -2.0, "z": 9.81}
  }
}
```

The ROS topic `/ros2embedded/domain_7/connection_phone_5f1/accelerometer` has type `sensor_msgs/msg/Imu`. Missing orientation/angular velocity are marked unavailable by covariance element zero equal to -1. Missing orientation uses an identity quaternion placeholder, not an asserted orientation estimate. A gyroscope can publish a separate Imu topic containing only `angular_velocity`; the bridge does not fuse the two topics.

`ros2/bridge_node/config/phone_native_config.json` is a full native-sensor example for a simulated device. In automatic mode the app publishes its own enabled interface set; do not have an independent helper overwrite that config while the app owns the connection.

## Timestamps, covariance, and unknown measurements

- Every type accepts optional `header` with `frame_id` and `stamp: {sec, nanosec}`. Explicit header time wins, including explicit zero. Without it, the envelope's Unix timestamp is used. With neither, ROS time stays zero; arrival time is not substituted for measurement time.
- ROS Time seconds must fit int32 and nanoseconds must be 0..999999999. Frame IDs are UTF-8 strings up to 256 bytes without NUL characters.
- Vector objects require numeric x/y/z; quaternion objects require x/y/z/w. Available IMU orientation must be a unit quaternion (norm tolerance 1e-3). Unavailable orientation may use its placeholder with covariance[0]=-1.
- Covariances contain exactly nine finite numbers. Diagonal variances must be nonnegative, except the IMU unavailable sentinel -1. An omitted covariance for a provided measurement is all zeros (unknown uncertainty). Missing IMU measurements always get the unavailable sentinel. A covariance without its IMU measurement must explicitly use that sentinel.
- `NavSatFix` accepts status values -1..2 and service bitmask 0..15. If status is omitted, it defaults to no fix (-1). Latitude/longitude must be within ±90/±180 degrees; they may be null for no-fix messages. Missing/null altitude becomes NaN, rather than zero. Covariance type is 0..3; a known type requires covariance values. The bridge preserves the supplied type, including unknown (0).
- `BatteryState` optional temperature/current/charge/capacity/design_capacity/percentage become NaN when absent/null. Supplied values must fit float32; percentage must be a fraction, charge/capacities nonnegative. Status/health/technology use their ROS enum ranges. `present` defaults false unless supplied. Optional cell arrays accept at most 256 values, with null for unknown cells. Location and serial number strings are limited to 256 bytes.
- Nullable unknown ROS numbers are serialized as JSON null in the reverse direction. Infinity is rejected. Finite values are preserved without sensor-specific correction or calibration.

## Camera frames and resource limits

`data_base64` follows standard padded base64 without whitespace. Invalid characters, noncanonical padding, two simultaneous data representations, empty images, and images exceeding 256 KiB are rejected. `data` is an alternative array of integers 0..255 for small device messages. Use base64 for large frames; a large integer array may exceed the JSON value-size limit. The converter does not decompress or verify the codec bytes; consumers must support the advertised format.

The frame's format (for example `jpeg`) and optical frame ID are preserved. Native ROS-to-device image messages are encoded with `data_base64`. Structured JSON values are limited to 512 KiB, data envelopes to 1 MiB. String/byte scalar values retain the 4096-byte limit. Configuration messages remain limited to 64 KiB; serial values remain 256 bytes.

MQTT application queues enforce a 4 MiB content budget in each direction as well as message-count limits. Metadata, decoder temporaries, and middleware buffers are additional bounded costs. Receive processing drains at most 16 events / approximately 1 MiB per poll to avoid camera traffic monopolizing the ROS executor. Control traffic keeps reserved outgoing count/byte capacity, and incoming configurations can evict data when necessary. Overflow drops/rejections are reported, not buffered without limit. Lower camera frame rate/size when backpressure warnings appear; this remains a bounded MQTT bridge, not a dedicated video transport.

Native ROS publishers are reliable with depth 5 (depth 1 for images), matching reliable and best-effort subscribers. Native ROS-to-device subscriptions use best-effort sensor QoS with the same depth and accept both reliable and best-effort producers. This avoids retaining a backlog of old frames.

## View the camera in RViz

The app's camera binding publishes `sensor_msgs/msg/CompressedImage` on `/ros2embedded/domain_7/connection_phone_5f1/camera/image/compressed`. For a plain video preview use RViz's **Image** display. The **Camera** display also needs calibrated `CameraInfo` and transforms; neither is supplied by the current phone interface. A topic existing and the app's sent counter increasing do not alone establish image decodability.

RViz's compressed image transport can consume the compressed topic with `compressed_image_transport` installed. To expose a regular `sensor_msgs/msg/Image` topic that appears in the Image display's dropdown, run the standard republisher in an additional terminal while the bridge and phone camera continue running:

```bash
cd /home/oseekero/robotics_stuff/ros2embedded
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/ros2embedded/domain_7/connection_phone_5f1/camera/image/compressed \
  -r out:=/ros2embedded/domain_7/connection_phone_5f1/camera/image
```

Leave it running, then in RViz:

1. Remove or disable the old **Camera** display.
2. Select **Add → By display type → Image**.
3. Set its **Topic** to `/ros2embedded/domain_7/connection_phone_5f1/camera/image` and press Enter.
4. Set Reliability to **Best Effort** and Durability to **Volatile**.

Keep Rosnect in the foreground with Camera enabled; the app pauses capture when backgrounded or screen-locked. The republisher requires `image_transport` and `compressed_image_transport` (`sudo apt-get install ros-humble-image-transport ros-humble-compressed-image-transport` if missing). It does not change the MQTT protocol or the bridge's binding configuration. Stop it with Ctrl+C when finished. Real Camera-display overlays require actual camera calibration and TF; do not invent calibration just to suppress RViz errors.

## Reverse direction and configuration changes

Native types also support `ROS_TO_DEVICE` and `BIDIRECTIONAL` bindings. A ROS sensor message is validated and encoded as structured JSON inside the normal device payload. The JSON header is preserved; its nonnegative time is also used for the outer timestamp. Bidirectional bindings require distinct output/command topics as before.

A sensor's ROS type is part of its interface, not something guessed from each sample. Changing a live topic from String to Imu should use a new topic or a bridge restart; middleware may reject staging different types under the same existing topic. Stop old bridge instances and old `ros2 topic echo` processes before switching an existing deployment's topic types.

## Build and run with the updated app

No additional Flutter changes are required for the native declarations in the current `mqtt_connection` checkout. Keep its `sensorMessageTypes` mapping and `_advertise()` field; do not replace them with String-only declarations. Live capture in that app is currently Android-only.

After stopping the previous bridge/manager, from the repository:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ros2 --ignore-src -r -y
colcon build --symlink-install --base-paths ros2
source install/setup.bash
ros2 run bridge_manager bridge_manager --ros-args \
  -p auto_start:=true -p connection_id:=phone_1 -p protocol:=mqtt \
  -p domain_id:=7 -p mqtt_broker:=192.168.0.81 -p mqtt_port:=1883 \
  -p config_source:=device
```

Reconnect Rosnect and enable the sensors. It should receive an accepted configuration ACK and begin streaming. In another sourced terminal:

```bash
ros2 topic type /ros2embedded/domain_7/connection_phone_5f1/accelerometer
ros2 topic echo /ros2embedded/domain_7/connection_phone_5f1/accelerometer sensor_msgs/msg/Imu
```

The type query should print `sensor_msgs/msg/Imu`. Other native sensor topics follow the app's advertised relative topics. Touch/microphone custom JSON can continue using `std_msgs/msg/String`.
