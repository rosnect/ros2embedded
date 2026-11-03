# Rosnect live sensors and RViz (ROS 2 Humble)

Rosnect is one device client of the generic bridge. Any conforming device can advertise the same supported ROS sensor schemas. No phone identifiers are hard-coded into the bridge.

## Linux build and run

Copy/update **this version of ros2embedded** on the Linux ROS machine before testing the new app. Earlier builds only accept std_msgs and 4 KB values, so they will reject these interfaces or drop images.

From the repository root:

```bash
source /opt/ros/humble/setup.bash
sudo apt-get install libmosquitto-dev nlohmann-json3-dev libprotobuf-dev protobuf-compiler \
  ros-humble-sensor-msgs ros-humble-rviz2 ros-humble-image-transport \
  ros-humble-compressed-image-transport
rosdep install --from-paths ros2 --ignore-src -r -y
colcon build --symlink-install --base-paths ros2
source install/setup.bash
ros2 run bridge_manager bridge_manager --ros-args \
  -p auto_start:=true -p protocol:=mqtt \
  -p connection_id:=phone_1 -p domain_id:=7 \
  -p mqtt_broker:=192.168.0.81 -p mqtt_port:=1883 \
  -p config_source:=device
```

Stop any older bridge for the same connection before launching this one. Do not run `publish_device_config.py` alongside Rosnect: the app owns the full retained config. No `config_file` is needed. Keep `config_encoding=json` and `payload_encoding=json` (defaults). Broker credentials/TLS, if enabled, must also be configured on the host.

## On the phone

1. Connect to `192.168.0.81`, Normal 1883, connection ID `phone_1`, domain `7`.
2. Open Sensors. Enable Accelerometer first. The app advertises the new full interface set, waits for the matching bridge ACK, then begins capture.
3. Check **ROS interfaces ready** and the sensor's sent counter. Each card displays its acknowledged ROS topic and latest real reading.
4. Enable Camera, accept camera permission, and leave Rosnect in the foreground. A JPEG preview should appear. Start with rear camera at 2 FPS; 1/2/5 FPS are available.
5. Enable other available sensors individually and accept any requested permissions. Unsupported hardware is marked unavailable. GPS may need time outdoors to obtain a fresh fix; no last-known location is invented.
6. Submit edited identifiers with the keyboard's Done/Enter action. Subscription entries declare an expected value type and are advertised automatically. Use separate command identifiers for structured sensor outputs.

A green broker dot and **ROS interfaces ready** are separate states. A timeout/rejection displays a reason and Retry ROS setup. The app does not publish selected sensor data until the current full snapshot is acknowledged. Configuration updates are serialized and intermediate edits coalesced. A rejected snapshot stops streaming and restores the last accepted retained configuration; fix the selection or retry.

## Permissions and foreground behavior

- Camera: allow **while using the app** when Camera is enabled.
- Microphone: allow **while using the app** for Microphone. Only RMS and dBFS levels are sent, not recorded audio.
- Location: allow **while using the app**, enable the Android Location toggle, and choose Precise for GPS. Approximate access can use the network provider. No background-location permission is requested.
- Accelerometer/gyroscope/magnetometer/light/barometer/battery/touch do not require a dangerous runtime permission at these rates.
- If access was denied, use **App permissions**, or Android Settings → Apps → Rosnect → Permissions. Android camera/microphone privacy toggles must also permit access. Other apps using the camera/microphone can make those sources unavailable.
- Capture pauses when Rosnect is backgrounded or the screen locks. Resume the app to restart acknowledged sources. If a source reports an error, turn its switch off/on after fixing the cause.
- Phone and Linux machine must reach the laptop broker over the network. USB installation does not route MQTT traffic.

Live capture in this build is Android-specific. The iOS app does not yet have a native sensor capture implementation and displays that limitation.

## ROS topics

All rows use prefix `/ros2embedded/domain_7/connection_phone_5f1` and appear only when the corresponding source is enabled and accepted.

| Relative topic | ROS type | Measurement |
|---|---|---|
| `accelerometer` | sensor_msgs/msg/Imu | Linear acceleration in m/s², including gravity; orientation and angular velocity unavailable |
| `gyroscope` | sensor_msgs/msg/Imu | Angular velocity in rad/s; orientation and linear acceleration unavailable |
| `magnetometer` | sensor_msgs/msg/MagneticField | Tesla (converted from Android microtesla) |
| `location` | sensor_msgs/msg/NavSatFix | Degrees; available altitude is WGS84 ellipsoid height |
| `light` | sensor_msgs/msg/Illuminance | Lux |
| `barometer` | sensor_msgs/msg/FluidPressure | Pascal (converted from Android hPa) |
| `camera/image/compressed` | sensor_msgs/msg/CompressedImage | JPEG, default 2 FPS, image up to 640 × 640 |
| `battery` | sensor_msgs/msg/BatteryState | Fraction 0–1, voltage, temperature, charging state; unavailable fields are NaN |
| `microphone` | std_msgs/msg/String | JSON RMS normalized to full scale and dBFS, not calibrated sound-pressure level |
| `touch` | std_msgs/msg/String | JSON normalized x/y and touch phase from the app's touch pad |

Motion/environment sensors are sampled up to roughly 10 Hz, location about 1 Hz when providers deliver fixes, battery on Android battery updates. Frames are JPEG quality 55 and capped at 256 KiB. Telemetry uses MQTT QoS 0 without retain to avoid retransmitting stale frames; control snapshots use QoS 1 retained. Sensor ROS publishers use best-effort, volatile, depth 2. std_msgs JSON outputs retain their existing reliable QoS.

Sensor headers preserve acquisition time (camera uses exposure time when Android exposes a realtime sensor clock, otherwise callback time) and `phone_link` (Android natural-orientation axes: x right, y up, z out of the screen). Camera uses `phone_camera_optical` with image optical axes. No TF between those frames, robot frames or the world is invented. IMU orientation is not estimated; covariance marks unmeasured IMU components with -1. No camera calibration/CameraInfo is supplied.

## Check the data

In another Linux terminal, source the same workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list -t
ros2 topic echo /ros2embedded/domain_7/connection_phone_5f1/accelerometer --qos-reliability best_effort
# Stop echo with Ctrl-C, then inspect another sensor:
ros2 topic echo /ros2embedded/domain_7/connection_phone_5f1/location --qos-reliability best_effort
ros2 topic hz /ros2embedded/domain_7/connection_phone_5f1/camera/image/compressed
```

Some Humble ros2cli versions have QoS limitations in `topic hz`; use `topic echo --qos-reliability best_effort --field header --once` to verify camera headers if hz reports no samples.

## RViz camera

Launch `rviz2`. Add an **Image** display, choose the base image topic:

```text
/ros2embedded/domain_7/connection_phone_5f1/camera/image
```

Choose **compressed** under Transport Hint and **Best Effort** reliability under the topic's QoS options. The actual wire topic ends in `/compressed`; image_transport subscribes to that suffix automatically. The Image display does not need CameraInfo or TF. Use Image rather than the calibrated 3D Camera display.

If compressed does not appear, verify `ros-humble-compressed-image-transport` is installed, the camera topic exists with type `sensor_msgs/msg/CompressedImage`, and restart RViz. As a fallback, run:

```bash
ros2 run image_transport republish compressed raw --ros-args \
  -r in/compressed:=/ros2embedded/domain_7/connection_phone_5f1/camera/image/compressed \
  -r out:=/rosnect/camera/image_raw
```

Then use an RViz Image display with `/rosnect/camera/image_raw`, transport raw. Other sensor values are available through echo/plotting; RViz does not provide a built-in display for every scalar, GPS, or battery measurement. A 3D phone pose needs a separate orientation estimator and valid TF.

## Generic structured JSON contract

These mappings are explicit, optional binding schemas. Use `value_type=JSON`, `direction=DEVICE_TO_ROS` and `ros_message_type` equal to one of the seven sensor_msgs types above. Other JSON still maps to std_msgs/String. Sensor schema command/bidirectional bindings are rejected. Existing primitive mappings, protobuf enums, serial framing and identifiers are unchanged.

The envelope's `value` is a ROS-shaped object with optional `header` (`stamp.sec`, `stamp.nanosec`, `frame_id`). A missing header stamp uses the envelope timestamp. CompressedImage supports `format=jpeg|png` and `data_base64` (strict base64) or `data` (0–255 byte array). Image content is not decoded by the bridge. Missing measurements are marked unknown rather than fabricated; malformed schemas/ranges are dropped.

JSON values are bounded at 512 KiB and complete wire messages at 1 MiB. Raw BYTES/STRING remain capped at 4096 bytes; serial remains 256 bytes. The MQTT queue remains count-bounded at 128 events (up to about 64 MiB of JSON values at the upper limit). These limits permit low-rate snapshots; this does not provide high-frame-rate video transport.

## Validation

Flutter tests cover handshake correlation/coalescing/rejection, existing connection behavior, SI conversions, unmeasured IMU fields and JPEG JSON encoding. The APK compiles on the Mac. A host-side codec check exercises a 300 KB JSON camera envelope through JSON/protobuf and rejects oversized payloads.

ROS C++ sensor decoder tests and a full MQTT→sensor_msgs integration test are included for Linux:

```bash
colcon test --base-paths ros2
colcon test-result --verbose
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173 python3 -m unittest discover -s tests -v
```

Those ROS tests and actual RViz display must be verified on the Linux ROS machine; ROS is not installed on the development Mac.

Reference documentation: [Android sensor coordinates](https://developer.android.com/develop/sensors-and-location/sensors/sensors_overview), [ROS CompressedImage definition](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CompressedImage.msg), and [compressed image transport](https://index.ros.org/p/compressed_image_transport/).
