#!/usr/bin/env python3
"""Device-advertised interfaces: JSON/protobuf, live replacement, retained bootstrap."""
import base64
import math
import importlib.util
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import threading
import time
import unittest

import paho.mqtt.client as mqtt
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from std_msgs.msg import Bool, Float32, Int32, String
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, Illuminance, FluidPressure, BatteryState, CompressedImage
from rclpy.qos import qos_profile_sensor_data

ROOT = Path(__file__).resolve().parents[1]


class DeviceConfiguration(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ros2embedded-device-config-")
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.processes, self.logs = [], []
        self.addCleanup(self.stop_processes)
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            self.port = sock.getsockname()[1]
        broker_config = self.path / "broker.conf"
        broker_config.write_text(f"listener {self.port} 127.0.0.1\nallow_anonymous true\npersistence false\n")
        self.spawn([shutil.which("mosquitto") or "/usr/sbin/mosquitto", "-c", str(broker_config)], "broker")
        deadline = time.monotonic() + 5
        while True:
            try:
                with socket.create_connection(("127.0.0.1", self.port), timeout=.2):
                    break
            except OSError:
                if time.monotonic() >= deadline:
                    self.fail("Broker did not start:\n" + self.read_logs())
                time.sleep(.05)
        rclpy.init()
        self.node = rclpy.create_node("device_configuration_test")
        self.addCleanup(self.shutdown_ros)
        self.prefix = "/ros2embedded/domain_7/connection_phone_5f1"
        self.mqtt_prefix = "ros2embedded/7/phone_1"
        self.incoming, self.statuses, self.interface_statuses = [], [], []
        self.subscriptions = [
            self.node.create_subscription(String, self.prefix + "/status", lambda msg: self.statuses.append(msg.data), 10),
            self.node.create_subscription(String, self.prefix + "/interface_status", lambda msg: self.interface_statuses.append(json.loads(msg.data)), 10)]
        self.peer = mqtt.Client(client_id="phone_1", protocol=mqtt.MQTTv311)
        subscribed = threading.Event()
        self.peer.on_connect = lambda client, userdata, flags, rc: client.subscribe(self.mqtt_prefix + "/#", qos=1)
        self.peer.on_subscribe = lambda *args: subscribed.set()
        self.peer.on_message = lambda client, userdata, msg: self.incoming.append((msg.topic, bytes(msg.payload)))
        self.peer.connect("127.0.0.1", self.port, 15)
        self.peer.loop_start()
        self.addCleanup(self.stop_peer)
        self.assertTrue(subscribed.wait(5))

    def spawn(self, args, name):
        stream = open(self.path / f"{name}-{len(self.logs)}.log", "w+")
        self.logs.append(stream)
        process = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT)
        self.processes.append(process)
        return process

    def start_bridge(self, *, manager=False, **extra):
        params = dict(connection_id="phone_1", domain_id=7, protocol="mqtt", mqtt_broker="127.0.0.1", mqtt_port=self.port)
        params.update(extra)
        package = "bridge_manager" if manager else "bridge_node"
        executable = Path(get_package_prefix(package)) / f"lib/{package}/{package}"
        if manager:
            params["auto_start"] = True
        params_file = self.path / f"params-{len(self.logs)}.yaml"
        params_file.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
        self.bridge = self.spawn([str(executable), "--ros-args", "--params-file", str(params_file)], "bridge")
        return self.bridge

    def snapshot(self, config_id, bindings):
        return dict(schema_version=1, configuration_id=config_id, connection_id="phone_1", domain_id=7,
                    protocol="PROTOCOL_MQTT", bindings=bindings)

    def results(self):
        return [json.loads(wire) for topic, wire in self.incoming if topic == self.mqtt_prefix + "/config_status"]

    def publish_config(self, config, *, retain=True):
        previous = len(self.results())
        self.peer.publish(self.mqtt_prefix + "/config", json.dumps(config), qos=1, retain=retain).wait_for_publish()
        self.wait(lambda: any(p["configuration_id"] == config["configuration_id"] for p in self.results()[previous:]))
        return next(p for p in self.results()[previous:] if p["configuration_id"] == config["configuration_id"])

    def publish_data(self, identifier, value, value_type="json"):
        self.peer.publish(self.mqtt_prefix + "/to_ros", json.dumps(dict(identifier=identifier, domain_id=7,
            value_type=value_type, value=value)), qos=1)

    def test_json_live_interfaces_validation_replacement_and_retained_restart(self):
        self.start_bridge()  # No config_file: automatic MQTT mode waits for the device.
        self.wait(lambda: "connected" in self.statuses and
                  any(topic == self.mqtt_prefix + "/config_request" for topic, _ in self.incoming))
        requests = [json.loads(wire) for topic, wire in self.incoming if topic.endswith("/config_request")]
        self.assertEqual(requests[-1]["schema_version"], 1)
        self.assertEqual(requests[-1]["config_encoding"], "json")
        self.assertEqual(requests[-1]["payload_encoding"], "json")
        self.assertTrue(all(not self.node.count_publishers(self.prefix + "/" + name)
                            for name in ("accelerometer", "test", "robot_status")))
        self.publish_data("phone.gyroscope", {"x": 100})
        self.pump(.1)
        self.assertFalse(self.node.count_publishers(self.prefix + "/data/id_phone_2egyroscope"))
        first = self.snapshot("snapshot-1", [
            dict(identifier="phone.gyroscope", direction="DEVICE_TO_ROS", value_type="JSON"),
            dict(identifier="robot.status", direction="ROS_TO_DEVICE", value_type="STRING"),
            dict(identifier="robot.enabled", direction="BIDIRECTIONAL", value_type="BOOL", ros_topic="enabled_state")])
        result = self.publish_config(first)
        self.assertTrue(result["accepted"], result)
        self.assertEqual(result["state"], "ready")
        self.assertEqual(result["bindings_count"], 3)
        self.assertEqual(result["ros_namespace"], self.prefix)
        self.assertEqual(result["bindings"][0]["ros_topic"], "data/id_phone_2egyroscope")
        self.assertEqual(result["bindings"][0]["ros_message_type"], "std_msgs/msg/String")
        self.assertEqual(result["bindings"][2]["ros_command_topic"], "commands/id_robot_2eenabled")
        gyro, enabled = [], []
        self.subscriptions.extend([
            self.node.create_subscription(String, self.prefix + "/data/id_phone_2egyroscope", lambda msg: gyro.append(json.loads(msg.data)), 10),
            self.node.create_subscription(Bool, self.prefix + "/enabled_state", lambda msg: enabled.append(msg.data), 10)])
        status_command = self.node.create_publisher(String, self.prefix + "/commands/id_robot_2estatus", 10)
        enabled_command = self.node.create_publisher(Bool, self.prefix + "/commands/id_robot_2eenabled", 10)
        self.wait(lambda: status_command.get_subscription_count() == 1 and enabled_command.get_subscription_count() == 1)
        self.pump(.3)
        self.publish_data("phone.gyroscope", {"x": 1.25})
        self.publish_data("robot.enabled", True, "boolValue")
        self.wait(lambda: gyro == [{"x": 1.25}] and enabled == [True])
        status_command.publish(String(data="automatic-ready"))
        enabled_command.publish(Bool(data=False))
        def command_values():
            return [json.loads(wire) for topic, wire in self.incoming if topic.endswith("/to_device")]
        self.wait(lambda: len(command_values()) >= 2)
        self.assertTrue(any(p["identifier"] == "robot.status" and p["value"] == "automatic-ready" for p in command_values()))
        self.assertTrue(any(p["identifier"] == "robot.enabled" and p["value"] is False for p in command_values()))
        self.assertTrue(self.publish_config(first)["accepted"])  # QoS retry is idempotent.
        self.assertEqual(self.node.count_publishers(self.prefix + "/data/id_phone_2egyroscope"), 1)
        # Invalid snapshots preserve every existing publisher/subscriber.
        cases = [
            lambda c: c.update(domain_id=8), lambda c: c.update(connection_id="another"),
            lambda c: c.update(protocol="PROTOCOL_SERIAL"), lambda c: c.update(schema_version=2),
            lambda c: c["bindings"].append(dict(c["bindings"][0])),
            lambda c: c["bindings"][0].update(ros_topic="/outside"),
            lambda c: c["bindings"][0].update(ros_topic="~private"),
            lambda c: c["bindings"][0].update(ros_topic="bad topic"),
            lambda c: c["bindings"][0].update(ros_topic="status"),
            lambda c: c["bindings"][0].update(ros_topic="interface_status"),
            lambda c: c["bindings"][0].update(ros_message_type="std_msgs/msg/Float32"),
            lambda c: c["bindings"][2].update(ros_command_topic="enabled_state"),
        ]
        for index, mutate in enumerate(cases):
            with self.subTest(index=index):
                config = json.loads(json.dumps(first))
                config["configuration_id"] = f"invalid-{index}"
                mutate(config)
                result = self.publish_config(config, retain=False)
                self.assertFalse(result["accepted"], result)
                self.assertEqual(result["active_configuration_id"], "snapshot-1")
                self.assertEqual(result["state"], "ready")
                self.assertEqual(self.node.count_publishers(self.prefix + "/data/id_phone_2egyroscope"), 1)
                self.assertEqual(status_command.get_subscription_count(), 1)
        changed_same_id = json.loads(json.dumps(first))
        changed_same_id["bindings"][0]["ros_topic"] = "changed"
        self.assertFalse(self.publish_config(changed_same_id, retain=False)["accepted"])
        previous = len(self.results())
        self.peer.publish(self.mqtt_prefix + "/config", "not-json", qos=1)
        self.wait(lambda: len(self.results()) > previous)
        self.assertFalse(self.results()[-1]["accepted"])
        self.assertEqual(self.results()[-1]["active_configuration_id"], "snapshot-1")
        second = self.snapshot("snapshot-2", [
            dict(identifier="phone.battery", ros_topic="battery", direction="DEVICE_TO_ROS", value_type="JSON"),
            dict(identifier="robot.status", ros_topic="robot_status", direction="ROS_TO_DEVICE", value_type="STRING")])
        self.assertTrue(self.publish_config(second)["accepted"])
        self.wait(lambda: self.node.count_publishers(self.prefix + "/data/id_phone_2egyroscope") == 0 and
                  status_command.get_subscription_count() == 0 and enabled_command.get_subscription_count() == 0)
        battery = []
        self.subscriptions.append(self.node.create_subscription(String, self.prefix + "/battery", lambda msg: battery.append(json.loads(msg.data)), 10))
        self.pump(.3)
        self.publish_data("phone.battery", {"level": 80})
        self.publish_data("phone.gyroscope", {"x": 999})
        self.wait(lambda: battery == [{"level": 80}])
        self.assertEqual(gyro, [{"x": 1.25}])
        self.assertFalse(self.publish_config(first, retain=False)["accepted"])  # Superseded QoS replay.
        # Recreate the host: the retained snapshot rebuilds interfaces without a file or a new app publish.
        self.bridge.terminate()
        self.assertEqual(self.bridge.wait(5), 0)
        self.wait(lambda: self.node.count_publishers(self.prefix + "/battery") == 0)
        previous = len(self.results())
        self.start_bridge()
        self.wait(lambda: any(p["accepted"] and p["configuration_id"] == "snapshot-2" for p in self.results()[previous:]))
        self.wait(lambda: self.node.count_publishers(self.prefix + "/battery") == 1)
        third = self.snapshot("snapshot-3", [])
        result = self.publish_config(third)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["bindings_count"], 0)
        self.wait(lambda: self.node.count_publishers(self.prefix + "/battery") == 0)
        self.publish_data("phone.battery", {"level": 81})
        self.pump(.1)
        self.assertEqual(battery, [{"level": 80}])
        # The handoff tool uses exactly the device exchange (not a host config file).
        demo = self.snapshot("tool-1", [dict(identifier="phone.touch", direction="DEVICE_TO_ROS", value_type="JSON")])
        demo_path = self.path / "demo.json"
        demo_path.write_text(json.dumps(demo))
        tool = ROOT / "tools/publish_device_config.py"
        completed = subprocess.run(["/usr/bin/python3", str(tool), str(demo_path), "--broker", "127.0.0.1",
                                    "--port", str(self.port)], capture_output=True, text=True, timeout=15)
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertEqual(json.loads(completed.stdout)["active_configuration_id"], "tool-1")
        self.wait(lambda: self.node.count_publishers(self.prefix + "/data/id_phone_2etouch") == 1)

    def test_native_sensor_types_match_rosnect_fields_and_support_both_directions(self):
        self.start_bridge()
        self.wait(lambda: "connected" in self.statuses)
        header = {"stamp": {"sec": 1700000000, "nanosec": 123000000}, "frame_id": "phone_link"}
        unavailable = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
        image_bytes = bytes(range(256)) * 300  # Base64 envelope exceeds the old 64 KiB wire limit.
        cases = [
            ("phone.accelerometer", "accelerometer", Imu, "sensor_msgs/msg/Imu", {
                "header": header, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "orientation_covariance": unavailable,
                "linear_acceleration": {"x": 1.25, "y": -2, "z": 9.81}, "angular_velocity_covariance": unavailable}),
            ("phone.gyroscope", "gyroscope", Imu, "sensor_msgs/msg/Imu", {
                "header": header, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, "orientation_covariance": unavailable,
                "angular_velocity": {"x": .1, "y": -.2, "z": .3}, "linear_acceleration_covariance": unavailable}),
            ("phone.magnetometer", "magnetometer", MagneticField, "sensor_msgs/msg/MagneticField", {
                "header": header, "magnetic_field": {"x": 45e-6, "y": -12e-6, "z": 0}}),
            ("phone.location", "location", NavSatFix, "sensor_msgs/msg/NavSatFix", {
                "header": header, "status": {"status": 0, "service": 1}, "latitude": 52.5, "longitude": 13.4,
                "position_covariance": [4, 0, 0, 0, 4, 0, 0, 0, 0], "position_covariance_type": 0}),
            ("phone.light", "light", Illuminance, "sensor_msgs/msg/Illuminance", {"header": header, "illuminance": 123.5, "variance": 0}),
            ("phone.pressure", "barometer", FluidPressure, "sensor_msgs/msg/FluidPressure", {"header": header, "fluid_pressure": 101325, "variance": 0}),
            ("phone.battery", "battery", BatteryState, "sensor_msgs/msg/BatteryState", {
                "header": header, "voltage": 3.9, "temperature": 25.5, "percentage": .75, "present": True, "power_supply_status": 2}),
            ("phone.camera.frame", "camera/image/compressed", CompressedImage, "sensor_msgs/msg/CompressedImage", {
                "header": dict(header, frame_id="phone_camera_optical"), "format": "jpeg", "data_base64": base64.b64encode(image_bytes).decode()}),
            ("phone.touch", "touch", String, "std_msgs/msg/String", {"header": header, "x": .25, "y": .5, "phase": "move"}),
        ]
        bindings = [dict(identifier=identifier, ros_topic=topic, direction="DEVICE_TO_ROS", value_type="JSON", ros_message_type=ros_type)
                    for identifier, topic, cls, ros_type, value in cases]
        bindings.extend([
            dict(identifier="robot.imu", ros_topic="imu_input", direction="ROS_TO_DEVICE", value_type="JSON", ros_message_type="sensor_msgs/msg/Imu"),
            dict(identifier="robot.camera", ros_topic="camera_input", direction="ROS_TO_DEVICE", value_type="JSON", ros_message_type="sensor_msgs/msg/CompressedImage"),
        ])
        config = self.snapshot("native-1", bindings)
        result = self.publish_config(config)
        self.assertTrue(result["accepted"], result)
        self.assertEqual(result["bindings"][0]["ros_message_type"], "sensor_msgs/msg/Imu")
        received = {identifier: [] for identifier, *_ in cases}
        for identifier, topic, cls, ros_type, value in cases:
            self.subscriptions.append(self.node.create_subscription(cls, self.prefix + "/" + topic,
                lambda msg, identifier=identifier: received[identifier].append(msg), 10))
        imu_input = self.node.create_publisher(Imu, self.prefix + "/imu_input", qos_profile_sensor_data)
        image_input = self.node.create_publisher(CompressedImage, self.prefix + "/camera_input", qos_profile_sensor_data)
        self.wait(lambda: imu_input.get_subscription_count() == 1 and image_input.get_subscription_count() == 1)
        self.pump(.3)
        for identifier, topic, cls, ros_type, value in cases:
            self.publish_data(identifier, value)
        self.wait(lambda: all(received.values()))
        accel = received["phone.accelerometer"][-1]
        self.assertEqual(accel.header.frame_id, "phone_link")
        self.assertEqual((accel.header.stamp.sec, accel.header.stamp.nanosec), (1700000000, 123000000))
        self.assertEqual(accel.linear_acceleration.z, 9.81)
        self.assertEqual(accel.angular_velocity_covariance[0], -1)
        self.assertEqual(accel.orientation_covariance[0], -1)
        self.assertEqual(received["phone.gyroscope"][-1].angular_velocity.y, -.2)
        self.assertEqual(received["phone.magnetometer"][-1].magnetic_field.x, 45e-6)
        self.assertTrue(math.isnan(received["phone.location"][-1].altitude))
        self.assertEqual(received["phone.light"][-1].illuminance, 123.5)
        self.assertEqual(received["phone.pressure"][-1].fluid_pressure, 101325)
        battery = received["phone.battery"][-1]
        self.assertAlmostEqual(battery.voltage, 3.9, places=5)
        self.assertEqual(battery.percentage, .75)
        self.assertTrue(math.isnan(battery.current))
        self.assertEqual(bytes(received["phone.camera.frame"][-1].data), image_bytes)
        self.assertEqual(received["phone.camera.frame"][-1].header.frame_id, "phone_camera_optical")
        self.assertEqual(json.loads(received["phone.touch"][-1].data)["phase"], "move")
        graph = dict(self.node.get_topic_names_and_types())
        for identifier, topic, cls, ros_type, value in cases:
            self.assertEqual(graph[self.prefix + "/" + topic], [ros_type])
        # Native ROS producers (including best-effort) become JSON device values.
        imu_input.publish(accel)
        image_input.publish(received["phone.camera.frame"][-1])
        def replies():
            return [json.loads(wire) for topic, wire in self.incoming if topic.endswith("/to_device")]
        self.wait(lambda: {p["identifier"] for p in replies()} >= {"robot.imu", "robot.camera"})
        outgoing_imu = next(p for p in replies() if p["identifier"] == "robot.imu")
        self.assertEqual(outgoing_imu["value_type"], "json")
        self.assertEqual(outgoing_imu["value"]["linear_acceleration"]["z"], 9.81)
        outgoing_camera = next(p for p in replies() if p["identifier"] == "robot.camera")
        self.assertEqual(base64.b64decode(outgoing_camera["value"]["data_base64"]), image_bytes)
        # Bad measurements are dropped without taking down valid configured interfaces.
        before = {key: len(value) for key, value in received.items()}
        for identifier, value in [
            ("phone.accelerometer", {"linear_acceleration": {"x": "wrong", "y": 0, "z": 0}}),
            ("phone.gyroscope", {"angular_velocity": {"x": 0, "y": 0, "z": 1}, "angular_velocity_covariance": [0]}),
            ("phone.location", {"latitude": 91, "longitude": 13}),
            ("phone.battery", {"voltage": 3.9, "percentage": 75}),
            ("phone.camera.frame", {"format": "jpeg", "data_base64": "A==="}),
        ]:
            self.publish_data(identifier, value)
        self.pump(.4)
        self.assertEqual(before, {key: len(value) for key, value in received.items()})
        self.publish_data("phone.accelerometer", {"linear_acceleration": {"x": 3, "y": 0, "z": 9.81}})
        self.wait(lambda: received["phone.accelerometer"][-1].linear_acceleration.x == 3)
        # Configuration limits remain 64 KiB, independently of larger image data.
        previous = len(self.results())
        huge = json.dumps(self.snapshot("oversized", [])) + " " * 65536
        self.peer.publish(self.mqtt_prefix + "/config", huge, qos=1).wait_for_publish()
        self.pump(.2)
        self.assertEqual(len(self.results()), previous)
        for index, (value_type, ros_type) in enumerate([
                ("FLOAT64", "sensor_msgs/msg/Imu"), ("JSON", "sensor_msgs/msg/PointCloud2")]):
            invalid = self.snapshot(f"bad-native-{index}", [dict(identifier="x", direction="DEVICE_TO_ROS", value_type=value_type, ros_message_type=ros_type)])
            self.assertFalse(self.publish_config(invalid, retain=False)["accepted"])
            self.assertEqual(self.node.count_publishers(self.prefix + "/accelerometer"), 1)

    def test_binary_config_and_data_bootstrap_through_manager_without_file(self):
        subprocess.run(["protoc", f"-I{ROOT / 'proto'}", f"--python_out={self.path}",
                        str(ROOT / "proto/bridge_config.proto"), str(ROOT / "proto/bridge_payload.proto")], check=True)
        modules = []
        for name in ("bridge_config", "bridge_payload"):
            spec = importlib.util.spec_from_file_location("device_" + name, self.path / f"{name}_pb2.py")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            modules.append(module)
        config_pb, payload_pb = modules
        config = config_pb.BridgeConfig(connection_id="phone_1", domain_id=7, protocol=config_pb.PROTOCOL_MQTT,
            configuration_id="protobuf-1", schema_version=1)
        config.bindings.add(identifier="temperature", ros_topic="temperature", direction=config_pb.DEVICE_TO_ROS, value_type=config_pb.FLOAT32)
        config.bindings.add(identifier="motor.target", ros_topic="command", direction=config_pb.ROS_TO_DEVICE, value_type=config_pb.INT32)
        # Device connects first and advertises a retained binary config.
        self.peer.publish(self.mqtt_prefix + "/config", config.SerializeToString(), qos=1, retain=True).wait_for_publish()
        self.start_bridge(manager=True, config_encoding="protobuf", payload_encoding="protobuf")
        self.wait(lambda: any(p["accepted"] and p["configuration_id"] == "protobuf-1" for p in self.results()))
        values = []
        self.subscriptions.append(self.node.create_subscription(Float32, self.prefix + "/temperature", lambda msg: values.append(msg.data), 10))
        command = self.node.create_publisher(Int32, self.prefix + "/command", 10)
        self.wait(lambda: command.get_subscription_count() == 1)
        self.pump(.3)
        self.peer.publish(self.mqtt_prefix + "/to_ros", payload_pb.BridgePayload(
            identifier="temperature", domain_id=7, float32_value=19.5).SerializeToString(), qos=1)
        self.wait(lambda: values == [19.5])
        command.publish(Int32(data=42))
        self.wait(lambda: any(topic.endswith("/to_device") for topic, _ in self.incoming))
        wire = next(wire for topic, wire in self.incoming if topic.endswith("/to_device"))
        reply = payload_pb.BridgePayload.FromString(wire)
        self.assertEqual((reply.identifier, reply.domain_id, reply.int32_value), ("motor.target", 7, 42))

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)

    def wait(self, predicate):
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            if predicate():
                return
            self.pump(.03)
        self.fail("Device configuration timed out:\n" + self.read_logs())

    def read_logs(self):
        return "\n".join(p.read_text() for p in self.path.glob("*.log"))

    def stop_peer(self):
        self.peer.disconnect()
        self.peer.loop_stop()

    def shutdown_ros(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def stop_processes(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        for stream in self.logs:
            stream.close()


if __name__ == "__main__":
    unittest.main()
