#!/usr/bin/env python3
"""Real broker + ROS + PTY integration. Source the installed workspace first.

Requires mosquitto and python3-paho-mqtt. This test starts only local processes,
uses temporary files/ports, and never connects to a user's configured broker.
Run: python3 tests/test_pipeline.py
"""
import json
import os
import pathlib
import pty
import select
import signal
import shutil
import socket
import subprocess
import tempfile
import time
import unittest

import yaml

import paho.mqtt.client as mqtt
import rclpy
from ament_index_python.packages import get_package_prefix
from std_msgs.msg import Float32, Int32, String

ROOT = pathlib.Path(__file__).resolve().parents[1]


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class Pipeline(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="ros2embedded-pipeline-")
        self.addCleanup(self.temp.cleanup)
        self.path = pathlib.Path(self.temp.name)
        self.processes = []
        self.logs = []
        self.addCleanup(self.stop_processes)
        self.port = free_port()
        broker_config = self.path / "mosquitto.conf"
        broker_config.write_text(f"listener {self.port} 127.0.0.1\nallow_anonymous true\npersistence false\n")
        self.broker = self.spawn([shutil.which("mosquitto") or "/usr/sbin/mosquitto", "-c", str(broker_config)], "broker")
        self.wait_port()
        rclpy.init()
        self.node = rclpy.create_node("pipeline_test")
        self.addCleanup(self.shutdown_ros)
        self.observed = {"phone7": [], "phone8": [], "serial": [], "proto": []}
        self.subscriptions = []
        for key, topic, msg_type in [
            ("phone7", "/ros2embedded/domain_7/connection_phone_5f1/accelerometer", String),
            ("phone8", "/ros2embedded/domain_8/connection_phone_5f1/accelerometer", String),
            ("serial", "/ros2embedded/domain_7/connection_esp32_5flab_5f1/temperature", Float32),
            ("proto", "/ros2embedded/domain_7/connection_device/temperature", Float32),
        ]:
            self.subscriptions.append(self.node.create_subscription(msg_type, topic,
                lambda msg, key=key: self.observed[key].append(msg.data), 10))
        self.phone_command = self.node.create_publisher(String,
            "/ros2embedded/domain_7/connection_phone_5f1/robot_status", 10)
        self.serial_command = self.node.create_publisher(Int32,
            "/ros2embedded/domain_7/connection_esp32_5flab_5f1/motor_target", 10)
        self.proto_command = self.node.create_publisher(Int32,
            "/ros2embedded/domain_7/connection_device/motor_target", 10)
        self.master, slave = pty.openpty()
        self.addCleanup(os.close, self.master)
        serial_path = os.ttyname(slave)
        self.addCleanup(os.close, slave)
        self.bridge("phone7", ROOT / "ros2/bridge_node/config/phone_mqtt.json", "phone_1", 7, "mqtt")
        config = json.loads((ROOT / "ros2/bridge_node/config/phone_mqtt.json").read_text())
        config["domain_id"] = 8
        config_path = self.path / "phone8.json"
        config_path.write_text(json.dumps(config))
        self.bridge("phone8", config_path, "phone_1", 8, "mqtt")
        self.bridge("serial", ROOT / "ros2/bridge_node/config/arduino_serial.json", "esp32_lab_1", 7,
                    "serial", serial_port=serial_path)
        config_path = self.path / "device.json"
        config_path.write_text(json.dumps({"connection_id": "device", "domain_id": 7, "protocol": "PROTOCOL_MQTT",
            "bindings": [
                {"identifier": "temperature", "ros_topic": "temperature", "direction": "DEVICE_TO_ROS", "value_type": "FLOAT32"},
                {"identifier": "motor.target", "ros_topic": "motor_target", "direction": "ROS_TO_DEVICE", "value_type": "INT32"}]}))
        self.bridge("proto", config_path, "device", 7, "mqtt", payload_encoding="protobuf")
        # Generate the simulated device's bindings from the actual contract.
        subprocess.run(["protoc", f"-I{ROOT / 'proto'}", f"--python_out={self.path}",
                        str(ROOT / "proto/bridge_payload.proto")], check=True)
        import importlib.util
        spec = importlib.util.spec_from_file_location("pipeline_payload", self.path / "bridge_payload_pb2.py")
        self.proto = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.proto)
        self.incoming = []
        # Use Flutter's actual client ID to catch host/device MQTT ID collisions.
        self.peer = mqtt.Client(client_id="phone_1", protocol=mqtt.MQTTv311)
        self.peer.on_connect = lambda client, userdata, flags, rc: client.subscribe("ros2embedded/+/+/to_device", qos=1)
        self.peer.on_message = lambda client, userdata, msg: self.incoming.append((msg.topic, msg.payload))
        self.peer.connect("127.0.0.1", self.port, 15)
        self.peer.subscribe("ros2embedded/+/+/to_device", qos=1)
        self.peer.loop_start()
        self.addCleanup(self.stop_peer)
        self.spin_until(lambda: self.phone_command.get_subscription_count() == 1 and
                        self.serial_command.get_subscription_count() == 1 and
                        self.proto_command.get_subscription_count() == 1)
        self.pump(1.5)  # MQTT CONNACK/SUBACK and DDS discovery.

    def spawn(self, args, name):
        log = open(self.path / f"{name}.log", "w+")
        self.logs.append(log)
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        self.processes.append(process)
        return process

    def bridge(self, name, config, connection, domain, protocol, **extra):
        executable = pathlib.Path(get_package_prefix("bridge_node")) / "lib/bridge_node/bridge_node"
        values = dict(config_file=str(config), connection_id=connection, domain_id=domain,
                      protocol=protocol, mqtt_broker="127.0.0.1", mqtt_port=self.port)
        values.update(extra)
        args = [str(executable), "--ros-args", "-r", f"__node:=test_{name}"]
        for key, value in values.items():
            args.extend(["-p", f"{key}:={value}"])
        return self.spawn(args, name)

    def wait_port(self):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            try:
                with socket.create_connection(("127.0.0.1", self.port), timeout=.2):
                    return
            except OSError:
                time.sleep(.05)
        logs = "\n".join(p.read_text() for p in self.path.glob("*.log"))
        self.fail("Broker did not start:\n" + logs)

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

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)

    def spin_until(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.pump(.05)
        logs = "\n".join(p.read_text() for p in self.path.glob("*.log"))
        self.fail("Pipeline timed out:\n" + logs)

    def publish_phone(self, identifier, value, *, domain=7, topic_domain=7, value_type="json"):
        self.peer.publish(f"ros2embedded/{topic_domain}/phone_1/to_ros", json.dumps({
            "identifier": identifier, "domain_id": domain, "value_type": value_type,
            "value": value, "timestamp_unix_ms": 1700000000000}), qos=1)

    def test_bidirectional_pipeline_filtering_isolation_and_reconnect(self):
        # Flutter-compatible structured values, wrong domain/type/id, malformed JSON.
        self.publish_phone("phone.accelerometer", {"x": 1.25})
        self.spin_until(lambda: len(self.observed["phone7"]) == 1)
        self.assertEqual(json.loads(self.observed["phone7"][0]), {"x": 1.25})
        self.assertEqual(self.observed["phone8"], [])
        self.publish_phone("phone.accelerometer", {"x": 99}, domain=8)
        self.publish_phone("phone.accelerometer", "wrong type", value_type="string")
        self.publish_phone("unknown", {"x": 99})
        self.peer.publish("ros2embedded/7/phone_1/to_ros", "malformed")
        self.pump(.3)
        self.assertEqual(len(self.observed["phone7"]), 1)
        self.publish_phone("phone.accelerometer", {"x": 8}, domain=8, topic_domain=8)
        self.spin_until(lambda: len(self.observed["phone8"]) == 1)
        self.assertEqual(len(self.observed["phone7"]), 1)
        self.phone_command.publish(String(data="ready"))
        self.spin_until(lambda: any(t == "ros2embedded/7/phone_1/to_device" for t, _ in self.incoming))
        reply = next(json.loads(data) for topic, data in self.incoming if topic == "ros2embedded/7/phone_1/to_device")
        self.assertEqual((reply["identifier"], reply["value"], reply["value_type"]), ("robot.status", "ready", "string"))
        self.assertFalse(any(t.startswith("ros2embedded/8/") for t, _ in self.incoming))
        # Real PTY exercises termios, fragmented frames, resynchronization, and ROS routing.
        os.write(self.master, b"noise\n" + b"x" * 700 + b"\nR2E,8,6,sensor.temperature_c,0000C441\nR2E,7,6,sensor.temperature_c,0000")
        self.pump(.1)
        self.assertEqual(self.observed["serial"], [])
        os.write(self.master, b"C441\n")
        self.spin_until(lambda: len(self.observed["serial"]) == 1)
        self.assertEqual(self.observed["serial"], [24.5])
        self.serial_command.publish(Int32(data=-42))
        self.assertTrue(select.select([self.master], [], [], 5)[0], "No serial command")
        self.assertEqual(os.read(self.master, 1024), b"R2E,7,2,motor.target,D6FFFFFF\n")
        # A non-Flutter device uses protobuf on exactly the same MQTT transport.
        self.peer.publish("ros2embedded/7/device/to_ros", self.proto.BridgePayload(
            identifier="temperature", domain_id=7, float32_value=19.5).SerializeToString(), qos=1)
        self.spin_until(lambda: self.observed["proto"] == [19.5])
        self.proto_command.publish(Int32(data=42))
        self.spin_until(lambda: any(t == "ros2embedded/7/device/to_device" for t, _ in self.incoming))
        wire = next(data for topic, data in self.incoming if topic == "ros2embedded/7/device/to_device")
        message = self.proto.BridgePayload.FromString(wire)
        self.assertEqual((message.identifier, message.domain_id, message.int32_value), ("motor.target", 7, 42))
        # Restart the broker; host reconnects/resubscribes without restarting ROS nodes.
        self.broker.terminate()
        self.broker.wait(5)
        self.pump(.3)
        self.broker = self.spawn([shutil.which("mosquitto") or "/usr/sbin/mosquitto", "-c", str(self.path / "mosquitto.conf")], "broker-restarted")
        self.wait_port()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and len(self.observed["phone7"]) < 2:
            self.publish_phone("phone.accelerometer", {"reconnected": True})
            self.pump(.4)
        self.assertGreaterEqual(len(self.observed["phone7"]), 2)
        self.assertTrue(all(p.poll() is None for p in self.processes[1:5]))
        self.assertTrue(self.peer.is_connected(), "Host evicted phone MQTT client ID")
        self.check_launch_and_manager()

    def check_launch_and_manager(self):
        # Exercise the public launch path with typed/default parameters.
        config = {"connection_id": "launched", "domain_id": 7, "protocol": "PROTOCOL_MQTT",
            "bindings": [{"identifier": "value", "ros_topic": "value", "direction": "DEVICE_TO_ROS", "value_type": "INT32"}]}
        config_path = self.path / "launched.json"
        config_path.write_text(json.dumps(config))
        observed, statuses = [], []
        self.subscriptions.append(self.node.create_subscription(Int32,
            "/ros2embedded/domain_7/connection_launched/value", lambda msg: observed.append(msg.data), 10))
        self.subscriptions.append(self.node.create_subscription(String,
            "/ros2embedded/domain_7/connection_launched/status", lambda msg: statuses.append(msg.data), 10))
        launch = self.spawn(["ros2", "launch", "bridge_node", "bridge.launch.py", "connection_id:=launched",
            "domain_id:=7", "protocol:=mqtt", f"config_file:={config_path}",
            "mqtt_broker:=127.0.0.1", f"mqtt_port:={self.port}"], "launch")
        self.spin_until(lambda: "connected" in statuses)
        self.peer.publish("ros2embedded/7/launched/to_ros", json.dumps({"identifier": "value", "domain_id": 7,
            "value_type": "int32", "value": 42}), qos=1)
        self.spin_until(lambda: observed == [42])
        launch.send_signal(signal.SIGINT)
        self.assertEqual(launch.wait(8), 0)
        # Manager owns a direct bridge child; shutdown must reap it and its private parameters.
        config["connection_id"] = "managed"
        config_path = self.path / "managed.json"
        config_path.write_text(json.dumps(config))
        params = dict(auto_start=True, connection_id="managed", domain_id=7, protocol="mqtt",
            config_file=str(config_path), mqtt_broker="127.0.0.1", mqtt_port=self.port)
        params_path = self.path / "manager.yaml"
        params_path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
        executable = pathlib.Path(get_package_prefix("bridge_manager")) / "lib/bridge_manager/bridge_manager"
        manager = self.spawn([str(executable), "--ros-args", "--params-file", str(params_path)], "manager")
        children_path = pathlib.Path(f"/proc/{manager.pid}/task/{manager.pid}/children")
        self.spin_until(lambda: manager.poll() is None and children_path.exists() and children_path.read_text().strip())
        child = int(children_path.read_text().split()[0])
        command_path = pathlib.Path(f"/proc/{child}/cmdline")
        self.spin_until(lambda: command_path.exists() and b"--params-file" in command_path.read_bytes())
        args = command_path.read_bytes().split(b"\0")
        child_params = pathlib.Path(os.fsdecode(args[args.index(b"--params-file") + 1]))
        self.assertTrue(child_params.exists())
        manager.terminate()
        self.assertEqual(manager.wait(8), 0)
        self.assertFalse(pathlib.Path(f"/proc/{child}").exists(), "Manager left its bridge child running")
        self.assertFalse(child_params.exists(), "Manager left its credential parameter file")


class StartupValidation(unittest.TestCase):
    def test_invalid_configs_fail_before_transport_start(self):
        executable = pathlib.Path(get_package_prefix("bridge_node")) / "lib/bridge_node/bridge_node"
        base = {"connection_id": "controller", "domain_id": 7, "protocol": "PROTOCOL_MQTT",
                "bindings": [{"identifier": "value", "ros_topic": "value", "direction": "DEVICE_TO_ROS", "value_type": "INT32"}]}
        cases = []
        def changed(mutator):
            config = json.loads(json.dumps(base))
            mutator(config)
            return config
        cases.extend([
            (base, {"domain_id": 4294967296}), (base, {"domain_id": -1}),
            (changed(lambda c: c.update(domain_id=8)), {}),
            (changed(lambda c: c["bindings"].append(dict(c["bindings"][0]))), {}),
            (changed(lambda c: c["bindings"][0].update(identifier="bad,id")), {}),
            (changed(lambda c: c["bindings"][0].update(ros_message_type="std_msgs/msg/String")), {}),
            (changed(lambda c: c["bindings"][0].update(direction="BIDIRECTIONAL")), {}),
            (changed(lambda c: c["bindings"][0].update(direction="BIDIRECTIONAL", ros_command_topic="value")), {}),
            (changed(lambda c: c["bindings"][0].update(value_type="VALUE_TYPE_UNSPECIFIED")), {}),
            (base, {"protocol": "redis"}),
        ])
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "invalid.json"
            for config, overrides in cases:
                with self.subTest(config=config, overrides=overrides):
                    path.write_text(json.dumps(config))
                    params = dict(connection_id="controller", domain_id=7, protocol="mqtt", config_file=str(path))
                    params.update(overrides)
                    args = [str(executable), "--ros-args"]
                    for key, value in params.items():
                        args += ["-p", f"{key}:={value}"]
                    result = subprocess.run(args, capture_output=True, text=True, timeout=5)
                    self.assertEqual(result.returncode, 1, result.stdout + result.stderr)



if __name__ == "__main__":
    unittest.main()
