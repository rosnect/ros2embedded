#!/usr/bin/env python3
"""Local TLS/mTLS + password pipeline and certificate rejection checks.
Requires mosquitto (including mosquitto_passwd), openssl, python3-paho-mqtt.
Source the installed workspace, then run python3 tests/test_mqtt_tls.py -v.
"""
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import time
import unittest

import paho.mqtt.client as mqtt
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from std_msgs.msg import String


class MqttTls(unittest.TestCase):
    def test_verified_tls_client_certificate_password_and_rejection(self):
        with tempfile.TemporaryDirectory(prefix="ros2embedded-tls-") as directory:
            self.path = Path(directory)
            self.processes = []
            self.logs = []
            self.addCleanup(self.stop_processes)
            def openssl(*args):
                subprocess.run(["openssl", *args], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            for name in ("ca", "wrong-ca"):
                openssl("req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "1",
                        "-subj", f"/CN={name}", "-keyout", str(self.path / f"{name}.key"),
                        "-out", str(self.path / f"{name}.crt"))
            for name, usage in (("server", "serverAuth"), ("client", "clientAuth")):
                openssl("req", "-newkey", "rsa:2048", "-nodes", "-subj", f"/CN={name}",
                        "-keyout", str(self.path / f"{name}.key"), "-out", str(self.path / f"{name}.csr"))
                extension = self.path / f"{name}.ext"
                extension.write_text(f"extendedKeyUsage={usage}\n" + ("subjectAltName=DNS:localhost\n" if name == "server" else ""))
                openssl("x509", "-req", "-days", "1", "-in", str(self.path / f"{name}.csr"),
                        "-CA", str(self.path / "ca.crt"), "-CAkey", str(self.path / "ca.key"), "-CAcreateserial",
                        "-extfile", str(extension), "-out", str(self.path / f"{name}.crt"))
            password_file = self.path / "passwords"
            subprocess.run([shutil.which("mosquitto_passwd") or "/usr/bin/mosquitto_passwd",
                            "-b", "-c", str(password_file), "bridge", "local-test-password"], check=True)
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            broker_config = self.path / "broker.conf"
            broker_config.write_text(f"listener {port} 127.0.0.1\nallow_anonymous false\n"
                f"password_file {password_file}\ncafile {self.path / 'ca.crt'}\n"
                f"certfile {self.path / 'server.crt'}\nkeyfile {self.path / 'server.key'}\nrequire_certificate true\n")
            broker = self.spawn([shutil.which("mosquitto") or "/usr/sbin/mosquitto", "-c", str(broker_config)], "broker")
            time.sleep(.2)
            self.assertIsNone(broker.poll(), self.read_logs())
            rclpy.init()
            self.node = rclpy.create_node("mqtt_tls_pipeline")
            self.addCleanup(self.shutdown_ros)
            received, statuses = [], {}
            self.subscriptions = [self.node.create_subscription(String,
                "/ros2embedded/domain_7/connection_secure/rx", lambda msg: received.append(msg.data), 10)]
            command = self.node.create_publisher(String, "/ros2embedded/domain_7/connection_secure/tx", 10)
            executable = Path(get_package_prefix("bridge_node")) / "lib/bridge_node/bridge_node"
            for name, host, ca, password in (
                ("secure", "localhost", "ca", "local-test-password"),
                ("untrusted", "localhost", "wrong-ca", "local-test-password"),
                ("badhost", "127.0.0.1", "ca", "local-test-password"),
                ("badauth", "localhost", "ca", "wrong-password"),
            ):
                config = self.path / f"{name}.json"
                config.write_text(json.dumps({"connection_id": name, "domain_id": 7, "protocol": "PROTOCOL_MQTT",
                    "bindings": [
                        {"identifier": "device.status", "ros_topic": "rx", "direction": "DEVICE_TO_ROS", "value_type": "STRING"},
                        {"identifier": "host.status", "ros_topic": "tx", "direction": "ROS_TO_DEVICE", "value_type": "STRING"}]}))
                parameters = {"connection_id": name, "domain_id": 7, "protocol": "mqtt", "config_file": str(config),
                    "mqtt_broker": host, "mqtt_port": port, "mqtt_tls": True,
                    "mqtt_ca_file": str(self.path / f"{ca}.crt"),
                    "mqtt_client_cert": str(self.path / "client.crt"), "mqtt_client_key": str(self.path / "client.key"),
                    "mqtt_username": "bridge", "mqtt_password": password}
                params_file = self.path / f"{name}.yaml"
                params_file.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
                os.chmod(params_file, 0o600)
                statuses[name] = []
                self.subscriptions.append(self.node.create_subscription(String,
                    f"/ros2embedded/domain_7/connection_{name}/status",
                    lambda msg, name=name: statuses[name].append(msg.data), 10))
                self.spawn([str(executable), "--ros-args", "--params-file", str(params_file),
                            "-r", f"__node:=test_tls_{name}"], name)
            peer = mqtt.Client(client_id="secure-device", protocol=mqtt.MQTTv311)
            self.peer = peer
            replies = []
            peer.username_pw_set("bridge", "local-test-password")
            peer.tls_set(ca_certs=str(self.path / "ca.crt"), certfile=str(self.path / "client.crt"), keyfile=str(self.path / "client.key"))
            peer.on_connect = lambda client, userdata, flags, rc: client.subscribe("ros2embedded/7/secure/to_device", qos=1)
            peer.on_message = lambda client, userdata, msg: replies.append(json.loads(msg.payload))
            peer.connect("localhost", port, 15)
            peer.loop_start()
            self.addCleanup(self.stop_peer)
            self.wait(lambda: "connected" in statuses["secure"] and command.get_subscription_count() == 1)
            peer.publish("ros2embedded/7/secure/to_ros", json.dumps({"identifier": "device.status", "domain_id": 7,
                "value_type": "string", "value": "tls-ready"}), qos=1)
            self.wait(lambda: received == ["tls-ready"])
            command.publish(String(data="host-ready"))
            self.wait(lambda: any(p["value"] == "host-ready" for p in replies))
            self.wait(lambda: all(statuses[name] for name in ("untrusted", "badhost", "badauth")))
            for name in ("untrusted", "badhost", "badauth"):
                self.assertNotIn("connected", statuses[name], name + " accepted invalid TLS/auth")
                self.assertEqual(statuses[name][-1], "disconnected")

    def spawn(self, args, name):
        stream = open(self.path / f"{name}.log", "w+")
        self.logs.append(stream)
        process = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT)
        self.processes.append(process)
        return process

    def read_logs(self):
        return "\n".join(p.read_text() for p in self.path.glob("*.log"))

    def wait(self, predicate):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self.node, timeout_sec=.05)
        self.fail("TLS pipeline timed out:\n" + self.read_logs())

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
