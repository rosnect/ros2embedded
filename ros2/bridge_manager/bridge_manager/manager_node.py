import re
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


def validate_identity(connection_id: str, protocol: str, domain_id: int) -> None:
    if not re.fullmatch(r"[A-Za-z0-9_.-]{1,96}", connection_id):
        raise ValueError("Invalid connection_id")
    if protocol not in ("serial", "mqtt"):
        raise ValueError("Only serial and mqtt are implemented")
    if not isinstance(domain_id, int) or not 0 <= domain_id <= 0xFFFFFFFF:
        raise ValueError("domain_id must fit uint32")


def node_name(connection_id: str, domain_id: int) -> str:
    escaped = "".join(c if c.isalnum() else f"_{ord(c):02x}" for c in connection_id)
    return f"bridge_d{domain_id}_{escaped}"


@dataclass
class ManagedBridge:
    connection_id: str
    protocol: str
    domain_id: int
    process: subprocess.Popen
    params_file: str


class BridgeManager(Node):
    def __init__(self) -> None:
        super().__init__("ros2_embedded_bridge_manager")
        self._bridges: Dict[str, ManagedBridge] = {}
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self._timer = self.create_timer(1.0, self._publish_status)
        defaults = {
            "auto_start": False, "connection_id": "default", "protocol": "serial",
            "domain_id": 0, "config_file": "", "config_encoding": "json", "config_source": "auto",
            "serial_port": "", "baudrate": 115200, "mqtt_broker": "",
            "mqtt_port": 0, "mqtt_tls": False, "mqtt_username": "",
            "mqtt_password": "", "mqtt_ca_file": "", "mqtt_client_cert": "",
            "mqtt_client_key": "", "mqtt_base_topic": "ros2embedded", "payload_encoding": "",
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        if self.get_parameter("auto_start").value:
            extras = [(key, self.get_parameter(key).value) for key in defaults
                      if key not in ("auto_start", "connection_id", "protocol", "domain_id")]
            self.start_bridge(
                self.get_parameter("connection_id").value,
                self.get_parameter("protocol").value,
                self.get_parameter("domain_id").value,
                extras,
            )

    def start_bridge(self, connection_id: str, protocol: str, domain_id: int,
                     extra_args: List[tuple[str, object]]) -> None:
        validate_identity(connection_id, protocol, domain_id)
        previous = self._bridges.get(connection_id)
        if previous is not None:
            if previous.process.poll() is None:
                self.get_logger().warn(f"Bridge already running: {connection_id}")
                return
            previous.process.wait()
            Path(previous.params_file).unlink(missing_ok=True)
            del self._bridges[connection_id]
        params = dict(extra_args)
        # Identity cannot be overridden by extra arguments.
        params.update(connection_id=connection_id, protocol=protocol, domain_id=domain_id)
        source = params.get("config_source", "auto")
        if source == "auto":
            source = "file" if params.get("config_file") or protocol != "mqtt" else "device"
        if source not in ("file", "device"):
            raise ValueError("config_source must be auto, file or device")
        if source == "device" and (protocol != "mqtt" or params.get("config_file")):
            raise ValueError("Device configuration requires MQTT and no config_file")
        if source == "file" and not params.get("config_file"):
            raise ValueError("config_file is required for file configuration")
        params["config_source"] = source
        if not params.get("payload_encoding"):
            params["payload_encoding"] = "serial" if protocol == "serial" else "json"
        if not params.get("mqtt_port"):
            params["mqtt_port"] = 8883 if params.get("mqtt_tls") else 1883
        executable = Path(get_package_prefix("bridge_node")) / "lib/bridge_node/bridge_node"
        # Keep credentials out of process command lines. NamedTemporaryFile uses mode 0600.
        with tempfile.NamedTemporaryFile(mode="w", prefix="ros2embedded-", suffix=".yaml",
                                         delete=False) as stream:
            yaml.safe_dump({"/**": {"ros__parameters": params}}, stream)
            params_file = stream.name
        try:
            process = subprocess.Popen([
                str(executable), "--ros-args", "--params-file", params_file,
                "-r", f"__node:={node_name(connection_id, domain_id)}",
            ])
        except BaseException:
            Path(params_file).unlink(missing_ok=True)
            raise
        self._bridges[connection_id] = ManagedBridge(
            connection_id, protocol, domain_id, process, params_file)

    def stop_bridge(self, connection_id: str) -> None:
        bridge = self._bridges.pop(connection_id, None)
        if bridge is None:
            return
        try:
            if bridge.process.poll() is None:
                bridge.process.terminate()
            try:
                bridge.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                bridge.process.kill()
                bridge.process.wait(timeout=3)
        finally:
            Path(bridge.params_file).unlink(missing_ok=True)

    def _publish_status(self) -> None:
        states = []
        for bridge in self._bridges.values():
            state = "running" if bridge.process.poll() is None else "exited"
            if state == "exited":
                Path(bridge.params_file).unlink(missing_ok=True)
            states.append(f"{bridge.connection_id}:{bridge.protocol}:domain={bridge.domain_id}:{state}")
        msg = String()
        msg.data = ",".join(states)
        self._status_pub.publish(msg)

    def destroy_node(self):
        for connection_id in list(self._bridges):
            self.stop_bridge(connection_id)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = BridgeManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
