import subprocess
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class ManagedBridge:
    connection_id: str
    protocol: str
    domain_id: int
    process: subprocess.Popen


class BridgeManager(Node):
    def __init__(self) -> None:
        super().__init__("ros2_embedded_bridge_manager")
        self._bridges: Dict[str, ManagedBridge] = {}
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self._timer = self.create_timer(1.0, self._publish_status)

        self.declare_parameter("domain_id", 0)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("connection_id", "default")
        self.declare_parameter("protocol", "serial")
        self.declare_parameter("serial_port", "")
        self.declare_parameter("baudrate", 115200)

        if self.get_parameter("auto_start").value:
            domain_id = int(self.get_parameter("domain_id").value)
            if domain_id < 0:
                raise ValueError("domain_id must be greater than or equal to 0")
            self.start_bridge(
                connection_id=self.get_parameter("connection_id").value,
                protocol=self.get_parameter("protocol").value,
                domain_id=domain_id,
                extra_args=[
                    ("serial_port", self.get_parameter("serial_port").value),
                    ("baudrate", str(self.get_parameter("baudrate").value)),
                ],
            )

    def start_bridge(
        self,
        connection_id: str,
        protocol: str,
        domain_id: int,
        extra_args: List[tuple[str, str]],
    ) -> None:
        if domain_id < 0:
            raise ValueError("domain_id must be greater than or equal to 0")
        if connection_id in self._bridges:
            self.get_logger().warn(f"Bridge already exists: {connection_id}")
            return

        ros_args = [
            "ros2",
            "run",
            "bridge_node",
            "bridge_node",
            "--ros-args",
            "-p",
            f"connection_id:={connection_id}",
            "-p",
            f"protocol:={protocol}",
            "-p",
            f"domain_id:={domain_id}",
        ]
        for key, value in extra_args:
            ros_args.extend(["-p", f"{key}:={value}"])

        process = subprocess.Popen(ros_args)
        self._bridges[connection_id] = ManagedBridge(
            connection_id=connection_id,
            protocol=protocol,
            domain_id=domain_id,
            process=process,
        )

    def _publish_status(self) -> None:
        alive = []
        for bridge in self._bridges.values():
            state = "running" if bridge.process.poll() is None else "exited"
            alive.append(
                f"{bridge.connection_id}:{bridge.protocol}:domain={bridge.domain_id}:{state}"
            )
        msg = String()
        msg.data = ",".join(alive)
        self._status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = BridgeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
