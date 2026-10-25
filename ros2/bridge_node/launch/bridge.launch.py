from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_bridge(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    connection_id = value("connection_id")
    escaped = "".join(c if c.isascii() and c.isalnum() else f"_{ord(c):02x}"
                      for c in connection_id)
    protocol = value("protocol")
    tls = value("mqtt_tls").lower() == "true"
    params = {key: ParameterValue(value(key), value_type=str) for key in (
        "connection_id", "protocol", "config_file", "config_encoding", "config_source", "serial_port",
        "mqtt_broker", "mqtt_base_topic", "mqtt_username", "mqtt_password", "mqtt_ca_file",
        "mqtt_client_cert", "mqtt_client_key",
    )}
    params.update(
        domain_id=int(value("domain_id")), baudrate=int(value("baudrate")), mqtt_tls=tls,
        mqtt_port=int(value("mqtt_port")) or (8883 if tls else 1883),
        payload_encoding=ParameterValue(value("payload_encoding") or
                                        ("serial" if protocol == "serial" else "json"), value_type=str),
    )
    return [Node(package="bridge_node", executable="bridge_node",
                 name=f"bridge_d{params['domain_id']}_{escaped}", parameters=[params], output="screen")]


def generate_launch_description():
    defaults = {
        "connection_id": "default", "protocol": "serial", "domain_id": "0",
        "config_file": "", "config_encoding": "json", "config_source": "auto", "serial_port": "",
        "baudrate": "115200", "payload_encoding": "", "mqtt_broker": "",
        "mqtt_port": "0", "mqtt_tls": "false", "mqtt_base_topic": "ros2embedded",
        "mqtt_username": "", "mqtt_password": "", "mqtt_ca_file": "",
        "mqtt_client_cert": "", "mqtt_client_key": "",
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()),
        OpaqueFunction(function=launch_bridge),
    ])
