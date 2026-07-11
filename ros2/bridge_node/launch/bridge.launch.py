from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("connection_id", default_value="esp32_default"),
            DeclareLaunchArgument("protocol", default_value="serial"),
            DeclareLaunchArgument("domain_id", default_value="0"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            Node(
                package="bridge_node",
                executable="bridge_node",
                name="ros2_embedded_bridge",
                parameters=[
                    {
                        "connection_id": LaunchConfiguration("connection_id"),
                        "protocol": LaunchConfiguration("protocol"),
                        "domain_id": ParameterValue(
                            LaunchConfiguration("domain_id"), value_type=int
                        ),
                        "serial_port": LaunchConfiguration("serial_port"),
                        "baudrate": ParameterValue(
                            LaunchConfiguration("baudrate"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
