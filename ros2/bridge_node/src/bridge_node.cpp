#include "ros2_embedded/connection_handler.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{

std::string plugin_name_for_protocol(const std::string & protocol)
{
  if (protocol == "serial") {
    return "connection_handlers/SerialConnectionHandler";
  }
  if (protocol == "mqtt") {
    return "connection_handlers/MqttConnectionHandler";
  }
  if (protocol == "redis") {
    return "connection_handlers/RedisConnectionHandler";
  }
  throw std::invalid_argument("Unsupported protocol: " + protocol);
}

}  // namespace

class BridgeNode final : public rclcpp::Node
{
public:
  BridgeNode()
  : Node("ros2_embedded_bridge"),
    loader_("connection_handlers", "ros2_embedded::ConnectionHandler")
  {
    const auto connection_id = declare_parameter<std::string>("connection_id", "default");
    const auto protocol = declare_parameter<std::string>("protocol", "serial");
    const auto domain_id_param = declare_parameter<int>("domain_id", 0);
    if (domain_id_param < 0) {
      throw std::invalid_argument("domain_id must be greater than or equal to 0");
    }
    const auto domain_id = static_cast<std::uint32_t>(domain_id_param);
    const auto device_to_ros_topic =
      declare_parameter<std::string>("device_to_ros_topic", "~/device_data");
    const auto ros_to_device_topic =
      declare_parameter<std::string>("ros_to_device_topic", "~/device_commands");

    ros2_embedded::ConnectionParams params;
    params.connection_id = connection_id;
    params.domain_id = domain_id;
    params.values["serial_port"] = declare_parameter<std::string>("serial_port", "");
    params.values["baudrate"] = std::to_string(declare_parameter<int>("baudrate", 115200));
    params.values["mqtt_broker"] = declare_parameter<std::string>("mqtt_broker", "");
    params.values["redis_uri"] = declare_parameter<std::string>("redis_uri", "");

    device_pub_ = create_publisher<std_msgs::msg::String>(device_to_ros_topic, 10);
    command_sub_ = create_subscription<std_msgs::msg::String>(
      ros_to_device_topic,
      10,
      [this, domain_id](const std_msgs::msg::String::SharedPtr msg) {
        ros2_embedded::TransportPayload payload;
        payload.identifier = "command";
        payload.domain_id = domain_id;
        payload.data.assign(msg->data.begin(), msg->data.end());
        if (!handler_->send(payload)) {
          RCLCPP_WARN(get_logger(), "Failed to send payload for domain_id=%u", domain_id);
        }
      });

    handler_ = loader_.createSharedInstance(plugin_name_for_protocol(protocol));
    handler_->configure(params);
    handler_->set_receive_callback(
      [this, domain_id](const ros2_embedded::TransportPayload & payload) {
        if (payload.domain_id != domain_id) {
          RCLCPP_WARN(
            get_logger(), "Dropping payload for domain_id=%u; expected domain_id=%u",
            payload.domain_id, domain_id);
          return;
        }

        std_msgs::msg::String msg;
        msg.data.assign(payload.data.begin(), payload.data.end());
        device_pub_->publish(msg);
      });

    if (!handler_->connect()) {
      throw std::runtime_error("Connection handler failed to connect");
    }

    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(5),
      [this]() {
        handler_->poll();
      });

    RCLCPP_INFO(
      get_logger(), "Started bridge connection_id=%s protocol=%s domain_id=%u",
      connection_id.c_str(), protocol.c_str(), domain_id);
  }

private:
  pluginlib::ClassLoader<ros2_embedded::ConnectionHandler> loader_;
  std::shared_ptr<ros2_embedded::ConnectionHandler> handler_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr device_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
