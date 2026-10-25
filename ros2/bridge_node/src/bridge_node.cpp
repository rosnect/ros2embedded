#include "ros2_embedded/connection_handler.hpp"
#include "bridge_config.pb.h"
#include <google/protobuf/util/json_util.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <chrono>
#include <fstream>
#include <deque>
#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace
{
using ros2_embedded::TransportPayload;
using ros2_embedded::ValueType;
using ros2embedded::v1::TopicBinding;
using ros2embedded::v1::BridgeConfig;
std::string plugin_name(const std::string & protocol)
{
  if (protocol == "serial") {return "connection_handlers/SerialConnectionHandler";}
  if (protocol == "mqtt") {return "connection_handlers/MqttConnectionHandler";}
  if (protocol == "redis") {throw std::invalid_argument("Redis is not implemented");}
  throw std::invalid_argument("Unsupported protocol: " + protocol);
}
std::string ros_type(ValueType type)
{
  switch (type) {
    case ValueType::Bool: return "std_msgs/msg/Bool";
    case ValueType::Int32: return "std_msgs/msg/Int32";
    case ValueType::UInt32: return "std_msgs/msg/UInt32";
    case ValueType::Int64: return "std_msgs/msg/Int64";
    case ValueType::UInt64: return "std_msgs/msg/UInt64";
    case ValueType::Float32: return "std_msgs/msg/Float32";
    case ValueType::Float64: return "std_msgs/msg/Float64";
    case ValueType::String: case ValueType::Json: return "std_msgs/msg/String";
    case ValueType::Bytes: return "std_msgs/msg/ByteMultiArray";
  }
  throw std::invalid_argument("Unsupported value_type");
}
// Injective escaping: '.', '-', and '_' cannot collapse distinct connection IDs.
std::string ros_connection_name(const std::string & id)
{
  if (id.empty() || id.size() > 96) {throw std::invalid_argument("Invalid connection_id length");}
  std::string name = "connection_";
  const char * hex = "0123456789abcdef";
  for (unsigned char c : id) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {name += c;}
    else if (c == '.' || c == '-' || c == '_') {name += '_'; name += hex[c >> 4]; name += hex[c & 15];}
    else {throw std::invalid_argument("Invalid connection_id character");}
  }
  return name;
}
std::string read_config(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {throw std::invalid_argument("Cannot open config_file: " + path);}
  std::string contents;
  char chunk[4096];
  while (file.read(chunk, sizeof(chunk)) || file.gcount()) {
    contents.append(chunk, static_cast<std::size_t>(file.gcount()));
    if (contents.size() > 1024 * 1024) {throw std::invalid_argument("Config exceeds 1 MiB");}
  }
  return contents;
}
}  // namespace

class BridgeNode final : public rclcpp::Node
{
  struct Route {ValueType type; std::function<void(const TransportPayload &)> publish;};
  struct Interfaces {
    std::map<std::string, Route> routes;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  };
public:
  BridgeNode()
  : Node("ros2_embedded_bridge"), loader_("connection_handlers", "ros2_embedded::ConnectionHandler")
  {
    ros2_embedded::ConnectionParams params;
    connection_id_ = params.connection_id = declare_parameter<std::string>("connection_id", "default");
    protocol_ = declare_parameter<std::string>("protocol", "serial");
    const auto domain = declare_parameter<std::int64_t>("domain_id", 0);
    if (domain < 0 || domain > UINT32_MAX) {throw std::invalid_argument("domain_id must fit uint32");}
    domain_id_ = params.domain_id = static_cast<std::uint32_t>(domain);
    prefix_ = "/ros2embedded/domain_" + std::to_string(domain_id_) + "/" + ros_connection_name(connection_id_);
    const auto config_path = declare_parameter<std::string>("config_file", "");
    config_encoding_ = declare_parameter<std::string>("config_encoding", "json");
    if (config_encoding_ != "json" && config_encoding_ != "protobuf") {
      throw std::invalid_argument("config_encoding must be json or protobuf");
    }
    auto source = declare_parameter<std::string>("config_source", "auto");
    if (source == "auto") {source = !config_path.empty() ? "file" : protocol_ == "mqtt" ? "device" : "file";}
    if (source != "file" && source != "device") {throw std::invalid_argument("config_source must be auto, file or device");}
    device_configuration_ = source == "device";
    if (device_configuration_ && !config_path.empty()) {
      throw std::invalid_argument("Device configuration does not use config_file; choose one source");
    }
    if (!device_configuration_ && config_path.empty()) {throw std::invalid_argument("config_file is required for file configuration");}
    params.values["config_source"] = source;
    params.values["config_encoding"] = config_encoding_;
    params.values["serial_port"] = declare_parameter<std::string>("serial_port", "");
    params.values["baudrate"] = std::to_string(declare_parameter<std::int64_t>("baudrate", 115200));
    payload_encoding_ = params.values["payload_encoding"] = declare_parameter<std::string>(
      "payload_encoding", protocol_ == "serial" ? "serial" : "json");
    for (const auto & entry : std::map<std::string, std::string>{
      {"mqtt_broker", ""}, {"mqtt_base_topic", "ros2embedded"}, {"mqtt_username", ""},
      {"mqtt_password", ""}, {"mqtt_ca_file", ""}, {"mqtt_client_cert", ""}, {"mqtt_client_key", ""}})
    {params.values[entry.first] = declare_parameter<std::string>(entry.first, entry.second);}
    const bool tls = declare_parameter<bool>("mqtt_tls", false);
    params.values["mqtt_tls"] = tls ? "true" : "false";
    params.values["mqtt_port"] = std::to_string(declare_parameter<std::int64_t>("mqtt_port", tls ? 8883 : 1883));

    handler_ = loader_.createSharedInstance(plugin_name(protocol_));
    if (device_configuration_ && !handler_->supports_device_configuration()) {
      throw std::invalid_argument("This transport does not yet support device configuration");
    }
    handler_->configure(params);
    status_pub_ = create_publisher<std_msgs::msg::String>(prefix_ + "/status", rclcpp::QoS(1).transient_local());
    interface_status_pub_ = create_publisher<std_msgs::msg::String>(prefix_ + "/interface_status", rclcpp::QoS(1).transient_local());
    if (device_configuration_) {
      handler_->set_configuration_callback([this](const std::string & wire) {receive_configuration(wire);});
      publish_configuration_status(make_status("", false, "", nullptr));
    } else {
      auto config = parse_config(read_config(config_path));
      apply_config(std::move(config));
      publish_configuration_status(make_status(active_config_.configuration_id(), true, "", &active_config_));
    }
    handler_->set_receive_callback([this](const TransportPayload & p) {
      if (p.domain_id != domain_id_) {return;}
      const auto route = interfaces_->routes.find(p.identifier);
      if (route == interfaces_->routes.end() || route->second.type != p.type) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Dropping unconfigured identifier or mismatched value type; device must advertise interfaces first");
        return;
      }
      try {ros2_embedded::validate_payload(p); route->second.publish(p);}
      catch (const std::exception & e) {RCLCPP_WARN(get_logger(), "Dropping invalid payload: %s", e.what());}
    });
    if (!handler_->connect()) {throw std::runtime_error("Transport failed to open; check its configuration");}
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {handler_->poll();});
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      std_msgs::msg::String message;
      message.data = handler_->is_connected() ? "connected" : "disconnected";
      status_pub_->publish(message);
    });
    if (device_configuration_) {
      request_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
        if (!interfaces_ready_ && handler_->is_connected()) {handler_->request_configuration();}
      });
      RCLCPP_INFO(get_logger(), "Waiting for device BridgeConfig on MQTT <base>/<domain>/<connection>/config");
    }
    RCLCPP_INFO(get_logger(), "Bridge started: connection=%s protocol=%s domain=%u prefix=%s config_source=%s",
      connection_id_.c_str(), protocol_.c_str(), domain_id_, prefix_.c_str(), source.c_str());
  }
  ~BridgeNode() override {if (handler_) {handler_->disconnect();}}
private:
  BridgeConfig parse_config(const std::string & wire)
  {
    BridgeConfig config;
    if (config_encoding_ == "json") {
      const auto result = google::protobuf::util::JsonStringToMessage(wire, &config);
      if (!result.ok()) {throw std::invalid_argument("Invalid BridgeConfig JSON: " + result.ToString());}
    } else if (!config.ParseFromString(wire)) {throw std::invalid_argument("Invalid BridgeConfig protobuf");}
    return config;
  }
  std::string topic_name(const std::string & configured)
  {
    if (configured.empty()) {throw std::invalid_argument("ros_topic is required");}
    return get_node_topics_interface()->resolve_topic_name(
      configured.front() == '/' ? configured : prefix_ + "/" + configured);
  }
  std::string binding_topic(const std::string & configured)
  {
    if (device_configuration_ && (!configured.empty() && (configured.front() == '/' || configured.front() == '~'))) {
      throw std::invalid_argument("Device-advertised ROS topics must be relative to this connection");
    }
    const auto topic = topic_name(configured);
    if (device_configuration_ && topic.rfind(prefix_ + "/", 0) != 0) {
      throw std::invalid_argument("Device-advertised ROS topics cannot escape their connection namespace");
    }
    if (topic == status_pub_->get_topic_name() || topic == interface_status_pub_->get_topic_name()) {
      throw std::invalid_argument("Binding conflicts with a reserved bridge status topic");
    }
    return topic;
  }
  void normalize_and_validate(BridgeConfig & config)
  {
    const auto expected_protocol = protocol_ == "serial" ? ros2embedded::v1::PROTOCOL_SERIAL : ros2embedded::v1::PROTOCOL_MQTT;
    if (config.connection_id() != connection_id_ || config.domain_id() != domain_id_ || config.protocol() != expected_protocol) {
      throw std::invalid_argument("Config connection_id/domain_id/protocol must match node parameters");
    }
    if (config.schema_version() > 1 || (device_configuration_ && config.schema_version() != 1)) {
      throw std::invalid_argument("Device config requires schema_version=1");
    }
    if (device_configuration_ || !config.configuration_id().empty()) {
      try {ros_connection_name(config.configuration_id());}
      catch (const std::exception &) {throw std::invalid_argument("configuration_id requires 1..96 ASCII letters, digits, _, . or -");}
    }
    config.set_schema_version(1);
    if (config.bindings_size() > 256) {throw std::invalid_argument("Config permits at most 256 bindings");}
    std::set<std::string> identifiers, publish_topics, command_topics;
    for (auto & binding : *config.mutable_bindings()) {
      if (!ros2_embedded::valid_identifier(binding.identifier()) || !identifiers.insert(binding.identifier()).second) {
        throw std::invalid_argument("Invalid or duplicate identifier");
      }
      const auto expected_type = ros_type(static_cast<ValueType>(binding.value_type()));
      if (!binding.ros_message_type().empty() && binding.ros_message_type() != expected_type) {
        throw std::invalid_argument("ros_message_type does not match value_type for " + binding.identifier());
      }
      binding.set_ros_message_type(expected_type);
      const auto direction = binding.direction();
      if (direction != ros2embedded::v1::DEVICE_TO_ROS && direction != ros2embedded::v1::ROS_TO_DEVICE &&
        direction != ros2embedded::v1::BIDIRECTIONAL) {throw std::invalid_argument("Invalid binding direction");}
      if (device_configuration_) {
        // Escaping is identical to connection IDs, including dots/slashes in identifiers.
        // A slash is first encoded as its ASCII escape; underscores are escaped by the helper.
        std::string token = "id_";
        const char * hex = "0123456789abcdef";
        for (unsigned char c : binding.identifier()) {
          if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {token += c;}
          else {token += '_'; token += hex[c >> 4]; token += hex[c & 15];}
        }
        if (binding.ros_topic().empty()) {
          binding.set_ros_topic((direction == ros2embedded::v1::ROS_TO_DEVICE ? "commands/" : "data/") + token);
        }
        if (direction == ros2embedded::v1::BIDIRECTIONAL && binding.ros_command_topic().empty()) {
          binding.set_ros_command_topic("commands/" + token);
        }
      }
      if (direction != ros2embedded::v1::ROS_TO_DEVICE) {
        if (!publish_topics.insert(binding_topic(binding.ros_topic())).second) {throw std::invalid_argument("Duplicate device output topic");}
      }
      if (direction != ros2embedded::v1::DEVICE_TO_ROS) {
        if (direction == ros2embedded::v1::BIDIRECTIONAL && binding.ros_command_topic().empty()) {
          throw std::invalid_argument("BIDIRECTIONAL requires a separate ros_command_topic");
        }
        if (direction == ros2embedded::v1::ROS_TO_DEVICE && !binding.ros_command_topic().empty()) {
          throw std::invalid_argument("ROS_TO_DEVICE uses ros_topic; ros_command_topic is only for BIDIRECTIONAL");
        }
        command_topics.insert(binding_topic(direction == ros2embedded::v1::BIDIRECTIONAL ? binding.ros_command_topic() : binding.ros_topic()));
      } else if (!binding.ros_command_topic().empty()) {throw std::invalid_argument("Unexpected ros_command_topic");}
    }
    for (const auto & topic : publish_topics) {
      if (command_topics.count(topic)) {throw std::invalid_argument("ROS output and command topics overlap: " + topic);}
    }
  }
  std::string make_status(const std::string & request_id, bool accepted, const std::string & error, const BridgeConfig * active)
  {
    ros2embedded::v1::BridgeConfigStatus status;
    status.set_connection_id(connection_id_); status.set_domain_id(domain_id_);
    status.set_configuration_id(request_id); status.set_accepted(accepted);
    status.set_error(error.substr(0, 512)); status.set_state(active ? "ready" : "waiting_for_config");
    status.set_payload_encoding(payload_encoding_); status.set_schema_version(1); status.set_ros_namespace(prefix_);
    if (active) {
      status.set_active_configuration_id(active->configuration_id());
      status.set_bindings_count(active->bindings_size());
      *status.mutable_bindings() = active->bindings();
    }
    google::protobuf::util::JsonPrintOptions options;
    options.preserve_proto_field_names = true;
    options.always_print_primitive_fields = true;
    std::string wire;
    const auto result = google::protobuf::util::MessageToJsonString(status, &wire, options);
    if (!result.ok()) {throw std::runtime_error("Could not serialize config status");}
    return wire;
  }
  void publish_configuration_status(const std::string & wire)
  {
    std_msgs::msg::String message; message.data = wire; interface_status_pub_->publish(message);
    if (device_configuration_ && !handler_->send_configuration_status(wire) && handler_->is_connected()) {
      RCLCPP_WARN(get_logger(), "Could not queue config acknowledgement; device should retry its snapshot");
    }
  }
  void receive_configuration(const std::string & wire)
  {
    BridgeConfig config;
    try {
      config = parse_config(wire);
      apply_config(config);
      publish_configuration_status(make_status(active_config_.configuration_id(), true, "", &active_config_));
    } catch (const std::exception & e) {
      std::string request_id = config.configuration_id();
      try {ros_connection_name(request_id);} catch (const std::exception &) {request_id.clear();}
      RCLCPP_WARN(get_logger(), "Rejected device configuration: %s", e.what());
      publish_configuration_status(make_status(request_id, false, e.what(), interfaces_ready_ ? &active_config_ : nullptr));
    }
  }
  void apply_config(BridgeConfig config)
  {
    normalize_and_validate(config);
    const auto canonical = config.SerializeAsString();
    if (interfaces_ready_ && canonical == active_config_.SerializeAsString()) {return;}
    if (device_configuration_ && interfaces_ready_) {
      if (config.configuration_id() == active_config_.configuration_id()) {
        throw std::invalid_argument("Changed snapshots require a new configuration_id");
      }
      if (std::find(retired_ids_.begin(), retired_ids_.end(), config.configuration_id()) != retired_ids_.end()) {
        throw std::invalid_argument("Superseded configuration_id; publish a new snapshot ID");
      }
    }
    // Ensure even normalized configs have a bounded acknowledgement (with error headroom).
    if (make_status(config.configuration_id(), true, "", &config).size() > ros2_embedded::kMaxWireBytes - 2048) {
      throw std::invalid_argument("Normalized configuration/status is too large");
    }
    auto candidate = std::make_shared<Interfaces>();
    const auto next_generation = generation_ + 1;
    for (const auto & binding : config.bindings()) {
      switch (static_cast<ValueType>(binding.value_type())) {
        case ValueType::Bool: add_binding<std_msgs::msg::Bool>(binding, *candidate, next_generation); break;
        case ValueType::Int32: add_binding<std_msgs::msg::Int32>(binding, *candidate, next_generation); break;
        case ValueType::UInt32: add_binding<std_msgs::msg::UInt32>(binding, *candidate, next_generation); break;
        case ValueType::Int64: add_binding<std_msgs::msg::Int64>(binding, *candidate, next_generation); break;
        case ValueType::UInt64: add_binding<std_msgs::msg::UInt64>(binding, *candidate, next_generation); break;
        case ValueType::Float32: add_binding<std_msgs::msg::Float32>(binding, *candidate, next_generation); break;
        case ValueType::Float64: add_binding<std_msgs::msg::Float64>(binding, *candidate, next_generation); break;
        case ValueType::String: case ValueType::Json: add_binding<std_msgs::msg::String>(binding, *candidate, next_generation); break;
        case ValueType::Bytes: add_binding<std_msgs::msg::ByteMultiArray>(binding, *candidate, next_generation); break;
      }
    }
    if (device_configuration_ && interfaces_ready_) {
      retired_ids_.push_back(active_config_.configuration_id());
      if (retired_ids_.size() > 128) {retired_ids_.pop_front();}
    }
    active_config_.Swap(&config);
    generation_ = next_generation;
    interfaces_.swap(candidate);
    interfaces_ready_ = true;
    RCLCPP_INFO(get_logger(), "Applied %d bindings for configuration_id=%s", active_config_.bindings_size(), active_config_.configuration_id().c_str());
  }
  template<typename Message>
  void add_binding(const TopicBinding & binding, Interfaces & interfaces, std::uint64_t generation)
  {
    const auto type = static_cast<ValueType>(binding.value_type());
    using Data = typename Message::_data_type;
    if (binding.direction() != ros2embedded::v1::ROS_TO_DEVICE) {
      auto publisher = create_publisher<Message>(topic_name(binding.ros_topic()), 10);
      interfaces.routes.emplace(binding.identifier(), Route{type, [publisher](const TransportPayload & p) {
        Message message;
        if constexpr (std::is_same_v<Data, bool>) {message.data = std::get<bool>(p.value);}
        else if constexpr (std::is_integral_v<Data> && std::is_signed_v<Data>) {message.data = static_cast<Data>(std::get<std::int64_t>(p.value));}
        else if constexpr (std::is_integral_v<Data>) {message.data = static_cast<Data>(std::get<std::uint64_t>(p.value));}
        else if constexpr (std::is_floating_point_v<Data>) {message.data = static_cast<Data>(std::get<double>(p.value));}
        else if constexpr (std::is_same_v<Data, std::string>) {message.data = std::get<std::string>(p.value);}
        else {message.data = std::get<ros2_embedded::Bytes>(p.value);}
        publisher->publish(message);
      }});
    }
    if (binding.direction() != ros2embedded::v1::DEVICE_TO_ROS) {
      const auto topic = binding.direction() == ros2embedded::v1::BIDIRECTIONAL ? binding.ros_command_topic() : binding.ros_topic();
      interfaces.subscriptions.push_back(create_subscription<Message>(topic_name(topic), 10,
        [this, identifier = binding.identifier(), type, generation](const typename Message::SharedPtr msg) {
          // An executor may still hold a callback from a removed subscription.
          if (generation != generation_) {return;}
          TransportPayload p;
          p.identifier = identifier; p.domain_id = domain_id_; p.type = type;
          p.timestamp_unix_ns = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
          if constexpr (std::is_same_v<Data, bool>) {p.value = msg->data;}
          else if constexpr (std::is_integral_v<Data> && std::is_signed_v<Data>) {p.value = std::int64_t(msg->data);}
          else if constexpr (std::is_integral_v<Data>) {p.value = std::uint64_t(msg->data);}
          else if constexpr (std::is_floating_point_v<Data>) {p.value = double(msg->data);}
          else {p.value = msg->data;}
          if (!handler_->send(p)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Command rejected: disconnected, invalid value, or full transport queue");
          }
        }));
    }
  }
  pluginlib::ClassLoader<ros2_embedded::ConnectionHandler> loader_;
  std::shared_ptr<ros2_embedded::ConnectionHandler> handler_;
  std::shared_ptr<Interfaces> interfaces_{std::make_shared<Interfaces>()};
  BridgeConfig active_config_;
  std::deque<std::string> retired_ids_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, interface_status_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_, status_timer_, request_timer_;
  std::uint32_t domain_id_{0};
  std::uint64_t generation_{0};
  bool device_configuration_{false}, interfaces_ready_{false};
  std::string prefix_, connection_id_, protocol_, config_encoding_, payload_encoding_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<BridgeNode>());}
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ros2_embedded_bridge"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
