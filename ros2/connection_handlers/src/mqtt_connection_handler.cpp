#include "ros2_embedded/mqtt_connection_handler.hpp"
#include <mosquitto.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <mutex>
#include <set>
#include <stdexcept>
#include <cstdlib>
#include <regex>
#include <algorithm>
#include <variant>
#include "bridge_config.pb.h"
#include <google/protobuf/util/json_util.h>

namespace ros2_embedded
{
namespace
{
const auto logger = rclcpp::get_logger("mqtt_connection_handler");
void check(int result)
{
  if (result != MOSQ_ERR_SUCCESS) {throw std::runtime_error(mosquitto_strerror(result));}
}
std::string option(const ConnectionParams & params, const std::string & name, const std::string & fallback = "")
{
  auto it = params.values.find(name);
  return it == params.values.end() ? fallback : it->second;
}
}  // namespace
struct MqttConnectionHandler::Impl
{
  mosquitto * client{nullptr};
  bool loop_started{false};
  std::mutex mutex;
  struct Configuration {std::string wire;};
  std::deque<std::variant<TransportPayload, Configuration>> received;
  bool device_configuration{false};
  bool request_pending{false};
  std::set<int> pending;
  std::string input_topic;
  std::string output_topic;
  std::string encoding;
  std::string config_topic;
  std::string config_status_topic;
  std::string config_request_topic;
  std::set<int> subscription_mids;
};
MqttConnectionHandler::MqttConnectionHandler() : impl_(new Impl) {}
MqttConnectionHandler::~MqttConnectionHandler() {disconnect();}

bool MqttConnectionHandler::connect()
{
  disconnect();
  // Keep libmosquitto initialized for process lifetime: plugins may coexist.
  static const int initialized = mosquitto_lib_init();
  check(initialized);
  if (!std::regex_match(params_.connection_id, std::regex("[A-Za-z0-9_.-]+"))) {
    throw std::invalid_argument("MQTT connection_id must use letters, digits, _, . or -");
  }
  const auto host = option(params_, "mqtt_broker");
  if (host.empty() || host.find("://") != std::string::npos || host.find('/') != std::string::npos) {
    throw std::invalid_argument("mqtt_broker must be a hostname or IP without a URL scheme");
  }
  const bool tls = option(params_, "mqtt_tls", "false") == "true";
  const auto port_text = option(params_, "mqtt_port", tls ? "8883" : "1883");
  std::size_t parsed = 0;
  const int port = std::stoi(port_text, &parsed);
  if (parsed != port_text.size() || port < 1 || port > 65535) {throw std::invalid_argument("Invalid mqtt_port");}
  auto base = option(params_, "mqtt_base_topic", "ros2embedded");
  if (base.empty() || base.find_first_of("+#\0", 0, 3) != std::string::npos) {
    throw std::invalid_argument("Invalid mqtt_base_topic");
  }
  const auto prefix = base + "/" + std::to_string(params_.domain_id) + "/" + params_.connection_id;
  impl_->input_topic = prefix + "/to_ros";
  impl_->output_topic = prefix + "/to_device";
  impl_->config_topic = prefix + "/config";
  impl_->config_status_topic = prefix + "/config_status";
  impl_->config_request_topic = prefix + "/config_request";
  impl_->device_configuration = option(params_, "config_source", "file") == "device";
  impl_->encoding = option(params_, "payload_encoding", "json");
  if (impl_->encoding != "json" && impl_->encoding != "protobuf") {
    throw std::invalid_argument("MQTT payload_encoding must be json or protobuf");
  }
  const auto cert = option(params_, "mqtt_client_cert");
  const auto key = option(params_, "mqtt_client_key");
  if (cert.empty() != key.empty()) {throw std::invalid_argument("Both MQTT client certificate and key are required");}
  if (!tls && (!cert.empty() || !option(params_, "mqtt_ca_file").empty())) {
    throw std::invalid_argument("MQTT certificate options require mqtt_tls=true");
  }
  const auto id = "ros2embedded-host-" + std::to_string(params_.domain_id) + "-" + params_.connection_id;
  impl_->client = mosquitto_new(id.c_str(), true, this);
  if (!impl_->client) {throw std::runtime_error("Cannot allocate MQTT client");}
  try {
    check(mosquitto_int_option(impl_->client, MOSQ_OPT_PROTOCOL_VERSION, MQTT_PROTOCOL_V311));
    check(mosquitto_max_inflight_messages_set(impl_->client, 32));
    check(mosquitto_reconnect_delay_set(impl_->client, 1, 30, true));
    const auto username = option(params_, "mqtt_username");
    const auto password = option(params_, "mqtt_password");
    if (!password.empty() && username.empty()) {throw std::invalid_argument("MQTT password requires username");}
    if (!username.empty()) {check(mosquitto_username_pw_set(impl_->client, username.c_str(), password.c_str()));}
    if (tls) {
      auto ca = option(params_, "mqtt_ca_file");
      // Ubuntu/Humble system trust. Override mqtt_ca_file for a private CA.
      if (ca.empty()) {ca = "/etc/ssl/certs/ca-certificates.crt";}
      check(mosquitto_tls_set(impl_->client, ca.c_str(), nullptr,
        cert.empty() ? nullptr : cert.c_str(), key.empty() ? nullptr : key.c_str(), nullptr));
      check(mosquitto_tls_opts_set(impl_->client, 1, "tlsv1.2", nullptr));
    }
    mosquitto_connect_callback_set(impl_->client, [](mosquitto * client, void * context, int rc) {
      auto self = static_cast<MqttConnectionHandler *>(context);
      try {
        self->connected_ = false;
        if (rc != 0) {
          RCLCPP_WARN(logger, "Broker rejected connection: %s", mosquitto_connack_string(rc));
          return;
        }
        std::lock_guard<std::mutex> lock(self->impl_->mutex);
        self->impl_->subscription_mids.clear();
        int mid = 0;
        check(mosquitto_subscribe(client, &mid, self->impl_->input_topic.c_str(), 1));
        self->impl_->subscription_mids.insert(mid);
        if (self->impl_->device_configuration) {
          check(mosquitto_subscribe(client, &mid, self->impl_->config_topic.c_str(), 1));
          self->impl_->subscription_mids.insert(mid);
        }
      } catch (const std::exception & e) {RCLCPP_ERROR(logger, "MQTT subscribe failed: %s", e.what());}
    });
    mosquitto_subscribe_callback_set(impl_->client, [](mosquitto *, void * context, int mid, int count, const int * qos) {
      auto self = static_cast<MqttConnectionHandler *>(context);
      std::lock_guard<std::mutex> lock(self->impl_->mutex);
      if (!self->impl_->subscription_mids.count(mid)) {return;}
      if (count == 1 && qos && qos[0] >= 0 && qos[0] <= 2) {
        self->impl_->subscription_mids.erase(mid);
        if (self->impl_->subscription_mids.empty()) {
          self->connected_ = true;
          self->impl_->request_pending = self->impl_->device_configuration;
        }
      } else {RCLCPP_ERROR(logger, "Broker denied bridge subscription");}
    });
    mosquitto_disconnect_callback_set(impl_->client, [](mosquitto *, void * context, int rc) {
      auto self = static_cast<MqttConnectionHandler *>(context);
      self->connected_ = false;
      std::lock_guard<std::mutex> lock(self->impl_->mutex);
      self->impl_->received.clear();
      self->impl_->request_pending = false;
      if (rc != 0) {RCLCPP_WARN(logger, "MQTT disconnected; reconnecting with bounded backoff");}
    });
    mosquitto_publish_callback_set(impl_->client, [](mosquitto *, void * context, int mid) {
      auto self = static_cast<MqttConnectionHandler *>(context);
      std::lock_guard<std::mutex> lock(self->impl_->mutex);
      self->impl_->pending.erase(mid);
    });
    mosquitto_message_callback_set(impl_->client, [](mosquitto *, void * context, const mosquitto_message * msg) {
      auto self = static_cast<MqttConnectionHandler *>(context);
      try {
        if (!msg || !msg->topic || msg->payloadlen < 0 ||
          static_cast<std::size_t>(msg->payloadlen) > kMaxWireBytes) {return;}
        const std::string wire(msg->payloadlen ? static_cast<const char *>(msg->payload) : "", msg->payloadlen);
        if (self->impl_->device_configuration && self->impl_->config_topic == msg->topic) {
          std::lock_guard<std::mutex> lock(self->impl_->mutex);
          const auto is_config = [](const auto & event) {return std::holds_alternative<Impl::Configuration>(event);};
          if (std::count_if(self->impl_->received.begin(), self->impl_->received.end(), is_config) >= 8) {return;}
          if (self->impl_->received.size() >= 128) {
            auto data = std::find_if(self->impl_->received.begin(), self->impl_->received.end(),
              [&is_config](const auto & event) {return !is_config(event);});
            if (data != self->impl_->received.end()) {self->impl_->received.erase(data);}
          }
          self->impl_->received.emplace_back(Impl::Configuration{wire});
          return;
        }
        if (self->impl_->input_topic != msg->topic || wire.empty()) {return;}
        auto p = self->impl_->encoding == "json" ? decode_json(wire) : decode_protobuf(wire);
        if (!self->validate_domain(p)) {return;}
        std::lock_guard<std::mutex> lock(self->impl_->mutex);
        // Reserve room for configuration snapshots; preserve control/data arrival order.
        if (self->impl_->received.size() < 120) {self->impl_->received.push_back(std::move(p));}
        // Overflow drops the newest message. Never let a device grow memory without bound.
      } catch (const std::exception &) {
        // Malformed/unrelated traffic must not escape a C callback or kill the network loop.
      }
    });
    check(mosquitto_connect_async(impl_->client, host.c_str(), port, 15));
    check(mosquitto_loop_start(impl_->client));
    impl_->loop_started = true;
    return true;  // Started; is_connected becomes true only after successful SUBACK.
  } catch (...) {disconnect(); throw;}
}
void MqttConnectionHandler::disconnect()
{
  if (impl_->client) {
    mosquitto_disconnect(impl_->client);
    if (impl_->loop_started) {mosquitto_loop_stop(impl_->client, true);}
    mosquitto_destroy(impl_->client);
    impl_->client = nullptr;
  }
  impl_->loop_started = false;
  connected_ = false;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->received.clear(); impl_->pending.clear();
  impl_->subscription_mids.clear(); impl_->request_pending = false;
}
bool MqttConnectionHandler::send(const TransportPayload & p)
{
  if (!is_connected() || !validate_domain(p)) {return false;}
  try {
    auto wire = impl_->encoding == "json" ? encode_json(p) : encode_protobuf(p);
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->pending.size() >= 128) {return false;}
    int mid = 0;
    const auto result = mosquitto_publish(impl_->client, &mid, impl_->output_topic.c_str(),
      static_cast<int>(wire.size()), wire.data(), 1, false);
    if (result != MOSQ_ERR_SUCCESS) {return false;}
    impl_->pending.insert(mid);
    return true;  // Queued for QoS 1; not an acknowledgement from the device.
  } catch (const std::exception &) {return false;}
}
void MqttConnectionHandler::set_configuration_callback(ConfigurationCallback callback)
{
  configuration_callback_ = std::move(callback);
}
bool MqttConnectionHandler::publish_control(const std::string & topic, const std::string & wire)
{
  if (!is_connected() || wire.size() > kMaxWireBytes) {return false;}
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->pending.size() >= 128) {return false;}
  int mid = 0;
  if (mosquitto_publish(impl_->client, &mid, topic.c_str(), static_cast<int>(wire.size()),
    wire.data(), 1, false) != MOSQ_ERR_SUCCESS) {return false;}
  impl_->pending.insert(mid);
  return true;
}
bool MqttConnectionHandler::send_configuration_status(const std::string & json)
{
  return impl_->device_configuration && publish_control(impl_->config_status_topic, json);
}
bool MqttConnectionHandler::request_configuration()
{
  if (!impl_->device_configuration) {return false;}
  ros2embedded::v1::BridgeConfigRequest request;
  request.set_connection_id(params_.connection_id);
  request.set_domain_id(params_.domain_id);
  request.set_schema_version(1);
  request.set_config_encoding(option(params_, "config_encoding", "json"));
  request.set_payload_encoding(impl_->encoding);
  std::string wire;
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  options.always_print_primitive_fields = true;
  if (!google::protobuf::util::MessageToJsonString(request, &wire, options).ok()) {return false;}
  return publish_control(impl_->config_request_topic, wire);
}
void MqttConnectionHandler::poll()
{
  // ROS/configuration callbacks execute on the executor, never the MQTT worker.
  decltype(impl_->received) batch;
  bool request = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    batch.swap(impl_->received);
    request = impl_->request_pending;
    impl_->request_pending = false;
  }
  if (request) {request_configuration();}
  for (const auto & event : batch) {
    if (auto config = std::get_if<Impl::Configuration>(&event)) {
      if (configuration_callback_) {configuration_callback_(config->wire);}
    } else if (receive_callback_) {receive_callback_(std::get<TransportPayload>(event));}
  }
}
}  // namespace ros2_embedded
PLUGINLIB_EXPORT_CLASS(ros2_embedded::MqttConnectionHandler, ros2_embedded::ConnectionHandler)
