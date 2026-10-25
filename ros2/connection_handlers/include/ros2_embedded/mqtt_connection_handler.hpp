#pragma once
#include "ros2_embedded/basic_connection_handler.hpp"
#include <memory>
namespace ros2_embedded
{
class MqttConnectionHandler final : public BasicConnectionHandler
{
public:
  MqttConnectionHandler();
  ~MqttConnectionHandler() override;
  bool connect() override;
  void disconnect() override;
  bool send(const TransportPayload & payload) override;
  void poll() override;
  bool supports_device_configuration() const override {return true;}
  void set_configuration_callback(ConfigurationCallback callback) override;
  bool send_configuration_status(const std::string & json) override;
  bool request_configuration() override;
private:
  bool publish_control(const std::string & topic, const std::string & wire);
  ConfigurationCallback configuration_callback_;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace ros2_embedded
