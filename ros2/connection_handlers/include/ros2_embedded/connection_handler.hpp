#pragma once

#include "ros2_embedded/payload.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace ros2_embedded
{

struct ConnectionParams
{
  std::string connection_id;
  std::uint32_t domain_id{0};
  std::map<std::string, std::string> values;
};

using ConfigurationCallback = std::function<void(const std::string &)>;
using ReceiveCallback = std::function<void(const TransportPayload &)>;

class ConnectionHandler
{
public:
  virtual ~ConnectionHandler() = default;

  virtual void configure(const ConnectionParams & params) = 0;
  virtual bool connect() = 0;
  virtual void disconnect() = 0;
  virtual bool is_connected() const = 0;
  virtual bool send(const TransportPayload & payload) = 0;
  virtual void poll() = 0;
  virtual void set_receive_callback(ReceiveCallback callback) = 0;

  // Optional transport control channel. Codecs/ROS binding policy stay in the node.
  virtual bool supports_device_configuration() const {return false;}
  virtual void set_configuration_callback(ConfigurationCallback callback) {(void)callback;}
  virtual bool send_configuration_status(const std::string & json) {(void)json; return false;}
  virtual bool request_configuration() {return false;}
};

}  // namespace ros2_embedded
