#pragma once

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

struct TransportPayload
{
  std::string identifier;
  std::uint32_t domain_id{0};
  std::vector<std::uint8_t> data;
};

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
};

}  // namespace ros2_embedded
