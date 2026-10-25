#pragma once

#include "ros2_embedded/connection_handler.hpp"

#include <atomic>

namespace ros2_embedded
{

class BasicConnectionHandler : public ConnectionHandler
{
public:
  void configure(const ConnectionParams & params) override;
  void disconnect() override;
  bool is_connected() const override;
  void set_receive_callback(ReceiveCallback callback) override;

protected:
  bool validate_domain(const TransportPayload & payload) const;

  ConnectionParams params_;
  std::atomic<bool> connected_{false};
  ReceiveCallback receive_callback_;
};

}  // namespace ros2_embedded
