#pragma once

#include "ros2_embedded/basic_connection_handler.hpp"

namespace ros2_embedded
{

class SerialConnectionHandler final : public BasicConnectionHandler
{
public:
  bool connect() override;
  bool send(const TransportPayload & payload) override;
  void poll() override;
};

}  // namespace ros2_embedded
