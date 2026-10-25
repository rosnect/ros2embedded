#include "ros2_embedded/basic_connection_handler.hpp"

namespace ros2_embedded
{

void BasicConnectionHandler::configure(const ConnectionParams & params)
{
  disconnect();
  params_ = params;
}

void BasicConnectionHandler::disconnect()
{
  connected_ = false;
}

bool BasicConnectionHandler::is_connected() const
{
  return connected_;
}

void BasicConnectionHandler::set_receive_callback(ReceiveCallback callback)
{
  receive_callback_ = std::move(callback);
}

bool BasicConnectionHandler::validate_domain(const TransportPayload & payload) const
{
  return payload.domain_id == params_.domain_id;
}

}  // namespace ros2_embedded
