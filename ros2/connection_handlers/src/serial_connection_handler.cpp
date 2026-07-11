#include "ros2_embedded/serial_connection_handler.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace ros2_embedded
{

bool SerialConnectionHandler::connect()
{
  connected_ = true;
  return connected_;
}

bool SerialConnectionHandler::send(const TransportPayload & payload)
{
  return connected_ && validate_domain(payload);
}

void SerialConnectionHandler::poll()
{
  // Real serial framing is intentionally isolated here for the next milestone.
}

}  // namespace ros2_embedded

PLUGINLIB_EXPORT_CLASS(ros2_embedded::SerialConnectionHandler, ros2_embedded::ConnectionHandler)
