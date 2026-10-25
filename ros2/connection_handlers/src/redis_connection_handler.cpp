#include "ros2_embedded/redis_connection_handler.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace ros2_embedded
{

bool RedisConnectionHandler::connect()
{
  // Never report a usable transport until Redis I/O is implemented.
  connected_ = false;
  return false;
}

bool RedisConnectionHandler::send(const TransportPayload & payload)
{
  (void)payload;
  return false;
}

void RedisConnectionHandler::poll()
{
  // Redis pub/sub integration will publish received messages through receive_callback_.
}

}  // namespace ros2_embedded

PLUGINLIB_EXPORT_CLASS(ros2_embedded::RedisConnectionHandler, ros2_embedded::ConnectionHandler)
