#include "ros2_embedded/mqtt_connection_handler.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace ros2_embedded
{

bool MqttConnectionHandler::connect()
{
  connected_ = true;
  return connected_;
}

bool MqttConnectionHandler::send(const TransportPayload & payload)
{
  return connected_ && validate_domain(payload);
}

void MqttConnectionHandler::poll()
{
  // MQTT client integration will publish received messages through receive_callback_.
}

}  // namespace ros2_embedded

PLUGINLIB_EXPORT_CLASS(ros2_embedded::MqttConnectionHandler, ros2_embedded::ConnectionHandler)
