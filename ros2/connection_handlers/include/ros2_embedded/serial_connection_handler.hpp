#pragma once
#include "ros2_embedded/basic_connection_handler.hpp"
#include <chrono>
#include <string>
namespace ros2_embedded
{
class SerialConnectionHandler final : public BasicConnectionHandler
{
public:
  ~SerialConnectionHandler() override;
  bool connect() override;
  void disconnect() override;
  bool send(const TransportPayload & payload) override;
  void poll() override;
private:
  int fd_{-1};
  std::string input_;
  std::string output_;
  bool discard_line_{false};
  std::chrono::steady_clock::time_point retry_at_{};
};
}  // namespace ros2_embedded
