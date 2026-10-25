#include "ros2_embedded/serial_connection_handler.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>

namespace ros2_embedded
{
namespace
{
speed_t baud_speed(const std::string & baud)
{
  if (baud == "9600") {return B9600;}
  if (baud == "19200") {return B19200;}
  if (baud == "38400") {return B38400;}
  if (baud == "57600") {return B57600;}
  if (baud == "115200") {return B115200;}
  if (baud == "230400") {return B230400;}
  throw std::invalid_argument("Unsupported baudrate");
}
}  // namespace
SerialConnectionHandler::~SerialConnectionHandler() {disconnect();}
bool SerialConnectionHandler::connect()
{
  disconnect();
  const auto port = params_.values.at("serial_port");
  const auto speed = baud_speed(params_.values.at("baudrate"));
  const auto encoding = params_.values.find("payload_encoding");
  if (encoding != params_.values.end() && encoding->second != "serial") {
    throw std::invalid_argument("Serial handler requires payload_encoding=serial");
  }
  if (port.empty()) {throw std::invalid_argument("serial_port is required");}
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {return false;}
  termios options{};
  if (tcgetattr(fd_, &options) != 0) {disconnect(); return false;}
  cfmakeraw(&options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~(CSTOPB | CRTSCTS);
  options.c_cc[VMIN] = 0; options.c_cc[VTIME] = 0;
  if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0 ||
    tcsetattr(fd_, TCSANOW, &options) != 0) {disconnect(); return false;}
  tcflush(fd_, TCIOFLUSH);
  connected_ = true;
  return true;
}
void SerialConnectionHandler::disconnect()
{
  if (fd_ >= 0) {close(fd_); fd_ = -1;}
  connected_ = false; input_.clear(); output_.clear(); discard_line_ = false;
  retry_at_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
}
bool SerialConnectionHandler::send(const TransportPayload & p)
{
  if (!is_connected() || !validate_domain(p)) {return false;}
  try {
    auto frame = encode_serial(p);
    if (output_.size() + frame.size() > kMaxWireBytes) {return false;}
    output_ += frame;
    return true;
  } catch (const std::exception &) {return false;}
}
void SerialConnectionHandler::poll()
{
  if (!is_connected()) {
    if (std::chrono::steady_clock::now() >= retry_at_) {connect();}
    return;
  }
  pollfd descriptor{fd_, POLLIN, 0};
  if (::poll(&descriptor, 1, 0) < 0 && errno != EINTR) {disconnect(); return;}
  if (descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)) {disconnect(); return;}
  // Limit work each tick so high-rate serial traffic cannot starve ROS callbacks.
  char buffer[1024];
  for (int attempt = 0; attempt < 8; ++attempt) {
    const auto count = read(fd_, buffer, sizeof(buffer));
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {break;}
      disconnect(); return;
    }
    if (count == 0) {break;}
    for (ssize_t i = 0; i < count; ++i) {
      const char c = buffer[i];
      if (c == '\n') {
        if (!discard_line_) {
          try {
            auto payload = decode_serial(input_);
            if (validate_domain(payload) && receive_callback_) {receive_callback_(payload);}
          } catch (const std::exception &) { /* Drop malformed frame, resume at next line. */ }
        }
        input_.clear(); discard_line_ = false;
      } else if (!discard_line_) {
        if (input_.size() >= 639) {input_.clear(); discard_line_ = true;}
        else {input_ += c;}
      }
    }
  }
  if (!output_.empty()) {
    const auto written = write(fd_, output_.data(), output_.size());
    if (written > 0) {output_.erase(0, static_cast<std::size_t>(written));}
    else if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {disconnect();}
  }
}
}  // namespace ros2_embedded
PLUGINLIB_EXPORT_CLASS(ros2_embedded::SerialConnectionHandler, ros2_embedded::ConnectionHandler)
