#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <algorithm>
class HardwareSerial {
public:
  std::string incoming, outgoing;
  std::size_t write_limit = static_cast<std::size_t>(-1);
  int available() {return static_cast<int>(incoming.size());}
  int read() {
    if (incoming.empty()) return -1;
    const unsigned char c = incoming.front(); incoming.erase(0, 1); return c;
  }
  std::size_t write(const uint8_t * bytes, std::size_t size) {
    size = std::min(size, write_limit); outgoing.append(reinterpret_cast<const char *>(bytes), size); return size;
  }
};
extern HardwareSerial Serial;
