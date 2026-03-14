/**
 * @file usb/UsbTransport.cpp
 * @brief USB transport implementation.
 */

#include "usb/UsbTransport.hpp"
#include <cstdint>
#include <cstring>

namespace usb {

/**
 * @brief USB baud rate.
 */
constexpr uint32_t BAUD_RATE = 115200;

UsbTransport::UsbTransport() : m_serial(USBTX, USBRX, BAUD_RATE) {
  m_serial.set_blocking(false);
}

std::optional<uint8_t> UsbTransport::readChar() noexcept {
  uint8_t c;
  if (m_serial.read(&c, 1) == 1) {
    return c;
  }
  return std::nullopt;
}

void UsbTransport::writeChar(char c) noexcept { m_serial.write(&c, 1); }

void UsbTransport::writeLine(const char *msg) noexcept {
  if (!msg)
    return;

  const std::size_t len = std::char_traits<char>::length(msg);
  if (len > 0) {
    m_serial.write(msg, len);
  }
  m_serial.write("\r\n", 2);
}

} // namespace usb