/**
 * @file usb/UsbTransport.hpp
 * @brief Transport interface for USB.
 */

#ifndef USB_USBTRANSPORT_HPP
#define USB_USBTRANSPORT_HPP

#include "mbed.h"
#include "usb/IUsbTransport.hpp"
#include <cstdint>
#include <optional>

namespace usb {

class UsbTransport final : public IUsbTransport {
public:
  UsbTransport();
  std::optional<uint8_t> readChar() noexcept override;
  void writeChar(char c) noexcept override;
  void writeLine(const char *msg) noexcept override;

private:
  mbed::BufferedSerial m_serial;
};

} // namespace usb

#endif /* USB_USBTRANSPORT_HPP */