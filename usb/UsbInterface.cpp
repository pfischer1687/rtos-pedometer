/**
 * @file usb/UsbInterface.cpp
 * @brief USB command parsing and response implementation.
 */

#include "usb/UsbInterface.hpp"
#include "session/SessionManager.hpp"
#include <cstdio>
#include <cstring>
#include <type_traits>

namespace usb {

UsbInterface::UsbInterface(IUsbTransport &transport) noexcept
    : _transport(transport) {}

bool UsbInterface::poll(char *line_buffer, std::size_t max_len) noexcept {
  if (!line_buffer || max_len == 0) {
    return false;
  }

  while (true) {
    std::optional<uint8_t> ch = _transport.readChar();
    if (ch == std::nullopt)
      break;

    if (_discarding) {
      if (*ch == '\n') {
        _discarding = false;
        _rx_index = 0;
      }
      continue;
    }

    if (*ch == '\r') {
      continue;
    }

    if (*ch == '\n') {
      const std::size_t copy_len =
          (_rx_index < max_len - 1) ? _rx_index : max_len - 1;

      std::memcpy(line_buffer, _rx_buffer, copy_len);
      line_buffer[copy_len] = '\0';

      _rx_index = 0;
      return true;
    }

    if (_rx_index < USB_CMD_MAX_LEN - 1) {
      _rx_buffer[_rx_index++] = static_cast<char>(*ch);
    } else {
      _discarding = true;
      _rx_index = 0;
      sendError("LINE_TOO_LONG");
    }
  }

  return false;
}

void UsbInterface::sendResponse(const char *msg) noexcept {
  _transport.writeLine(msg);
}

void UsbInterface::sendError(const char *msg) noexcept {
  _transport.writeLine(msg);
}

void UsbInterface::sendStatus(
    const session::SessionSnapshot &snapshot) noexcept {

  char buffer[128];

  const int written =
      std::snprintf(buffer, sizeof(buffer), "STEPS=%u STATE=%u",
                    static_cast<unsigned>(snapshot.stepCount),
                    static_cast<std::underlying_type_t<session::SessionState>>(
                        snapshot.state));

  if (written < 0 || static_cast<std::size_t>(written) >= sizeof(buffer)) {
    sendError("FORMAT_ERROR");
    return;
  }

  sendResponse(buffer);
}

} // namespace usb
