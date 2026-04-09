/**
 * @file usb/UsbInterface.cpp
 * @brief USB command parsing and response implementation.
 */

#include "usb/UsbInterface.hpp"
#include "platform/StringUtils.hpp"
#include "session/SessionManager.hpp"
#include <cstdio>
#include <cstring>
#include <type_traits>

namespace usb {

namespace {

constexpr std::size_t MAX_BYTES_PER_POLL = 32;

} // anonymous namespace

ParsedLine parseLine(std::string_view line) noexcept {
  ParsedLine out{};

  std::string_view rest = platform::str::trim(line);
  if (rest.empty())
    return out;

  const std::string_view name = platform::str::nextToken(rest);
  out.name = name;
  if (name.empty())
    return out;

  for (std::size_t i = 0; i < out.args.size(); ++i) {
    const std::string_view arg = platform::str::nextToken(rest);
    if (arg.empty())
      break;

    out.args[i] = arg;
    out.argCount = i + 1u;
  }

  return out;
}

UsbInterface::UsbInterface(IUsbTransport &transport) noexcept
    : _transport(transport) {}

bool UsbInterface::handleBackspace(char c) noexcept {
  if (c != '\b' && c != 127)
    return false;

  if (_rx_index > 0) {
    _rx_index--;
    _transport.writeChar('\b');
    _transport.writeChar(' ');
    _transport.writeChar('\b');
  }
  return true;
}

void UsbInterface::printPrompt() noexcept {
  _transport.writeChar('>');
  _transport.writeChar(' ');
}

bool UsbInterface::handleNewline(char c, char *line_buffer,
                                 std::size_t max_len) noexcept {
  if (c != '\n')
    return false;

  _transport.writeChar('\r');
  _transport.writeChar('\n');

  if (_rx_index == 0) {
    printPrompt();
    return false;
  }

  const std::size_t copy_len = std::min(_rx_index, max_len - 1);

  std::memcpy(line_buffer, _rx_buffer, copy_len);
  line_buffer[copy_len] = '\0';

  _rx_index = 0;
  _discarding = false;
  return true;
}

void UsbInterface::handleNormalChar(char c) noexcept {
  if (c < 32 || c > 126)
    return;

  if (_rx_index < USB_CMD_MAX_LEN - 1) {
    _rx_buffer[_rx_index++] = c;
    _transport.writeChar(c);
  } else {
    _transport.writeChar('\r');
    _transport.writeChar('\n');
    sendError("LINE_TOO_LONG");
    printPrompt();

    _discarding = false;
    _rx_index = 0;
  }
}

bool UsbInterface::poll(char *line_buffer, std::size_t max_len) noexcept {
  if (!line_buffer || max_len == 0) {
    return false;
  }

  line_buffer[0] = '\0';
  std::size_t processed = 0;

  while (processed < MAX_BYTES_PER_POLL) {
    auto ch = _transport.readChar();
    if (!ch)
      break;
    processed++;
    char c = static_cast<char>(*ch);

    if (c == '\r') {
      c = '\n';
      _lastWasCR = true;
    } else if (c == '\n' && _lastWasCR) {
      _lastWasCR = false;
      continue;
    } else {
      _lastWasCR = false;
    }

    if (_discarding) {
      if (c == '\n') {
        _discarding = false;
        _lastWasCR = false;
        _rx_index = 0;
        printPrompt();
      }
      continue;
    }

    if (handleBackspace(c))
      continue;

    if (handleNewline(c, line_buffer, max_len))
      return true;

    handleNormalChar(c);
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
