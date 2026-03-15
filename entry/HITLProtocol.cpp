/**
 * @file entry/HITLProtocol.cpp
 * @brief HITL (Hardware In The Loop) protocol command parsing implementation.
 */

#include "entry/HITLProtocol.hpp"
#include <cctype>
#include <cstddef>
#include <optional>
#include <string_view>

namespace entry {

namespace {

/**
 * @brief Case-insensitive comparison of two string_views.
 * @param a First string view.
 * @param b Second string view.
 * @return True if the two string views are equal, false otherwise.
 */
constexpr bool caseInsensitiveEquals(std::string_view a,
                                     std::string_view b) noexcept {
  if (a.size() != b.size())
    return false;

  for (std::size_t i = 0; i < a.size(); ++i) {
    char ca = a[i];
    char cb = b[i];

    if (ca >= 'a' && ca <= 'z')
      ca -= ('a' - 'A');
    if (cb >= 'a' && cb <= 'z')
      cb -= ('a' - 'A');

    if (ca != cb)
      return false;
  }
  return true;
}

/**
 * @brief HITL command table for lookup.
 */
struct CommandEntry {
  std::string_view name;
  HITLCommand cmd;
};

/**
 * @brief HITL command table for lookup.
 */
constexpr CommandEntry commandTable[] = {
    {"PING", HITLCommand::PING},
    {"INIT", HITLCommand::INIT},
    {"CONFIGURE", HITLCommand::CONFIGURE},
    {"START", HITLCommand::START},
    {"STOP", HITLCommand::STOP},
    {"READ_N_BYTES", HITLCommand::READ_N_BYTES},
};

} // anonymous namespace

std::optional<HITLCommand> parseHITLCommand(std::string_view line) {
  if (line.empty()) {
    return std::nullopt;
  }

  constexpr std::size_t commandCount =
      sizeof(commandTable) / sizeof(commandTable[0]);

  for (std::size_t i = 0; i < commandCount; ++i) {
    if (caseInsensitiveEquals(line, commandTable[i].name)) {
      return commandTable[i].cmd;
    }
  }

  return std::nullopt;
}

void dispatchHITLCommand(usb::UsbInterface &usbInterface,
                         const std::optional<HITLCommand> &cmd) {

  if (!cmd.has_value()) {
    usbInterface.sendResponse("UNKNOWN_COMMAND");
    return;
  }

  switch (cmd.value()) {
  case HITLCommand::PING:
    usbInterface.sendResponse("PONG");
    break;
  case HITLCommand::INIT:
    usbInterface.sendResponse("INIT OK");
    break;
  case HITLCommand::CONFIGURE:
    usbInterface.sendResponse("CONFIGURED");
    break;
  case HITLCommand::START:
    usbInterface.sendResponse("STARTED");
    break;
  case HITLCommand::STOP:
    usbInterface.sendResponse("STOPPED");
    break;
  case HITLCommand::READ_N_BYTES:
    usbInterface.sendResponse("READING DATA");
    break;
  default:
    usbInterface.sendResponse("UNKNOWN_COMMAND");
    break;
  }
}

} // namespace entry
