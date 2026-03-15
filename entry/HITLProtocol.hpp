/**
 * @file entry/HITLProtocol.hpp
 * @brief HITL (Hardware In The Loop) protocol command types and parsing.
 */

#ifndef ENTRY_HITL_PROTOCOL_HPP
#define ENTRY_HITL_PROTOCOL_HPP

#include "usb/UsbInterface.hpp"
#include <optional>
#include <string_view>

namespace entry {

/**
 * @brief HITL command types.
 */
enum class HITLCommand {
  PING,
  INIT,
  CONFIGURE,
  START,
  STOP,
  READ_N_BYTES,
};

/**
 * @brief Parse a HITL command from a line.
 * @param line Input string view.
 * @return Optional HITL command.
 */
std::optional<HITLCommand> parseHITLCommand(std::string_view line);

/**
 * @brief Dispatch a HITL command.
 * @param usbInterface USB interface.
 * @param cmd HITL command.
 */
void dispatchHITLCommand(usb::UsbInterface &usbInterface,
                         const std::optional<HITLCommand> &cmd);

} // namespace entry

#endif // ENTRY_HITL_PROTOCOL_HPP
