/**
 * @file app/Command.hpp
 * @brief Production app-layer command parsing.
 */

#ifndef APP_COMMAND_HPP
#define APP_COMMAND_HPP

#include "usb/UsbInterface.hpp"
#include <optional>

namespace app {

enum class CommandId {
  Start,
  Stop,
  Status,
  DebugStatus,
  DebugStart,
  Reset,
};

/**
 * @brief Maps usb::ParsedLine to CommandId.
 * @param line The line to parse.
 * @return The parsed command id, or std::nullopt if the line is empty or
 * contains an unknown command.
 */
[[nodiscard]] std::optional<CommandId>
parseCommand(const usb::ParsedLine &line) noexcept;

} // namespace app

#endif // APP_COMMAND_HPP
