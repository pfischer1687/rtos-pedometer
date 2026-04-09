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
  Reset,
};

/**
 * @brief Command envelope from USB thread to session thread.
 */
struct Command {
  std::optional<CommandId> id{};
};

/**
 * @brief Maps usb::ParsedLine to Command.
 */
struct CommandParser {
  [[nodiscard]] static Command parse(const usb::ParsedLine &line) noexcept;
};

} // namespace app

#endif // APP_COMMAND_HPP
