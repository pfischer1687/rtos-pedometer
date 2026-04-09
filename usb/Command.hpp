/**
 * @file usb/Command.hpp
 * @brief Shared USB line command id and parsed representation (HITL +
 * production).
 */

#ifndef USB_COMMAND_HPP
#define USB_COMMAND_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace usb {

/**
 * @brief Command identifiers for shared USB line parsing.
 */
enum class CommandId : std::uint8_t {
  None,
  Ping,
  Start,
  Stop,
  Status,
  Reset,
  Configure,
  Init,
  ReadNBytes,
};

/**
 * @brief Maximum number of arguments per command line.
 */
constexpr std::size_t COMMAND_MAX_ARGS = 4u;

/**
 * @brief Tokenized command: id from the constexpr table, plus name and args.
 */
struct ParsedCommand {
  CommandId id = CommandId::None;
  std::string_view name;
  std::array<std::string_view, COMMAND_MAX_ARGS> args{};
  std::size_t argCount = 0u;
};

/**
 * @brief Parse a line into command id (None if unknown), name, and arguments.
 */
[[nodiscard]] ParsedCommand parseCommand(std::string_view line) noexcept;

} // namespace usb

#endif // USB_COMMAND_HPP
