/**
 * @file entry/HITLProtocol.hpp
 * @brief HITL (Hardware In The Loop) protocol command types and parsing.
 */

#ifndef ENTRY_HITL_PROTOCOL_HPP
#define ENTRY_HITL_PROTOCOL_HPP

#include "imu/Mpu6050Driver.hpp"
#include "usb/UsbInterface.hpp"
#include <array>
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
 * @brief Maximum number of arguments per command.
 */
constexpr std::size_t MAX_ARGS = 4u;

/**
 * @brief Tokenized command and arguments from a line.
 */
struct ParsedCommand {
  std::optional<HITLCommand> cmd;
  std::string_view name;
  std::array<std::string_view, MAX_ARGS> args{};
  std::size_t argCount = 0u;
};

/**
 * @brief Parse a line into command name and arguments (tokenized).
 * @param line Input string view (not modified).
 * @return ParsedCommand with cmd set if name matched, args and argCount filled.
 */
ParsedCommand parseCommand(std::string_view line) noexcept;

/**
 * @brief Parse a 16-bit unsigned integer from a string view.
 * @param s Input string view (digits only).
 * @return The value, or std::nullopt on failure or overflow.
 */
std::optional<std::uint16_t> parseInt(std::string_view s) noexcept;

/**
 * @brief Dispatch a HITL command.
 * @param usbInterface USB interface.
 * @param imu MPU6050 driver instance.
 * @param parsed Parsed command and arguments.
 */
void dispatchHITLCommand(usb::UsbInterface &usbInterface,
                         imu::Mpu6050Driver &imu, const ParsedCommand &parsed);

} // namespace entry

#endif // ENTRY_HITL_PROTOCOL_HPP
