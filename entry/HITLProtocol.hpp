/**
 * @file entry/HITLProtocol.hpp
 * @brief HITL (Hardware In The Loop) command dispatch.
 */

#ifndef ENTRY_HITL_PROTOCOL_HPP
#define ENTRY_HITL_PROTOCOL_HPP

#include "imu/Mpu6050Driver.hpp"
#include "usb/Command.hpp"
#include "usb/UsbInterface.hpp"
#include <optional>
#include <string_view>

namespace entry {

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
                         imu::Mpu6050Driver &imu,
                         const usb::ParsedCommand &parsed);

} // namespace entry

#endif // ENTRY_HITL_PROTOCOL_HPP
