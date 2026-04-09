/**
 * @file hitl/HitlProtocol.hpp
 * @brief HITL command parsing helpers and dispatch.
 */

#ifndef HITL_HITL_PROTOCOL_HPP
#define HITL_HITL_PROTOCOL_HPP

#include "imu/Mpu6050Driver.hpp"
#include "usb/UsbInterface.hpp"
#include <cstdint>
#include <optional>
#include <string_view>

namespace hitl {

/**
 * @brief HITL command identifiers.
 */
enum class HitlCommandId {
  Ping,
  Init,
  Config,
  Start,
  Stop,
  ReadNBytes,
  Reset,
};

/**
 * @brief Parse a 16-bit unsigned integer from a string view (digits only).
 * @param s The string view to parse.
 * @return The parsed integer, or std::nullopt if the string is empty or
 * contains non-digits.
 */
[[nodiscard]] std::optional<std::uint16_t>
parseInt(std::string_view s) noexcept;

/**
 * @brief Map a tokenized line to a HITL command, if recognized.
 * @param line The line to map.
 * @return The mapped command, or std::nullopt if the line is empty or contains
 * an unknown command.
 */
[[nodiscard]] std::optional<HitlCommandId>
mapCommandId(const usb::ParsedLine &line) noexcept;

/**
 * @brief Dispatch a HITL command.
 * @param usbInterface The USB interface to send responses.
 * @param imu The IMU driver to use.
 * @param id Resolved command id.
 * @param line The line to dispatch.
 */
void dispatchCommand(usb::UsbInterface &usbInterface, imu::Mpu6050Driver &imu,
                     HitlCommandId id, const usb::ParsedLine &line) noexcept;

} // namespace hitl

#endif // HITL_HITL_PROTOCOL_HPP
