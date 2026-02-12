/**
 * @file usb/UsbInterface.cpp
 * @brief USB command parsing and response implementation.
 */

#include "usb/UsbInterface.hpp"

namespace usb {

platform::Result UsbInterface::init() noexcept { return platform::Result::Ok; }

bool UsbInterface::poll(ParsedCommand &cmd) noexcept {
  cmd.id = CommandId::None;
  return false;
}

void UsbInterface::sendStatus(
    const session::SessionSnapshot &snapshot) noexcept {}

void UsbInterface::sendResponse(const char *msg) noexcept {}

void UsbInterface::sendError(const char *msg) noexcept {}

} // namespace usb
