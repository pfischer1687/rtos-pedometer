/**
 * @file entry/HITLProtocol.cpp
 * @brief HITL (Hardware In The Loop) protocol command parsing implementation.
 */

#include "entry/HITLProtocol.hpp"
#include "platform/Platform.hpp"
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
                         imu::Mpu6050Driver &imu,
                         const std::optional<HITLCommand> &cmd) {

  if (!cmd.has_value()) {
    usbInterface.sendResponse("UNKNOWN_COMMAND");
    return;
  }

  switch (cmd.value()) {
  case HITLCommand::PING:
    usbInterface.sendResponse("PONG");
    break;
  case HITLCommand::INIT: {
    const platform::Result r = imu.init();
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_INIT_OK"
                                                : "IMU_INIT_FAIL");
    break;
  }
  case HITLCommand::CONFIGURE: {
    const imu::ImuConfig config{};
    const platform::Result r = imu.configure(config);
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_CONFIG_OK"
                                                : "IMU_CONFIG_FAIL");
    break;
  }
  case HITLCommand::START: {
    const platform::Result r = imu.startSampling();
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_START_OK"
                                                : "IMU_START_FAIL");
    break;
  }
  case HITLCommand::STOP: {
    const platform::Result r = imu.stopSampling();
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_STOP_OK"
                                                : "IMU_STOP_FAIL");
    break;
  }
  case HITLCommand::READ_N_BYTES:
    usbInterface.sendResponse("READING DATA");
    break;
  default:
    usbInterface.sendResponse("UNKNOWN_COMMAND");
    break;
  }
}

} // namespace entry
