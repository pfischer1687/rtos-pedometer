/**
 * @file entry/HITLProtocol.cpp
 * @brief HITL (Hardware In The Loop) protocol command parsing implementation.
 */

#include "entry/HITLProtocol.hpp"
#include "platform/Platform.hpp"
#include "platform/StringUtils.hpp"
#include <cstddef>
#include <cstdio>
#include <limits>
#include <optional>
#include <string_view>

namespace entry {

namespace {

/**
 * @brief Command entry for the command table.
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
    {"RESET", HITLCommand::RESET},
};

constexpr std::size_t COMMAND_COUNT =
    sizeof(commandTable) / sizeof(commandTable[0]);

constexpr platform::TickUs READ_SAMPLE_TIMEOUT_US = 500'000u; // 500 ms
constexpr unsigned int READ_POLL_DELAY_MS = 1u;

} // anonymous namespace

std::optional<std::uint16_t> parseInt(std::string_view s) noexcept {
  s = platform::str::trim(s);
  if (s.empty())
    return std::nullopt;

  std::uint16_t n = 0u;

  for (char c : s) {
    if (c < '0' || c > '9')
      return std::nullopt;

    const std::uint16_t digit = static_cast<std::uint16_t>(c - '0');

    if (n > std::numeric_limits<std::uint16_t>::max() / 10u ||
        (n == std::numeric_limits<std::uint16_t>::max() / 10u &&
         digit > std::numeric_limits<std::uint16_t>::max() % 10u)) {
      return std::nullopt;
    }

    n = n * 10u + digit;
  }

  return n;
}

ParsedCommand parseCommand(std::string_view line) noexcept {
  ParsedCommand out{};
  out.cmd = std::nullopt;
  out.argCount = 0u;

  std::string_view rest = platform::str::trim(line);
  if (rest.empty())
    return out;

  const std::string_view name = platform::str::nextToken(rest);
  out.name = name;
  if (name.empty())
    return out;

  for (std::size_t i = 0; i < MAX_ARGS; ++i) {
    const std::string_view arg = platform::str::nextToken(rest);
    if (arg.empty())
      break;

    out.args[i] = arg;
    out.argCount = i + 1u;
  }

  for (std::size_t i = 0; i < COMMAND_COUNT; ++i) {
    if (platform::str::iequals(name, commandTable[i].name)) {
      out.cmd = commandTable[i].cmd;
      break;
    }
  }

  return out;
}

void dispatchHITLCommand(usb::UsbInterface &usbInterface,
                         imu::Mpu6050Driver &imu, const ParsedCommand &parsed) {

  if (!parsed.cmd.has_value()) {
    usbInterface.sendResponse("UNKNOWN_COMMAND");
    return;
  }

  switch (parsed.cmd.value()) {
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
  case HITLCommand::READ_N_BYTES: {
    if (parsed.argCount < 1u) {
      usbInterface.sendResponse("INVALID_ARGUMENT");
      return;
    }

    const std::optional<std::uint16_t> nBytes = parseInt(parsed.args[0]);
    if (!nBytes.has_value() || nBytes.value() == 0u) {
      usbInterface.sendResponse("INVALID_ARGUMENT");
      return;
    }

    if (!imu.isSampling()) {
      usbInterface.sendResponse("INVALID_STATE");
      return;
    }

    platform::ITimer &timer = platform::timer();
    char msgBuf[96];
    int len =
        std::snprintf(msgBuf, sizeof(msgBuf), "READ_START %u", nBytes.value());
    if (len > 0 && static_cast<std::size_t>(len) < sizeof(msgBuf)) {
      usbInterface.sendResponse(msgBuf);
    }

    imu::ImuSample sample{};
    for (std::uint16_t i = 0u; i < nBytes.value(); ++i) {
      const platform::TickUs startUs = timer.nowUs();

      while (!imu.consumeDataReady()) {
        timer.delayMs(READ_POLL_DELAY_MS);
        const platform::TickUs elapsedUs =
            platform::elapsed(startUs, timer.nowUs());

        if (elapsedUs >= READ_SAMPLE_TIMEOUT_US) {
          usbInterface.sendResponse("READ_TIMEOUT");
          return;
        }
      }

      const platform::Result readResult = imu.readSample(sample);
      if (!platform::isOk(readResult)) {
        usbInterface.sendResponse("READ_FAIL");
        return;
      }

      len = std::snprintf(msgBuf, sizeof(msgBuf), "SAMPLE %u %d %d %d %lu", i,
                          sample.accel[0], sample.accel[1], sample.accel[2],
                          static_cast<unsigned long>(sample.timestampUs));

      if (len > 0 && static_cast<std::size_t>(len) < sizeof(msgBuf)) {
        usbInterface.sendResponse(msgBuf);
      }
    }

    usbInterface.sendResponse("READ_DONE");
    break;
  }
  case HITLCommand::RESET: {
    const platform::Result r = imu.reset();
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_RESET_OK"
                                                : "IMU_RESET_FAIL");
    break;
  }
  default:
    usbInterface.sendResponse("UNKNOWN_COMMAND");
    break;
  }
}

} // namespace entry
