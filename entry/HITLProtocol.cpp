/**
 * @file entry/HITLProtocol.cpp
 * @brief HITL (Hardware In The Loop) protocol command parsing implementation.
 */

#include "entry/HITLProtocol.hpp"
#include "platform/Platform.hpp"
#include <cctype>
#include <cstddef>
#include <cstdio>
#include <limits>
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
};

constexpr std::size_t COMMAND_COUNT =
    sizeof(commandTable) / sizeof(commandTable[0]);

constexpr platform::TickUs READ_SAMPLE_TIMEOUT_US = 100000u; // 100 ms
constexpr unsigned int READ_POLL_DELAY_MS = 1u;

/**
 * @brief Trim leading whitespace (space or tab).
 */
constexpr std::string_view trimLeading(std::string_view s) noexcept {
  while (!s.empty() && (s[0] == ' ' || s[0] == '\t')) {
    s.remove_prefix(1);
  }
  return s;
}

/**
 * @brief Extract next token (non-whitespace run); advance rest past the token.
 */
constexpr std::string_view nextToken(std::string_view &rest) noexcept {
  rest = trimLeading(rest);
  if (rest.empty())
    return {};

  const std::size_t start = 0u;
  std::size_t len = 0u;

  while (len < rest.size() && rest[len] != ' ' && rest[len] != '\t') {
    ++len;
  }

  std::string_view token = rest.substr(start, len);
  rest.remove_prefix(len);
  return token;
}

} // anonymous namespace

std::optional<std::uint16_t> parseInt(std::string_view s) noexcept {
  s = trimLeading(s);
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

  std::string_view rest = trimLeading(line);
  if (rest.empty())
    return out;

  const std::string_view name = nextToken(rest);
  out.name = name;
  if (name.empty())
    return out;

  for (std::size_t i = 0; i < MAX_ARGS; ++i) {
    const std::string_view arg = nextToken(rest);
    if (arg.empty())
      break;

    out.args[i] = arg;
    out.argCount = i + 1u;
  }

  for (std::size_t i = 0; i < COMMAND_COUNT; ++i) {
    if (caseInsensitiveEquals(name, commandTable[i].name)) {
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
  default:
    usbInterface.sendResponse("UNKNOWN_COMMAND");
    break;
  }
}

} // namespace entry
