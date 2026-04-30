/**
 * @file hitl/HitlProtocol.cpp
 * @brief HITL command mapping and dispatch.
 */

#include "hitl/HitlProtocol.hpp"
#include "platform/Platform.hpp"
#include "platform/StringUtils.hpp"

#include <cstddef>
#include <cstdio>
#include <limits>
#include <optional>
#include <string_view>

namespace hitl {

namespace {

constexpr platform::TickUs READ_SAMPLE_TIMEOUT_US = 500'000u; // 500 ms
constexpr unsigned int READ_POLL_DELAY_MS = 1u;

struct HitlCommandEntry {
  std::string_view name;
  HitlCommandId id;
};

constexpr HitlCommandEntry hitlCommandTable[] = {
    {"PING", HitlCommandId::Ping},
    {"INIT", HitlCommandId::Init},
    {"CONFIGURE", HitlCommandId::Config},
    {"START", HitlCommandId::Start},
    {"STOP", HitlCommandId::Stop},
    {"READ_N_BYTES", HitlCommandId::ReadNBytes},
    {"RESET", HitlCommandId::Reset},
};

constexpr std::size_t HITL_COMMAND_COUNT =
    sizeof(hitlCommandTable) / sizeof(hitlCommandTable[0]);

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

std::optional<HitlCommandId>
mapCommandId(const usb::ParsedLine &line) noexcept {
  if (line.name.empty())
    return std::nullopt;

  for (std::size_t i = 0; i < HITL_COMMAND_COUNT; ++i) {
    if (platform::str::iequals(line.name, hitlCommandTable[i].name)) {
      return hitlCommandTable[i].id;
    }
  }
  return std::nullopt;
}

void dispatchCommand(usb::UsbInterface &usbInterface, imu::Mpu6050Driver &imu,
                     HitlCommandId id, const usb::ParsedLine &line) noexcept {

  switch (id) {
  case HitlCommandId::Ping:
    usbInterface.sendResponse("PONG");
    break;
  case HitlCommandId::Init: {
    const platform::Result r = imu.init();
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_INIT_OK"
                                                : "IMU_INIT_FAIL");
    break;
  }
  case HitlCommandId::Config: {
    const imu::ImuConfig config{};
    const platform::Result r = imu.configure(config);
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_CONFIG_OK"
                                                : "IMU_CONFIG_FAIL");
    break;
  }
  case HitlCommandId::Start: {
    const platform::Result r = imu.startSampling();
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_START_OK"
                                                : "IMU_START_FAIL");
    break;
  }
  case HitlCommandId::Stop: {
    const platform::Result r = imu.stopSampling();
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_STOP_OK"
                                                : "IMU_STOP_FAIL");
    break;
  }
  case HitlCommandId::ReadNBytes: {
    if (line.argCount < 1u) {
      usbInterface.sendResponse("INVALID_ARGUMENT");
      return;
    }

    const std::optional<std::uint16_t> nBytes = parseInt(line.args[0]);
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

      // DataReady is edge-triggered; readSample() consumes the pending edge
      // atomically. Poll without consuming so readSample is the sole consumer.
      while (!imu.isDataReadyPending()) {
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
  case HitlCommandId::Reset: {
    const platform::Result r = imu.reset();
    usbInterface.sendResponse(platform::isOk(r) ? "IMU_RESET_OK"
                                                : "IMU_RESET_FAIL");
    break;
  }
  }
}

} // namespace hitl
