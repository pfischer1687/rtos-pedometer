/**
 * @file platform/Platform.cpp
 * @brief Implementation of HAL abstractions, timing, and watchdog (Mbed CE,
 * Nucleo-F767ZI).
 */

#include "platform/Platform.hpp"
#include "drivers/Watchdog.h"
#include "mbed.h"
#include <cstdint>

namespace platform {

namespace {

/**
 * @brief Number of microseconds per millisecond.
 */
constexpr uint32_t US_PER_MS = 1000u;

class TickSourceImpl final : public ITickSource {
public:
  TickUs nowUs() const noexcept override { return us_ticker_read(); }
};

class WatchdogImpl final : public IWatchdog {
public:
  Result start(uint32_t timeoutMs) noexcept override {
    mbed::Watchdog &wd = mbed::Watchdog::get_instance();

    if (wd.is_running()) {
      return Result::Ok;
    }
    if (timeoutMs == 0) {
      return Result::InvalidArgument;
    }

    return wd.start(timeoutMs) ? Result::Ok : Result::HardwareFault;
  }

  Result stop() noexcept override {
    mbed::Watchdog &wd = mbed::Watchdog::get_instance();

    if (!wd.is_running()) {
      return Result::Ok;
    }

    return wd.stop() ? Result::Ok : Result::HardwareFault;
  }

  void kick() noexcept override { mbed::Watchdog::get_instance().kick(); }
};

TickSourceImpl g_tickSource;
WatchdogImpl g_watchdog;

} // anonymous namespace

ITickSource &tickSource() noexcept { return g_tickSource; }

IWatchdog &watchdog() noexcept { return g_watchdog; }

Result init() noexcept { return Result::Ok; }

void delayUs(uint32_t us) noexcept {
  while (us > 0u) {
    uint32_t chunk = (us > INT32_MAX) ? INT32_MAX : us;
    wait_us(static_cast<int>(chunk));
    us -= chunk;
  }
}

void delayMs(uint32_t ms) noexcept {
  while (ms > 0u) {
    uint32_t chunk = (ms > INT32_MAX / US_PER_MS) ? INT32_MAX / US_PER_MS : ms;
    wait_us(static_cast<int>(chunk * US_PER_MS));
    ms -= chunk;
  }
}

} // namespace platform
