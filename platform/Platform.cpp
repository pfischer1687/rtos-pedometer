/**
 * @file platform/Platform.cpp
 * @brief Implementation of HAL abstractions, timing, and watchdog (Mbed CE,
 * Nucleo-F767ZI).
 */

#include "platform/Platform.hpp"
#include "drivers/Watchdog.h"
#include "mbed.h"
#include <cstdint>
#include <limits>

namespace platform {

namespace {

/**
 * @brief Number of microseconds per millisecond.
 */
constexpr uint32_t US_PER_MS = 1000u;

/**
 * @brief Maximum delay in microseconds that can be represented by an int32_t.
 */
constexpr uint32_t MAX_DELAY_US_INT32 =
    static_cast<uint32_t>(std::numeric_limits<int32_t>::max());

class TimerImpl final : public ITimer {
public:
  TickUs nowUs() const noexcept override { return us_ticker_read(); }

  void delayUs(uint32_t us) noexcept override {
    while (us > 0u) {
      uint32_t chunk = (us > MAX_DELAY_US_INT32) ? MAX_DELAY_US_INT32 : us;
      wait_us(static_cast<int>(chunk));
      us -= chunk;
    }
  }
  
  void delayMs(uint32_t ms) noexcept override {
    while (ms > 0u) {
      uint32_t chunk = (ms > MAX_DELAY_US_INT32 / US_PER_MS) ? MAX_DELAY_US_INT32 / US_PER_MS : ms;
      delayUs(chunk * US_PER_MS);
      ms -= chunk;
    }
  }
};

class WatchdogImpl final : public IWatchdog {
public:
  Result start(uint32_t timeoutMs) noexcept override {
    if (timeoutMs == 0) {
      return Result::InvalidArgument;
    }

    mbed::Watchdog &wd = mbed::Watchdog::get_instance();

    if (wd.is_running()) {
      return Result::Ok;
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

TimerImpl g_timer;
WatchdogImpl g_watchdog;

} // anonymous namespace

ITimer &timer() noexcept { return g_timer; }

IWatchdog &watchdog() noexcept { return g_watchdog; }

} // namespace platform
