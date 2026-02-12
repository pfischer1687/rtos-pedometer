/**
 * @file platform/Platform.cpp
 * @brief Implementation of HAL abstractions, timing, and watchdog.
 */

#include "platform/Platform.hpp"

namespace platform {

namespace {

class TickSource final : public ITickSource {
public:
  TickUs nowUs() const noexcept override { return 0; }
};

class Watchdog final : public IWatchdog {
public:
  Result start(uint32_t timeoutMs) noexcept override {
    (void)timeoutMs;
    return Result::Ok;
  }

  Result stop() noexcept override { return Result::Ok; }

  void kick() noexcept override {}
};

/**
 * @brief Static instances of tick source and watchdog.
 */
TickSource g_tickSource;
Watchdog g_watchdog;
} // anonymous namespace

ITickSource &tickSource() noexcept { return g_tickSource; }

IWatchdog &watchdog() noexcept { return g_watchdog; }

Result init() noexcept { return Result::Ok; }

} // namespace platform
