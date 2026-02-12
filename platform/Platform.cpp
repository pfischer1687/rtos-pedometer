/**
 * @file platform/Platform.cpp
 * @brief Implementation of HAL abstractions, timing, and watchdog (Mbed CE, Nucleo-F767ZI).
 */

#include "platform/Platform.hpp"
#include "hal/us_ticker_api.h"
#include "mbed.h"

#ifdef DEVICE_WATCHDOG
#include "drivers/Watchdog.h"
#endif

namespace platform {

namespace {

class TickSourceImpl final : public ITickSource {
public:
    TickUs nowUs() const noexcept override
    {
        return us_ticker_read();
    }
};

#ifdef DEVICE_WATCHDOG
class WatchdogImpl final : public IWatchdog {
public:
    Result start(uint32_t timeoutMs) noexcept override
    {
        mbed::Watchdog& wd = mbed::Watchdog::get_instance();
        if (wd.is_running()) {
            return Result::Ok;
        }
        return wd.start(timeoutMs) ? Result::Ok : Result::Error;
    }

    Result stop() noexcept override
    {
        mbed::Watchdog& wd = mbed::Watchdog::get_instance();
        return wd.stop() ? Result::Ok : Result::Error;
    }

    void kick() noexcept override
    {
        mbed::Watchdog::get_instance().kick();
    }
};
#else
class WatchdogImpl final : public IWatchdog {
public:
    Result start(uint32_t timeoutMs) noexcept override
    {
        (void)timeoutMs;
        return Result::Error;
    }

    Result stop() noexcept override
    {
        return Result::Ok;
    }

    void kick() noexcept override
    {
    }
};
#endif

TickSourceImpl g_tickSource;
WatchdogImpl g_watchdog;

} // anonymous namespace

ITickSource& tickSource() noexcept
{
    return g_tickSource;
}

IWatchdog& watchdog() noexcept
{
    return g_watchdog;
}

Result init() noexcept
{
    return Result::Ok;
}

void delayUs(uint32_t us) noexcept
{
    wait_us(static_cast<int>(us));
}

void delayMs(uint32_t ms) noexcept
{
    while (ms > 0u) {
        uint32_t chunk = (ms > 1000u) ? 1000u : ms;
        wait_us(static_cast<int>(chunk * 1000u));
        ms -= chunk;
    }
}

} // namespace platform
