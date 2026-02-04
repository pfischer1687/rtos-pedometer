/** \file platform/Platform.hpp
 *  HAL abstractions, timing, and watchdog for the motion monitoring system.
 *  Ownership: platform layer; no application or algorithm logic.
 */

#ifndef PLATFORM_PLATFORM_HPP
#define PLATFORM_PLATFORM_HPP

#include <cstdint>

namespace platform {

/** High-resolution tick in microseconds (wraps per platform). */
using TickUs = uint32_t;

/** Result of platform operations. */
enum class Result : int {
    Ok = 0,
    Error,
    Busy,
    Timeout,
};

/** Microsecond tick source (e.g. us_ticker). */
class TickSource {
public:
    /** Return current tick in microseconds. */
    static TickUs nowUs();
};

/** Watchdog timer: kick to prevent reset; platform owns hardware. */
class Watchdog {
public:
    /** Start watchdog with given timeout in milliseconds. */
    static Result start(uint32_t timeoutMs);

    /** Stop the watchdog. */
    static Result stop();

    /** Kick the watchdog to prevent reset. */
    static void kick();
};

/** Platform initialization (clocks, pins, HAL). Call once at startup. */
Result init();

} // namespace platform

#endif /* PLATFORM_PLATFORM_HPP */
