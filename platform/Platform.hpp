/**
 * @file platform/Platform.hpp
 * @brief HAL abstractions, timing, and watchdog for the motion monitoring
 * system.
 */

#ifndef PLATFORM_PLATFORM_HPP
#define PLATFORM_PLATFORM_HPP

#include <cstdint>

namespace platform {

static_assert(sizeof(uint32_t) == 4, "Unexpected uint32_t size");

/**
 * @enum Result
 * @brief Result of platform operations.
 */
enum class Result : uint8_t {
  Ok = 0,
  Error = 1,
  Busy = 2,
  Timeout = 3,
  InvalidState = 4,
  NotInitialized = 5,
  HardwareFault = 6,
  InvalidArgument = 7,
  DataNotReady = 8,
  NotSupported = 9,
};

/**
 * @brief Utility helper for checking if a Result is Ok.
 * @param r Result to check.
 * @return True if the result is Ok, false otherwise.
 */
constexpr bool isOk(Result r) noexcept { return r == Result::Ok; }

/**
 * @typedef TickUs
 * @brief High-resolution tick in microseconds (wraps per platform).
 */
using TickUs = uint32_t;

/**
 * @brief Compute elapsed time with wraparound safety.
 * @param start Start tick.
 * @param end End tick.
 * @return Elapsed time in microseconds.
 */
constexpr TickUs elapsed(TickUs start, TickUs end) noexcept {
  return static_cast<TickUs>(end - start);
}

/**
 * @interface ITimer
 * @brief Abstract timer interface.
 */
class ITimer {
public:
  virtual ~ITimer() = default;

  /**
   * @brief Get current microsecond tick.
   * @return Current microsecond tick.
   */
  virtual TickUs nowUs() const noexcept = 0;

  /**
   * @brief Delay in microseconds.
   * @param us Microseconds to delay.
   */
  virtual void delayUs(uint32_t us) noexcept = 0;

  /**
   * @brief Delay in milliseconds.
   * @param ms Milliseconds to delay.
   */
  virtual void delayMs(uint32_t ms) noexcept = 0;
};

/**
 * @interface IWatchdog
 * @brief Abstract watchdog interface.
 */
class IWatchdog {
public:
  virtual ~IWatchdog() = default;

  /**
   * @brief Start watchdog with given timeout in milliseconds.
   * @param timeoutMs Timeout in milliseconds.
   * @return Result.
   */
  virtual Result start(uint32_t timeoutMs) noexcept = 0;

  /**
   * @brief Stop watchdog.
   * @return Result.
   */
  virtual Result stop() noexcept = 0;

  /**
   * @brief Kick watchdog to prevent reset.
   */
  virtual void kick() noexcept = 0;
};

/**
 * @brief Get the timer.
 * @return Timer.
 */
ITimer &timer() noexcept;

/**
 * @brief Get the watchdog.
 * @return Watchdog.
 */
IWatchdog &watchdog() noexcept;

} // namespace platform

#endif /* PLATFORM_PLATFORM_HPP */
