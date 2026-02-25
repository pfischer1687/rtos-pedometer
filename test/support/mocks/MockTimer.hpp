/**
 * @file test/support/mocks/MockTimer.hpp
 * @brief Mock for platform::ITimer.
 */

#ifndef TEST_SUPPORT_MOCKS_MOCKTIMER_HPP
#define TEST_SUPPORT_MOCKS_MOCKTIMER_HPP

#include "platform/Platform.hpp"
#include <cstdint>

namespace test_support {

/**
 * @class MockTimer
 * @brief Implements platform::ITimer with tick injection.
 */
class MockTimer : public platform::ITimer {
public:
  MockTimer() = default;

  /**
   * @brief Constructor with initial tick.
   * @param initialTickUs Initial tick.
   */
  explicit MockTimer(platform::TickUs initialTickUs) : tickUs_(initialTickUs) {}

  ~MockTimer() override = default;
  platform::TickUs nowUs() const noexcept override { return tickUs_; }

  /**
   * @brief Advance tick by specified amount.
   * @param deltaUs Microseconds to advance.
   */
  void advanceUs(platform::TickUs deltaUs) noexcept { tickUs_ += deltaUs; }

  /**
   * @brief Advance tick by specified amount.
   * @param deltaMs Milliseconds to advance.
   */
  void advanceMs(uint32_t deltaMs) noexcept {
    tickUs_ += static_cast<platform::TickUs>(deltaMs) * 1000u;
  }

  void delayUs(uint32_t us) noexcept override { advanceUs(us); }
  void delayMs(uint32_t ms) noexcept override { advanceMs(ms); }

  /**
   * @brief Set current tick (deterministic).
   * @param us Current tick.
   */
  void setNowUs(platform::TickUs us) noexcept { tickUs_ = us; }

  /**
   * @brief Reset to zero.
   */
  void reset() noexcept { tickUs_ = 0u; }

private:
  platform::TickUs tickUs_{0u};
};

} // namespace test_support

#endif /* TEST_SUPPORT_MOCKS_MOCKTIMER_HPP */
