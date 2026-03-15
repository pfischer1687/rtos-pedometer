/**
 * @file test/support/mocks/MockGpioInput.hpp
 * @brief Multi-instance safe mock for platform::IDataReadyInput (data-ready
 * interrupt GPIO).
 */

#ifndef TEST_SUPPORT_MOCKS_MOCKGPIOINPUT_HPP
#define TEST_SUPPORT_MOCKS_MOCKGPIOINPUT_HPP

#include "platform/Gpio.hpp"

namespace test_support {

/**
 * @class MockGpioInput
 * @brief Implements platform::IDataReadyInput with stored callback/context and
 * triggerCallback() for tests.
 */
class MockGpioInput : public platform::IDataReadyInput {
public:
  MockGpioInput() = default;
  ~MockGpioInput() override = default;

  void setCallback(platform::DataReadyCallback cb,
                   void *ctx) noexcept override {
    callback_ = cb;
    context_ = ctx;
  }

  void enable() noexcept override { enabled_ = true; }

  void disable() noexcept override { enabled_ = false; }

  /**
   * @brief Simulate data-ready interrupt: invoke the stored callback with the
   * stored context.
   */
  void triggerCallback() noexcept {
    if (callback_ != nullptr) {
      callback_(context_);
    }
  }

  /**
   * @brief Whether enable() was last called (no disable after).
   */
  bool isEnabled() const noexcept { return enabled_; }

  /**
   * @brief Clear stored callback and context (e.g. for test reset).
   */
  void clearCallback() noexcept {
    callback_ = nullptr;
    context_ = nullptr;
  }

private:
  platform::DataReadyCallback callback_{nullptr};
  void *context_{nullptr};
  bool enabled_{false};
};

} // namespace test_support

#endif /* TEST_SUPPORT_MOCKS_MOCKGPIOINPUT_HPP */
