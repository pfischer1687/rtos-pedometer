/**
 * @file led/RecordingLed.hpp
 * @brief Recording status LED (on-board red LED).
 *
 * @details
 * Indicates whether step counting is active.
 */

#ifndef LED_RECORDING_LED_HPP
#define LED_RECORDING_LED_HPP

#include <cstdint>

namespace led {

/**
 * @brief Visual state for the recording LED.
 */
enum class LedState : uint8_t { Idle, Active };

/**
 * @brief Recording LED.
 */
class RecordingLed {
public:
  RecordingLed() = default;

  /**
   * @brief Apply a snapshot of application LED state to the GPIO sink.
   * @param state Value read from LED thread only.
   */
  void apply(LedState state) noexcept;
};

} // namespace led

#endif // LED_RECORDING_LED_HPP
