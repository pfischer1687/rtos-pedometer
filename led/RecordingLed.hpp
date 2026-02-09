/**
 * @file led/RecordingLed.hpp
 * @brief Recording status LED (on-board red LED).
 *
 * @details
 * Indicates whether IMU data recording is active.
 */

#ifndef LED_RECORDING_LED_HPP
#define LED_RECORDING_LED_HPP

#include "platform/Platform.hpp"

namespace led {

class RecordingLed {
public:
  RecordingLed() = default;

  /**
   * @brief Initialize GPIO for the recording LED.
   */
  platform::Result init();

  /**
   * @brief Turn recording indication on or off.
   *
   * @param active true = recording, false = idle
   *
   * @return Result
   */
  void setRecording(bool active);
};

} // namespace led

#endif // LED_RECORDING_LED_HPP
