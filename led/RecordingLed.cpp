/**
 * @file led/RecordingLed.cpp
 * @brief Recording status LED (on-board red LED) implementation.
 */

#include "led/RecordingLed.hpp"
#include "platform/Gpio.hpp"

namespace led {

void RecordingLed::apply(LedState state) noexcept {
  const bool on = (state == LedState::Active);
  platform::ledOutput().set(on);
}

} // namespace led
