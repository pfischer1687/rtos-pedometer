/**
 * @file: led/RecordingLed.cpp
 *
 * @brief: Recording status LED (on-board red LED) implementation.
 */

#include "led/RecordingLed.hpp"

namespace led {

platform::Result RecordingLed::init() { return platform::Result::Ok; }

void RecordingLed::setRecording(bool active) {}

} // namespace led
