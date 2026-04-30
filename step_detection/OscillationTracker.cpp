/**
 * @file step_detection/OscillationTracker.cpp
 * @brief Slope sign-change peak detector with Rising/Falling state machine.
 */

#include "step_detection/OscillationTracker.hpp"
#include "platform/Platform.hpp"
#include <cstdio>
#include <optional>

namespace step_detection {

namespace {

/**
 * @brief Check if the slope crossed from positive to negative and is above the
 * hysteresis threshold.
 * @param sPrev The previous slope.
 * @param sCurr The current slope.
 * @param hyst The hysteresis threshold.
 * @return True if the slope crossed from positive to negative and is above the
 * hysteresis threshold.
 */
constexpr bool positiveToNegativePeak(float sPrev, float sCurr,
                                      float hyst) noexcept {
  if (!(sPrev > 0.0f) || !(sCurr < 0.0f)) {
    return false;
  }

  return (sPrev >= hyst) || ((-sCurr) >= hyst);
}

/**
 * @brief Check if the slope crossed from negative to positive and is above the
 * hysteresis threshold.
 * @param sPrev The previous slope.
 * @param sCurr The current slope.
 * @param hyst The hysteresis threshold.
 * @return True if the slope crossed from negative to positive and is above the
 * hysteresis threshold.
 */
constexpr bool negativeToPositiveTrough(float sPrev, float sCurr,
                                        float hyst) noexcept {
  return (sPrev < 0.0f) && (sCurr >= hyst);
}

} // namespace

StepDecision OscillationTracker::processSample(
    const signal_processing::ProcessedSample &sample) noexcept {
  if (!_havePrevMag) {
    _prevMag = sample.magnitude;
    _havePrevMag = true;
    return StepDecision{};
  }

  const float slope = sample.magnitude - _prevMag;

  if (!_havePrevSlope) {
    _prevSlope = slope;
    _prevMag = sample.magnitude;
    _havePrevSlope = true;
    if (slope >= SLOPE_HYSTERESIS_G) {
      _leg = OscLeg::Rising;
    }
    return StepDecision{};
  }

  const float sPrev = _prevSlope;
  const float sCurr = slope;

  StepDecision out{};

  switch (_leg) {
  case OscLeg::SeekRise:
    if (sCurr >= SLOPE_HYSTERESIS_G) {
      _leg = OscLeg::Rising;
    }
    break;

  case OscLeg::Rising:
    if (positiveToNegativePeak(sPrev, sCurr, SLOPE_HYSTERESIS_G)) {
      ++_dbg.peaks;
      if (!_haveLastEmit ||
          platform::elapsed(_lastStepEmitUs, sample.timestampUs) >=
              MIN_STEP_INTERVAL) {
        StepEvent ev{};
        ev.timestampUs = sample.timestampUs;
        ev.stepIndex = _stepIndex;
        ++_stepIndex;
        _lastStepEmitUs = sample.timestampUs;
        _haveLastEmit = true;
        ++_dbg.emitted;
        out.event = ev;
      } else {
        ++_dbg.rejected;
      }
      _leg = OscLeg::Falling;
    }
    break;

  case OscLeg::Falling:
    if (negativeToPositiveTrough(sPrev, sCurr, SLOPE_HYSTERESIS_G)) {
      _leg = OscLeg::Rising;
    }
    break;
  }

  _prevSlope = sCurr;
  _prevMag = sample.magnitude;

  return out;
}

std::size_t OscillationTracker::processBatch(
    const signal_processing::ProcessedSample *samples, std::size_t count,
    StepEvent *events, std::size_t maxEvents) noexcept {
  if (!samples || !events || count == 0u || maxEvents == 0u) {
    return 0u;
  }

  std::size_t written = 0u;
  for (std::size_t i = 0u; i < count && written < maxEvents; ++i) {
    const StepDecision d = processSample(samples[i]);
    if (d.event) {
      events[written++] = *d.event;
    }
  }

  return written;
}

void OscillationTracker::reset() noexcept {
  _prevMag = 0.0f;
  _prevSlope = 0.0f;
  _havePrevMag = false;
  _havePrevSlope = false;
  _leg = OscLeg::SeekRise;
  _lastStepEmitUs = 0u;
  _haveLastEmit = false;
  _stepIndex = 0u;
  _dbg = StepDetectionDebugStats{};
}

void OscillationTracker::resetDebugStats() noexcept {
  _dbg = StepDetectionDebugStats{};
}

std::size_t OscillationTracker::formatOscillationDebugReport(
    char *buf, std::size_t size) const noexcept {
  if (buf == nullptr || size == 0u) {
    return 0u;
  }

  const int n =
      std::snprintf(buf, size, "OSC peaks=%lu emitted=%lu rejected=%lu",
                    static_cast<unsigned long>(_dbg.peaks),
                    static_cast<unsigned long>(_dbg.emitted),
                    static_cast<unsigned long>(_dbg.rejected));
  if (n <= 0 || static_cast<std::size_t>(n) >= size) {
    return 0u;
  }

  return static_cast<std::size_t>(n);
}

} // namespace step_detection
