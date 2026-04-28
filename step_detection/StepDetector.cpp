/**
 * @file step_detection/StepDetector.cpp
 * @brief Step detection from processed acceleration magnitudes.
 */

#include "step_detection/StepDetector.hpp"
#include <algorithm>

namespace step_detection {

namespace {

void clampConfig(StepDetectorConfig &c) noexcept {
  c.baselineEmaAlpha = std::clamp(c.baselineEmaAlpha, 1.0e-4f, 0.5f);
  c.tuning.baselineDamping =
      std::clamp(c.tuning.baselineDamping, 1.0e-4f, 1.0f);
  c.heuristics.plateauEps = std::max(0.0f, c.heuristics.plateauEps);
}

inline void recordNoPeak(StepDetectorDebugStats &dbg) noexcept {
  ++dbg.rejectNoPeak;
}

} // anonymous namespace

void StepDetector::setConfig(const StepDetectorConfig &config) noexcept {
  _config = config;
  clampConfig(_config);
  _warmupSamples = _config.warmupSamples;
}

const StepDetectorDebugStats &StepDetector::getDebugStats() const noexcept {
  return _dbg;
}

void StepDetector::resetDebugStats() noexcept {
  _dbg = StepDetectorDebugStats{};
}

float StepDetector::filterMagnitude(float magnitudeG) noexcept {
  const float hp = HP_ALPHA * (_hpState + magnitudeG - _prevInput);
  _hpState = hp;
  _prevInput = magnitudeG;

  _lpState = _lpState + LP_ALPHA * (hp - _lpState);
  return _lpState;
}

void StepDetector::shiftFilteredSample(float filtered,
                                       platform::TickUs timestampUs) noexcept {
  _prevPrevFiltered = _prevFiltered;
  _prevFiltered = filtered;
  _prevTimestampUs = timestampUs;
  if (_filteredFill < 2u) {
    ++_filteredFill;
  }
}

StepDecision
StepDetector::processSample(float magnitudeG,
                            platform::TickUs timestampUs) noexcept {
  const float filtered = filterMagnitude(magnitudeG);

  if (_warmupSamples > 0u) {
    --_warmupSamples;
    shiftFilteredSample(filtered, timestampUs);
    return StepDecision{.event = std::nullopt, .reason = StepRejectReason::None};
  }
  if (_filteredFill < 2u) {
    shiftFilteredSample(filtered, timestampUs);
    return StepDecision{.event = std::nullopt, .reason = StepRejectReason::None};
  }

  const bool isPeak =
      (_prevFiltered > _prevPrevFiltered) && (_prevFiltered > filtered);
  if (!isPeak) {
    recordNoPeak(_dbg);
    shiftFilteredSample(filtered, timestampUs);
    return StepDecision{.event = std::nullopt, .reason = StepRejectReason::None};
  }

  StepEvent ev{};
  ev.timestampUs = _prevTimestampUs;
  ev.stepIndex = _stepIndex++;
  ev.confidence = 0.0f;
  ++_dbg.peaks;
  ++_dbg.emitted;

  shiftFilteredSample(filtered, timestampUs);
  return StepDecision{.event = ev, .reason = StepRejectReason::None};
}

size_t
StepDetector::processBatch(const signal_processing::ProcessedSample *samples,
                           size_t count, StepEvent *events,
                           size_t maxEvents) noexcept {
  if (!samples || !events || count == 0u || maxEvents == 0u) {
    return 0u;
  }

  size_t written = 0u;
  for (size_t i = 0u; i < count && written < maxEvents; ++i) {
    const StepDecision d =
        processSample(samples[i].magnitude, samples[i].timestampUs);
    if (d.event) {
      events[written++] = *d.event;
    }
  }

  return written;
}

void StepDetector::reset() noexcept {
  _hpState = 0.0f;
  _lpState = 0.0f;
  _prevInput = 0.0f;
  _prevFiltered = 0.0f;
  _prevPrevFiltered = 0.0f;
  _prevTimestampUs = 0u;
  _filteredFill = 0u;
  _warmupSamples = _config.warmupSamples;

  _stepIndex = 0u;
}

} // namespace step_detection
