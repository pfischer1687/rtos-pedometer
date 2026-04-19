/**
 * @file step_detection/StepDetector.cpp
 * @brief Walking MVP: 3-sample peak, slope gate, peak-amplitude threshold,
 * baseline estimate with damped upward tracking, warm-up, confidence heuristic.
 */

#include "step_detection/StepDetector.hpp"
#include <algorithm>
#include <cmath>

namespace step_detection {

namespace {

constexpr float kPlateauEps = 1.0e-5f;
/// Reference cadence for interval confidence when `minStepInterval` is small (µs).
constexpr platform::TickUs kExpectedStepPeriodUsDefault = 500000u;

void clampConfig(StepDetectorConfig &c) noexcept {
  c.baselineEmaAlpha = std::clamp(c.baselineEmaAlpha, 1.0e-4f, 0.5f);
  c.slopeMin = std::clamp(c.slopeMin, 0.0f, 2.0f);
  c.minValleyProminenceG = std::max(0.0f, c.minValleyProminenceG);
  c.minPeakProminenceG = std::max(0.0f, c.minPeakProminenceG);
  if (c.minStepIntervalMs == 0u) {
    c.minStepIntervalMs = 1u;
  }
  c.tuning.peakAmpAdaptGain = std::clamp(c.tuning.peakAmpAdaptGain, 1.0e-4f, 4.0f);
  c.tuning.baselineDamping = std::clamp(c.tuning.baselineDamping, 1.0e-4f, 1.0f);
  c.tuning.peakEmaAlpha = std::clamp(c.tuning.peakEmaAlpha, 1.0e-4f, 1.0f);
}

} // namespace

void StepDetector::setConfig(const StepDetectorConfig &config) noexcept {
  _config = config;
  clampConfig(_config);
  _minStepIntervalUs = _config.minStepIntervalMs * 1000u;
  _warmupSamples = _config.warmupSamples;
}

bool StepDetector::detectPeakShape() const noexcept {
  if (_historyCount < 3u) {
    return false;
  }
  return (_m0 < _m1 - kPlateauEps) && (_m1 > _m2 - kPlateauEps);
}

float StepDetector::adaptiveThreshold() const noexcept {
  const float adaptive =
      _havePeakAmp ? (_config.tuning.peakAmpAdaptGain * _peakAmpEma)
                   : _config.minPeakProminenceG;
  return std::max(_config.minPeakProminenceG, adaptive);
}

bool StepDetector::intervalGate(platform::TickUs peakTimeUs) const noexcept {
  if (_lastStepTimeUs == 0u) {
    return true;
  }
  const platform::TickUs dt = platform::elapsed(_lastStepTimeUs, peakTimeUs);
  return dt >= _minStepIntervalUs;
}

void StepDetector::updateBaselineEstimate(float newestMag) noexcept {
  if (!_haveBaselineEstimate) {
    _baselineEstimate = newestMag;
    _haveBaselineEstimate = true;
    return;
  }
  float input = newestMag;
  if (newestMag > _baselineEstimate) {
    const float d = _config.tuning.baselineDamping;
    input = _baselineEstimate + d * (newestMag - _baselineEstimate);
  }
  const float aB = _config.baselineEmaAlpha;
  _baselineEstimate = (1.0f - aB) * _baselineEstimate + aB * input;
}

void StepDetector::updatePeakAmplitudeEma(float peakValleyProminence) noexcept {
  const float a = _config.tuning.peakEmaAlpha;
  if (!_havePeakAmp) {
    _peakAmpEma = peakValleyProminence;
    _havePeakAmp = true;
    return;
  }
  _peakAmpEma = (1.0f - a) * _peakAmpEma + a * peakValleyProminence;
}

float StepDetector::computeConfidence(float rise, float fall, float valleyProminence,
                                      float baselineProminence, float threshold,
                                      platform::TickUs intervalDtUs,
                                      bool firstSinceLastStep) const noexcept {
  const float sm = std::max(_config.slopeMin, 1.0e-6f);
  const float slopeScore =
      std::clamp(std::min(rise, fall) / (2.0f * sm), 0.0f, 1.0f);

  const float mv = std::max(_config.minValleyProminenceG, 1.0e-6f);
  const float valleyScore =
      std::clamp(valleyProminence / (1.5f * mv), 0.0f, 1.0f);

  const float thr = std::max(threshold, 1.0e-6f);
  const float baselineScore = std::clamp(baselineProminence / thr, 0.0f, 1.0f);

  const platform::TickUs expectedUs =
      std::max(_minStepIntervalUs, kExpectedStepPeriodUsDefault);
  const float expected = static_cast<float>(expectedUs);
  const float dt = static_cast<float>(intervalDtUs);

  float intervalScore = 1.0f;
  if (!firstSinceLastStep) {
    const float error = std::fabs(dt - expected);
    intervalScore = std::clamp(1.0f - (error / expected), 0.0f, 1.0f);
  }

  return 0.25f * (slopeScore + valleyScore + baselineScore + intervalScore);
}

std::optional<StepEvent>
StepDetector::processSample(float magnitudeG, platform::TickUs timestampUs) noexcept {

  const bool inWarmup = (_warmupSamples > 0u);
  std::optional<StepEvent> out{};

  if (_historyCount == 0u) {
    _m0 = _m1 = _m2 = magnitudeG;
    _t0 = _t1 = _t2 = timestampUs;
    _historyCount = 1u;
    updateBaselineEstimate(_m2);
  } else if (_historyCount == 1u) {
    _m2 = magnitudeG;
    _t2 = timestampUs;
    _historyCount = 2u;
    updateBaselineEstimate(_m2);
  } else {
    if (_historyCount == 2u) {
      _m0 = _m1;
      _t0 = _t1;
      _m1 = _m2;
      _t1 = _t2;
      _m2 = magnitudeG;
      _t2 = timestampUs;
      _historyCount = 3u;
    } else {
      _m0 = _m1;
      _t0 = _t1;
      _m1 = _m2;
      _t1 = _t2;
      _m2 = magnitudeG;
      _t2 = timestampUs;
    }

    if (!inWarmup && detectPeakShape()) {
      const float rise = _m1 - _m0;
      const float fall = _m1 - _m2;
      if ((rise >= _config.slopeMin) && (fall >= _config.slopeMin)) {
        const float baselineEstimateBefore = _baselineEstimate;
        const float valleyProminence = _m1 - std::min(_m0, _m2);
        const float baselineProminence = _m1 - baselineEstimateBefore;
        const float thresh = adaptiveThreshold();

        if ((valleyProminence >= _config.minValleyProminenceG) &&
            (baselineProminence >= thresh) && intervalGate(_t1)) {
          updatePeakAmplitudeEma(valleyProminence);

          const bool firstSinceLastStep = (_lastStepTimeUs == 0u);
          const platform::TickUs intervalDt =
              firstSinceLastStep
                  ? 0u
                  : platform::elapsed(_lastStepTimeUs, _t1);

          StepEvent ev{};
          ev.timestampUs = _t1;
          ev.stepIndex = _stepIndex;
          ev.confidence = computeConfidence(rise, fall, valleyProminence, baselineProminence,
                                            thresh, intervalDt, firstSinceLastStep);
          ++_stepIndex;
          _lastStepTimeUs = _t1;
          out = ev;
        }
      }
    }

    updateBaselineEstimate(_m2);
  }

  if (_warmupSamples > 0u) {
    --_warmupSamples;
  }
  return out;
}

size_t StepDetector::processBatch(const signal_processing::ProcessedSample *samples,
                                  size_t count, StepEvent *events,
                                  size_t maxEvents) noexcept {
  if (!samples || !events || count == 0u || maxEvents == 0u) {
    return 0u;
  }
  size_t written = 0u;
  for (size_t i = 0u; i < count && written < maxEvents; ++i) {
    if (const auto ev = processSample(samples[i].magnitude, samples[i].timestampUs)) {
      events[written++] = *ev;
    }
  }
  return written;
}

void StepDetector::reset() noexcept {
  _m0 = _m1 = _m2 = 0.0f;
  _t0 = _t1 = _t2 = 0u;
  _historyCount = 0u;
  _baselineEstimate = 0.0f;
  _haveBaselineEstimate = false;
  _peakAmpEma = 0.0f;
  _havePeakAmp = false;
  _warmupSamples = _config.warmupSamples;
  _lastStepTimeUs = 0u;
  _stepIndex = 0u;
}

} // namespace step_detection
