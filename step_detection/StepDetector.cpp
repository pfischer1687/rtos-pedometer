/**
 * @file step_detection/StepDetector.cpp
 * @brief Step detection from processed acceleration magnitudes.
 */

#include "step_detection/StepDetector.hpp"
#include <algorithm>
#include <cmath>

namespace step_detection {

namespace {

constexpr uint32_t MS_TO_US = 1000u;

/**
 * @brief Clamp the detector configuration.
 * @param c Configuration.
 */
void clampConfig(StepDetectorConfig &c) noexcept {
  c.baselineEmaAlpha = std::clamp(c.baselineEmaAlpha, 1.0e-4f, 0.5f);
  c.slopeMin = std::clamp(c.slopeMin, 0.0f, 2.0f);
  c.minValleyProminenceG = std::max(0.0f, c.minValleyProminenceG);
  c.minPeakProminenceG = std::max(0.0f, c.minPeakProminenceG);
  if (c.minStepIntervalMs == 0u) {
    c.minStepIntervalMs = 1u;
  }
  c.tuning.peakAmpAdaptGain =
      std::clamp(c.tuning.peakAmpAdaptGain, 1.0e-4f, 4.0f);
  c.tuning.baselineDamping =
      std::clamp(c.tuning.baselineDamping, 1.0e-4f, 1.0f);
  c.tuning.peakEmaAlpha = std::clamp(c.tuning.peakEmaAlpha, 1.0e-4f, 1.0f);
  c.tuning.adaptiveBlendT = std::clamp(c.tuning.adaptiveBlendT, 0.0f, 1.0f);

  c.heuristics.plateauEps = std::max(0.0f, c.heuristics.plateauEps);
  c.heuristics.confidenceSlopeScale =
      std::max(1.0e-6f, c.heuristics.confidenceSlopeScale);
  c.heuristics.confidenceValleyScale =
      std::max(1.0e-6f, c.heuristics.confidenceValleyScale);
  c.heuristics.confidenceWeightSlope =
      std::max(0.0f, c.heuristics.confidenceWeightSlope);
  c.heuristics.confidenceWeightValley =
      std::max(0.0f, c.heuristics.confidenceWeightValley);
  c.heuristics.confidenceWeightBaseline =
      std::max(0.0f, c.heuristics.confidenceWeightBaseline);
  c.heuristics.confidenceWeightInterval =
      std::max(0.0f, c.heuristics.confidenceWeightInterval);
  c.heuristics.intervalScoreEpsilonUs =
      std::max(1.0e-3f, c.heuristics.intervalScoreEpsilonUs);
}

inline void recordReject(StepDetectorDebugStats &dbg,
                         StepRejectReason r) noexcept {
  dbg.lastRejectReason = r;

  switch (r) {
  case StepRejectReason::NoPeak:
    ++dbg.rejectNoPeak;
    break;
  case StepRejectReason::Slope:
    ++dbg.rejectSlope;
    break;
  case StepRejectReason::Valley:
    ++dbg.rejectValley;
    break;
  case StepRejectReason::Baseline:
    ++dbg.rejectBaseline;
    break;
  case StepRejectReason::Interval:
    ++dbg.rejectInterval;
    break;
  default:
    break;
  }
}

} // anonymous namespace

void StepDetector::setConfig(const StepDetectorConfig &config) noexcept {
  _config = config;
  clampConfig(_config);
  _minStepIntervalUs = _config.minStepIntervalMs * MS_TO_US;
  _warmupSamples = _config.warmupSamples;
}

const StepDetectorDebugStats &StepDetector::getDebugStats() const noexcept {
  return _dbg;
}

void StepDetector::resetDebugStats() noexcept {
  _dbg = StepDetectorDebugStats{};
}

void StepDetector::advanceRing(float magnitudeG,
                               platform::TickUs timestampUs) noexcept {
  _idx = static_cast<uint8_t>((static_cast<uint32_t>(_idx) + 1u) % RING_SIZE);
  _m[_idx] = magnitudeG;
  _t[_idx] = timestampUs;
  if (_fill < RING_SIZE) {
    ++_fill;
  }
}

bool StepDetector::readWindow(float &m0, float &m1, float &m2,
                              platform::TickUs &tPeak) const noexcept {
  if (_fill < RING_SIZE) {
    return false;
  }

  const uint8_t iNew = _idx;
  const uint8_t iOld =
      static_cast<uint8_t>((static_cast<uint32_t>(_idx) + 1u) % RING_SIZE);
  const uint8_t iMid =
      static_cast<uint8_t>((static_cast<uint32_t>(_idx) + 2u) % RING_SIZE);

  m0 = _m[iOld];
  m1 = _m[iMid];
  m2 = _m[iNew];
  tPeak = _t[iMid];

  return true;
}

float StepDetector::adaptiveThreshold() const noexcept {
  const float lo = _config.minPeakProminenceG;
  const float hi =
      _havePeakAmp ? (_config.tuning.peakAmpAdaptGain * _peakAmpEma) : lo;
  const float t = _config.tuning.adaptiveBlendT;
  return (1.0f - t) * lo + t * hi;
}

bool StepDetector::intervalGate(platform::TickUs peakTimeUs) const noexcept {
  if (!_haveLastStep) {
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

float StepDetector::computeConfidence(
    float rise, float fall, float valleyProminence, float baselineProminence,
    float threshold, platform::TickUs intervalDtUs,
    bool havePriorStepInterval) const noexcept {
  const StepHeuristics &h = _config.heuristics;
  const float sm = std::max(_config.slopeMin, 1.0e-6f);
  const float slopeScore = std::clamp(
      std::min(rise, fall) / (h.confidenceSlopeScale * sm), 0.0f, 1.0f);

  const float mv = std::max(_config.minValleyProminenceG, 1.0e-6f);
  const float valleyScore =
      std::clamp(valleyProminence / (h.confidenceValleyScale * mv), 0.0f, 1.0f);

  const float thr = std::max(threshold, 1.0e-6f);
  const float baselineScore = std::clamp(baselineProminence / thr, 0.0f, 1.0f);

  const platform::TickUs expectedUs =
      std::max(_minStepIntervalUs, h.intervalConfidenceMinPeriodUs);
  const float expectedEff =
      std::max(static_cast<float>(expectedUs), h.intervalScoreEpsilonUs);
  const float dt = static_cast<float>(intervalDtUs);

  float intervalScore = 1.0f;
  if (havePriorStepInterval) {
    const float error = std::fabs(dt - static_cast<float>(expectedUs));
    intervalScore = std::exp(-error / expectedEff);
    intervalScore = std::clamp(intervalScore, 0.0f, 1.0f);
  }

  const float ws = h.confidenceWeightSlope;
  const float wv = h.confidenceWeightValley;
  const float wb = h.confidenceWeightBaseline;
  const float wi = h.confidenceWeightInterval;
  const float wsum = ws + wv + wb + wi;
  const float norm = (wsum > 0.0f) ? (1.0f / wsum) : 0.25f;
  return norm * (ws * slopeScore + wv * valleyScore + wb * baselineScore +
                 wi * intervalScore);
}

StepDecision
StepDetector::processSample(float magnitudeG,
                            platform::TickUs timestampUs) noexcept {
  advanceRing(magnitudeG, timestampUs);
  const float newest = _m[_idx];

  if (_warmupSamples > 0u) {
    updateBaselineEstimate(newest);
    --_warmupSamples;
    return StepDecision{.event = std::nullopt,
                        .reason = StepRejectReason::None};
  }
  if (_fill < RING_SIZE) {
    updateBaselineEstimate(newest);
    return StepDecision{.event = std::nullopt,
                        .reason = StepRejectReason::None};
  }

  float m0 = 0.0f;
  float m1 = 0.0f;
  float m2 = 0.0f;
  platform::TickUs tPeak = 0u;
  if (!readWindow(m0, m1, m2, tPeak)) {
    updateBaselineEstimate(newest);
    return StepDecision{.event = std::nullopt,
                        .reason = StepRejectReason::None};
  }

  const float eps = _config.heuristics.plateauEps;
  const bool isPeakCandidate = (m0 < m1 - eps) && (m1 >= m2 - eps);
  if (!isPeakCandidate) {
    recordReject(_dbg, StepRejectReason::NoPeak);
    updateBaselineEstimate(newest);
    return StepDecision{.event = std::nullopt,
                        .reason = StepRejectReason::NoPeak};
  }

  ++_dbg.peaks;
  const float rise = m1 - m0;
  const float fall = m1 - m2;
  const bool hasMinSlope =
      (rise >= _config.slopeMin) && (fall >= _config.slopeMin);
  if (!hasMinSlope) {
    recordReject(_dbg, StepRejectReason::Slope);
    updateBaselineEstimate(newest);
    return StepDecision{.event = std::nullopt,
                        .reason = StepRejectReason::Slope};
  }

  const float baselineEstimateBefore = _baselineEstimate;
  const float valleyProminence = m1 - std::min(m0, m2);
  const float baselineProminence = m1 - baselineEstimateBefore;
  const float thresh = adaptiveThreshold();

  if (valleyProminence < _config.minValleyProminenceG) {
    recordReject(_dbg, StepRejectReason::Valley);
    updateBaselineEstimate(newest);
    return StepDecision{.event = std::nullopt,
                        .reason = StepRejectReason::Valley};
  }

  if (baselineProminence < thresh) {
    recordReject(_dbg, StepRejectReason::Baseline);
    updateBaselineEstimate(newest);
    return StepDecision{.event = std::nullopt,
                        .reason = StepRejectReason::Baseline};
  }

  if (!intervalGate(tPeak)) {
    recordReject(_dbg, StepRejectReason::Interval);
    updateBaselineEstimate(newest);
    return StepDecision{.event = std::nullopt,
                        .reason = StepRejectReason::Interval};
  }

  const bool havePrior = _haveLastStep;
  const platform::TickUs intervalDt =
      havePrior ? platform::elapsed(_lastStepTimeUs, tPeak) : 0u;

  updatePeakAmplitudeEma(valleyProminence);

  StepEvent ev{};
  ev.timestampUs = tPeak;
  ev.stepIndex = _stepIndex;
  ev.confidence =
      computeConfidence(rise, fall, valleyProminence, baselineProminence,
                        thresh, intervalDt, havePrior);
  ++_stepIndex;
  _lastStepTimeUs = tPeak;
  _haveLastStep = true;
  ++_dbg.emitted;

  updateBaselineEstimate(newest);
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
  for (float &x : _m) {
    x = 0.0f;
  }

  for (platform::TickUs &x : _t) {
    x = 0u;
  }

  _idx = 0u;
  _fill = 0u;
  _baselineEstimate = 0.0f;
  _haveBaselineEstimate = false;
  _peakAmpEma = 0.0f;
  _havePeakAmp = false;
  _warmupSamples = _config.warmupSamples;
  _haveLastStep = false;
  _lastStepTimeUs = 0u;
  _stepIndex = 0u;
}

} // namespace step_detection
