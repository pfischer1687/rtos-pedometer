/**
 * @file step_detection/OscillationTracker.cpp
 * @brief Implementation of the step detection module.
 */

#include "step_detection/OscillationTracker.hpp"
#include <cmath>
#include <cstdio>

namespace step_detection {

namespace {

constexpr float S_PER_US = 1.0e-6f;

// NOTE: There is logic below dependent on this number being 3.
constexpr std::size_t NUM_SAMPLES_FOR_PEAK = 3u;

[[nodiscard]] constexpr bool is_idle(float dt_s, bool have_last_step,
                                     float idle_threshold_s) noexcept {
  return have_last_step && (dt_s > idle_threshold_s);
}

} // namespace

OscillationTracker::OscillationTracker(OscillationTrackerConfig config) noexcept
    : _cfg(config) {
  const std::size_t minRingSize = _cfg.quiet_pre_n + NUM_SAMPLES_FOR_PEAK;
  _ringCap = (minRingSize <= MAX_QUIET_RING) ? minRingSize : MAX_QUIET_RING;
  _xdRing.fill(0.0f);
}

bool OscillationTracker::passesRefractory(float dt_s) const noexcept {
  if (!_haveLastStep) {
    return true;
  }

  return dt_s >= _cfg.min_step_dt;
}

bool OscillationTracker::passesQuietGate(std::size_t sampleIndex,
                                         float dt_s) const noexcept {
  if (!_acceptedAny) {
    return true;
  }
  if (!is_idle(dt_s, _haveLastStep, _cfg.idle_threshold_s)) {
    return true;
  }
  if (sampleIndex < 2u) {
    return true;
  }

  const std::size_t peakIndex = sampleIndex - 1u;
  const std::size_t quietWindowEnd = peakIndex - 1u;
  const std::size_t quietWindowStart =
      (peakIndex > _cfg.quiet_pre_n) ? peakIndex - _cfg.quiet_pre_n : 0u;
  if (quietWindowEnd < quietWindowStart) {
    return true;
  }

  const std::size_t count = quietWindowEnd - quietWindowStart + 1;
  if (count < NUM_SAMPLES_FOR_PEAK) {
    return true;
  }

  float sum = 0.0f;
  for (std::size_t j = quietWindowStart; j <= quietWindowEnd; ++j) {
    const std::size_t idx = j % _ringCap;
    sum += std::fabs(_xdRing[idx]);
  }

  return (sum / static_cast<float>(count)) >= _cfg.quiet_abs;
}

bool OscillationTracker::passesShoulderFilter(float xd_im1,
                                              float dt_s) const noexcept {
  if (!is_idle(dt_s, _haveLastStep, _cfg.idle_threshold_s)) {
    return true;
  }
  if (_prevPeakValue <= 0.0f) {
    return true;
  }

  return std::fabs(xd_im1) >= (_cfg.peak_consistency_ratio * _prevPeakValue);
}

bool OscillationTracker::acceptPeak(std::size_t sampleIndex, float dt_s,
                                    float xd_im1) const noexcept {
  return passesRefractory(dt_s) && passesQuietGate(sampleIndex, dt_s) &&
         passesShoulderFilter(xd_im1, dt_s);
}

std::optional<StepEvent> OscillationTracker::processSample(
    const signal_processing::ProcessedSample &sample) noexcept {
  const float scaledMag = sample.magnitude * _cfg.mag_scale;
  float detrended = 0.0f;

  if (!_haveBaselineSeed) {
    _baseline = scaledMag;
    _haveBaselineSeed = true;
  } else {
    const float a = _cfg.ema_alpha;
    _baseline = a * scaledMag + (1.0f - a) * _baseline;
    detrended = scaledMag - _baseline;
  }

  if (_ringCap > 0u) {
    _xdRing[_sampleIndex % _ringCap] = detrended;
  }

  if (_sampleIndex < 2u || _ringCap < 3u) {
    _ts_im1 = sample.timestampUs;
    ++_sampleIndex;
    return std::nullopt;
  }

  const float xd_im2 = _xdRing[(_sampleIndex - 2u) % _ringCap];
  const float xd_im1 = _xdRing[(_sampleIndex - 1u) % _ringCap];
  const float xd_i = _xdRing[_sampleIndex % _ringCap];

  const bool isLocalPeak = (xd_im2 < xd_im1) && (xd_im1 > xd_i);
  if (!isLocalPeak || std::fabs(xd_im1) < _cfg.peak_threshold) {
    _ts_im1 = sample.timestampUs;
    ++_sampleIndex;
    return std::nullopt;
  }

  ++_dbg.peaks;

  const platform::TickUs peakTs = _ts_im1;
  const float dt_s =
      _haveLastStep
          ? static_cast<float>(platform::elapsed(_lastStepTsUs, peakTs)) *
                S_PER_US
          : 1.0e9f;

  if (!acceptPeak(_sampleIndex, dt_s, xd_im1)) {
    ++_dbg.rejected;
    _ts_im1 = sample.timestampUs;
    ++_sampleIndex;
    return std::nullopt;
  }

  StepEvent ev{};
  ev.timestampUs = peakTs;
  ev.stepIndex = _stepIndex++;

  ++_dbg.emitted;
  _lastStepTsUs = peakTs;
  _haveLastStep = true;
  _acceptedAny = true;
  _prevPeakValue = std::fabs(xd_im1);
  _ts_im1 = sample.timestampUs;
  ++_sampleIndex;

  return ev;
}

std::size_t OscillationTracker::processBatch(
    const signal_processing::ProcessedSample *samples, std::size_t count,
    StepEvent *events, std::size_t maxEvents) noexcept {
  if (!samples || !events || count == 0u || maxEvents == 0u) {
    return 0u;
  }

  std::size_t written = 0u;
  for (std::size_t k = 0u; k < count && written < maxEvents; ++k) {
    const std::optional<StepEvent> d = processSample(samples[k]);
    if (d) {
      events[written++] = d.value();
    }
  }

  return written;
}

void OscillationTracker::reset() noexcept {
  _baseline = 0.0f;
  _xdRing.fill(0.0f);
  _sampleIndex = 0u;
  _haveBaselineSeed = false;
  _lastStepTsUs = 0u;
  _haveLastStep = false;
  _prevPeakValue = 0.0f;
  _acceptedAny = false;
  _stepIndex = 0u;
  _ts_im1 = 0u;
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
