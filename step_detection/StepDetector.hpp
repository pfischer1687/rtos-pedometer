/**
 * @file step_detection/StepDetector.hpp
 * @brief Step detection from processed acceleration magnitudes (walking MVP).
 */

#ifndef STEP_DETECTION_STEPDETECTOR_HPP
#define STEP_DETECTION_STEPDETECTOR_HPP

#include "platform/Platform.hpp"
#include "signal_processing/SignalProcessing.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

namespace step_detection {

/**
 * @struct StepTuning
 * @brief Grouped internal gains (defaults match prior hard-coded behavior).
 */
struct StepTuning {
  float peakAmpAdaptGain{0.4f};
  float baselineDamping{0.5f};
  float peakEmaAlpha{0.1f};
};

/**
 * @struct StepDetectorConfig
 * @brief Tunable peak / timing parameters (magnitude in g).
 * @details
 * - `minValleyProminenceG` is the primary impulse rejector for walking: it
 *   requires a clear drop on both sides of the peak in the 3-sample window.
 */
struct StepDetectorConfig {
  /// EMA smoothing for baseline estimate (0–1); peaks are damped upward.
  float baselineEmaAlpha{0.05f};
  /// Minimum rise and fall on a candidate peak (g): `m1 - m0` and `m1 - m2`.
  float slopeMin{0.04f};
  /// Minimum valley-to-peak prominence: `m1 - min(m0, m2)` (g).
  float minValleyProminenceG{0.11f};
  /// Minimum prominence vs slow baseline estimate (g); combined with adaptive term.
  float minPeakProminenceG{0.10f};
  /// Ignore peaks closer than this (ms).
  uint32_t minStepIntervalMs{250u};
  /// Samples after reset before steps may be emitted (baseline-only period).
  uint32_t warmupSamples{30u};
  StepTuning tuning{};
};

/**
 * @struct StepEvent
 * @brief One accepted step at the peak sample time.
 */
struct StepEvent {
  platform::TickUs timestampUs{0};
  uint32_t stepIndex{0};
  /// Heuristic quality in [0,1] for logging / tuning (not a calibrated probability).
  float confidence{0.0f};
};

/**
 * @class StepDetector
 * @brief Streaming step detector: 3-sample peak, slopes, prominence, baseline.
 */
class StepDetector {
public:
  StepDetector() noexcept = default;

  void setConfig(const StepDetectorConfig &config) noexcept;

  /**
   * @brief Process one magnitude sample.
   * @param magnitudeG Smoothed magnitude from signal processing (g).
   * @param timestampUs Sample time (microseconds).
   * @return Step event if a peak passes gating; otherwise nullopt.
   */
  [[nodiscard]] std::optional<StepEvent>
  processSample(float magnitudeG, platform::TickUs timestampUs) noexcept;

  /**
   * @brief Convenience: uses ProcessedSample::magnitude and ::timestampUs.
   */
  [[nodiscard]] std::optional<StepEvent>
  processSample(const signal_processing::ProcessedSample &sample) noexcept {
    return processSample(sample.magnitude, sample.timestampUs);
  }

  /**
   * @brief Process a batch; writes up to maxEvents step events.
   * @return Number of steps written.
   */
  size_t processBatch(const signal_processing::ProcessedSample *samples,
                      size_t count, StepEvent *events,
                      size_t maxEvents) noexcept;

  void reset() noexcept;

private:
  static constexpr uint32_t kWarmupSamplesDefault = 30u;

  [[nodiscard]] bool detectPeakShape() const noexcept;
  [[nodiscard]] float adaptiveThreshold() const noexcept;
  [[nodiscard]] bool intervalGate(platform::TickUs peakTimeUs) const noexcept;
  void updateBaselineEstimate(float newestMag) noexcept;
  void updatePeakAmplitudeEma(float peakValleyProminence) noexcept;

  [[nodiscard]] float computeConfidence(float rise, float fall, float valleyProminence,
                                        float baselineProminence, float threshold,
                                        platform::TickUs intervalDtUs,
                                        bool firstSinceLastStep) const noexcept;

  StepDetectorConfig _config{};
  float _m0{0.0f};
  float _m1{0.0f};
  float _m2{0.0f};
  platform::TickUs _t0{0};
  platform::TickUs _t1{0};
  platform::TickUs _t2{0};
  uint8_t _historyCount{0};

  float _baselineEstimate{0.0f};
  bool _haveBaselineEstimate{false};

  float _peakAmpEma{0.0f};
  bool _havePeakAmp{false};

  uint32_t _warmupSamples{kWarmupSamplesDefault};

  platform::TickUs _lastStepTimeUs{0};
  uint32_t _stepIndex{0};
  uint32_t _minStepIntervalUs{250000u};
};

} // namespace step_detection

#endif /* STEP_DETECTION_STEPDETECTOR_HPP */
