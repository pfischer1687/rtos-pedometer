/**
 * @file step_detection/StepDetector.hpp
 * @brief Step detection from processed acceleration magnitudes.
 */

#ifndef STEP_DETECTION_STEPDETECTOR_HPP
#define STEP_DETECTION_STEPDETECTOR_HPP

#include "platform/Platform.hpp"
#include "signal_processing/SignalProcessing.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

namespace step_detection {

namespace defaults {
constexpr float MIN_VALLEY_PROMINENCE_G = 0.10f;
constexpr float MIN_PEAK_PROMINENCE_G = 0.09f;
constexpr uint32_t MIN_STEP_INTERVAL_MS = 200u;
constexpr uint32_t WARMUP_SAMPLES = 30u;
} // namespace defaults

/**
 * @struct StepTuning
 * @brief Core gains (exponential moving average (EMA) smoothing, damping, peak
 * amplitude tracking).
 */
struct StepTuning {
  float peakAmpAdaptGain{0.4f};
  float baselineDamping{0.5f};
  float peakEmaAlpha{0.1f};
  float adaptiveBlendT{0.5f};
};

/**
 * @struct StepHeuristics
 * @brief Tunable constants for plateau detection, confidence scoring, and
 * intervals.
 */
struct StepHeuristics {
  float plateauEps{1.0e-5f};
  float confidenceSlopeScale{2.0f};
  float confidenceValleyScale{1.5f};
  float confidenceWeightSlope{0.25f};
  float confidenceWeightValley{0.25f};
  float confidenceWeightBaseline{0.25f};
  float confidenceWeightInterval{0.25f};
  platform::TickUs intervalConfidenceMinPeriodUs{500'000u}; // 0.5s
  float intervalScoreEpsilonUs{1.0f};
};

/**
 * @struct StepDetectorConfig
 * @brief Tunable peak / timing parameters (magnitude in g).
 * @details
 * - `minValleyProminenceG` is the primary impulse rejector for walking: it
 *   requires a clear drop on both sides of the peak in the 3-sample window.
 */
struct StepDetectorConfig {
  float baselineEmaAlpha{0.05f};
  float slopeMin{0.04f};
  float minValleyProminenceG{defaults::MIN_VALLEY_PROMINENCE_G};
  float minPeakProminenceG{defaults::MIN_PEAK_PROMINENCE_G};
  uint32_t minStepIntervalMs{defaults::MIN_STEP_INTERVAL_MS};
  uint32_t warmupSamples{defaults::WARMUP_SAMPLES};
  StepTuning tuning{};
  StepHeuristics heuristics{};
};

/**
 * @struct StepEvent
 * @brief One accepted step at the peak sample time.
 */
struct StepEvent {
  platform::TickUs timestampUs{0};
  uint32_t stepIndex{0};
  float confidence{0.0f};
};

/**
 * @enum StepRejectReason
 * @brief Reason for rejecting a step.
 */
enum class StepRejectReason : uint8_t {
  None,
  NoPeak,
  Slope,
  Valley,
  Baseline,
  Interval
};

/**
 * @struct StepDecision
 * @brief Decision for a step.
 */
struct StepDecision {
  std::optional<StepEvent> event{};
  StepRejectReason reason{StepRejectReason::None};
};

/**
 * @struct StepDetectorDebugStats
 * @brief Decision-funnel counters (peak candidates vs emitted vs gate rejects).
 */
struct StepDetectorDebugStats {
  uint32_t peaks{0};
  uint32_t emitted{0};

  uint32_t rejectNoPeak{0};
  uint32_t rejectSlope{0};
  uint32_t rejectValley{0};
  uint32_t rejectBaseline{0};
  uint32_t rejectInterval{0};

  StepRejectReason lastRejectReason{StepRejectReason::None};
};

/**
 * @class StepDetector
 * @brief Streaming step detector.
 */
class StepDetector {
public:
  StepDetector() noexcept = default;

  /**
   * @brief Set the detector configuration.
   * @param config Configuration.
   */
  void setConfig(const StepDetectorConfig &config) noexcept;

  /**
   * @brief Process one magnitude sample.
   * @param magnitudeG Smoothed magnitude from signal processing (g).
   * @param timestampUs Sample time (microseconds).
   * @return Per-sample decision including optional step event and reject
   * reason.
   */
  [[nodiscard]] StepDecision
  processSample(float magnitudeG, platform::TickUs timestampUs) noexcept;

  /**
   * @brief Process one magnitude sample from a processed sample.
   * @param sample Processed sample.
   * @return Per-sample decision including optional step event and reject
   * reason.
   */
  [[nodiscard]] StepDecision
  processSample(const signal_processing::ProcessedSample &sample) noexcept {
    return processSample(sample.magnitude, sample.timestampUs);
  }

  /**
   * @brief Process a batch of processed samples.
   * @param samples Processed samples.
   * @param count Number of samples to process.
   * @param events Step events.
   * @param maxEvents Maximum number of step events to write.
   * @return Number of steps written.
   */
  size_t processBatch(const signal_processing::ProcessedSample *samples,
                      size_t count, StepEvent *events,
                      size_t maxEvents) noexcept;

  /**
   * @brief Reset the detector to its initial state.
   */
  void reset() noexcept;

  /**
   * @brief Read decision-funnel debug counters (for tuning telemetry).
   */
  [[nodiscard]] const StepDetectorDebugStats &getDebugStats() const noexcept;

  /**
   * @brief Clear decision-funnel debug counters.
   */
  void resetDebugStats() noexcept;

private:
  static constexpr uint32_t NUM_WARMUP_SAMPLES = 30u;
  static constexpr uint8_t RING_SIZE = 3u;

  /**
   * @brief Advance the ring buffer.
   * @param magnitudeG Magnitude.
   * @param timestampUs Timestamp.
   */
  void advanceRing(float magnitudeG, platform::TickUs timestampUs) noexcept;

  /**
   * @brief Read the window from the ring buffer.
   * @param m0 First magnitude.
   * @param m1 Second magnitude.
   * @param m2 Third magnitude.
   * @param tPeak Peak timestamp.
   * @return True if the window is valid; otherwise false.
   */
  [[nodiscard]] bool readWindow(float &m0, float &m1, float &m2,
                                platform::TickUs &tPeak) const noexcept;

  /**
   * @brief Compute the adaptive threshold.
   * @return Adaptive threshold.
   * @details The adaptive threshold is a blend of the minimum peak prominence
   * and the peak amplitude EMA, weighted by the adaptive blend tuning
   * parameter.
   */
  [[nodiscard]] float adaptiveThreshold() const noexcept;

  /**
   * @brief Check if the interval gate is satisfied.
   * @param peakTimeUs Peak timestamp.
   * @return True if the interval gate is satisfied; otherwise false.
   */
  [[nodiscard]] bool intervalGate(platform::TickUs peakTimeUs) const noexcept;

  /**
   * @brief Update the baseline estimate.
   * @param newestMag Newest magnitude.
   */
  void updateBaselineEstimate(float newestMag) noexcept;

  /**
   * @brief Update the peak amplitude EMA.
   * @param peakValleyProminence Peak valley prominence.
   */
  void updatePeakAmplitudeEma(float peakValleyProminence) noexcept;

  /**
   * @brief Compute the confidence.
   * @param rise Rise.
   * @param fall Fall.
   * @param valleyProminence Valley prominence.
   * @param baselineProminence Baseline prominence.
   * @param threshold Threshold.
   * @param intervalDtUs Interval duration.
   * @param havePriorStepInterval True if there is a prior step interval;
   * otherwise false.
   * @return Confidence score in [0,1].
   */
  [[nodiscard]] float
  computeConfidence(float rise, float fall, float valleyProminence,
                    float baselineProminence, float threshold,
                    platform::TickUs intervalDtUs,
                    bool havePriorStepInterval) const noexcept;

  StepDetectorConfig _config{};

  float _m[RING_SIZE]{};
  platform::TickUs _t[RING_SIZE]{};
  uint8_t _idx{0};
  uint8_t _fill{0};

  float _baselineEstimate{0.0f};
  bool _haveBaselineEstimate{false};

  float _peakAmpEma{0.0f};
  bool _havePeakAmp{false};

  uint32_t _warmupSamples{NUM_WARMUP_SAMPLES};

  bool _haveLastStep{false};
  platform::TickUs _lastStepTimeUs{0u};
  uint32_t _stepIndex{0u};
  uint32_t _minStepIntervalUs{250'000u}; // 0.25s

  StepDetectorDebugStats _dbg{};
};

} // namespace step_detection

#endif /* STEP_DETECTION_STEPDETECTOR_HPP */
