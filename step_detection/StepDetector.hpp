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
constexpr uint32_t MIN_STEP_INTERVAL_MS = 200u;
constexpr uint32_t WARMUP_SAMPLES = 30u;
} // namespace defaults

/**
 * @struct StepTuning
 * @brief Baseline tracking gains (EMA smoothing, damping).
 */
struct StepTuning {
  float baselineDamping{0.5f};
};

/**
 * @struct StepHeuristics
 * @brief Plateau tolerance for peak detection.
 */
struct StepHeuristics {
  float plateauEps{1.0e-5f};
};

/**
 * @struct StepDetectorConfig
 * @brief Tunable timing and baseline parameters (magnitude in g).
 */
struct StepDetectorConfig {
  float baselineEmaAlpha{0.08f};
  /** Ignored: timing uses fixed `MIN_STEP_INTERVAL_US` in the detector. */
  uint32_t minStepIntervalMs{defaults::MIN_STEP_INTERVAL_MS};
  uint32_t warmupSamples{defaults::WARMUP_SAMPLES};
  StepTuning tuning{};
  StepHeuristics heuristics{};
};

/**
 * @struct StepEvent
 * @brief One accepted step (timestamp from acquisition when the step was accepted).
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
enum class StepRejectReason : uint8_t { None, NoPeak };

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
 * @brief Minimal decision-funnel counters.
 */
struct StepDetectorDebugStats {
  uint32_t peaks{0};
  uint32_t emitted{0};
  uint32_t rejectNoPeak{0};
  uint32_t intervalBlocked{0};
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
   * @param timestampUs Sample time (microseconds); sole time reference for gating.
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
   * @brief Read decision-funnel debug counters.
   */
  [[nodiscard]] const StepDetectorDebugStats &getDebugStats() const noexcept;

  /**
   * @brief Clear decision-funnel debug counters.
   */
  void resetDebugStats() noexcept;

private:
  static constexpr uint32_t NUM_WARMUP_SAMPLES = 30u;
  static constexpr float HP_ALPHA = 0.02f;
  static constexpr float LP_ALPHA = 0.15f;

  [[nodiscard]] float filterMagnitude(float magnitudeG) noexcept;
  void shiftFilteredSample(float filtered, platform::TickUs timestampUs) noexcept;

  StepDetectorConfig _config{};

  float _hpState{0.0f};
  float _lpState{0.0f};
  float _prevInput{0.0f};
  float _prevFiltered{0.0f};
  float _prevPrevFiltered{0.0f};
  platform::TickUs _prevTimestampUs{0u};
  uint8_t _filteredFill{0u};

  uint32_t _warmupSamples{NUM_WARMUP_SAMPLES};

  uint32_t _stepIndex{0u};

  StepDetectorDebugStats _dbg{};
};

} // namespace step_detection

#endif /* STEP_DETECTION_STEPDETECTOR_HPP */
