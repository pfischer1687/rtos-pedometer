/**
 * @file step_detection/OscillationTracker.hpp
 * @brief Magnitude-domain oscillation tracking: slope zero-crossing peak
 * detection with interval gating.
 */

#ifndef STEP_DETECTION_OSCILLATIONTRACKER_HPP
#define STEP_DETECTION_OSCILLATIONTRACKER_HPP

#include "platform/Platform.hpp"
#include "signal_processing/SignalProcessing.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

namespace step_detection {

/**
 * @struct StepEvent
 * @brief Step event with timestamp, and step index.
 */
struct StepEvent {
  platform::TickUs timestampUs{0};
  std::uint32_t stepIndex{0};
};

/**
 * @struct StepDecision
 * @brief Step decision with event.
 */
struct StepDecision {
  std::optional<StepEvent> event{};
};

/**
 * @struct StepDetectionDebugStats
 * @brief Step detection debug statistics.
 */
struct StepDetectionDebugStats {
  std::uint32_t peaks{0u};
  std::uint32_t emitted{0u};
  std::uint32_t rejected{0u};
};

/**
 * @enum OscLeg
 * @brief Oscillation leg.
 * @details
 * - SeekRise: Waiting for a rising slope above hysteresis.
 * - Rising: Ascending leg; arm + -> - crossing as peak.
 * - Falling: Descending leg; arm - -> + crossing before next peak.
 */
enum class OscLeg : std::uint8_t {
  SeekRise,
  Rising,
  Falling,
};

/**
 * @class OscillationTracker
 * @brief Oscillation tracker for step detection.
 */
class OscillationTracker {
public:
  OscillationTracker() noexcept = default;

  /**
   * @brief Process a single sample and return a step decision.
   * @param sample The processed sample.
   * @return A step decision.
   */
  [[nodiscard]] StepDecision
  processSample(const signal_processing::ProcessedSample &sample) noexcept;

  /**
   * @brief Process a batch of samples and return the number of events written.
   * @param samples The batch of processed samples.
   * @param count The number of samples to process.
   * @param events The events to write.
   * @param maxEvents The maximum number of events to write.
   * @return The number of events written.
   */
  [[nodiscard]] std::size_t
  processBatch(const signal_processing::ProcessedSample *samples,
               std::size_t count, StepEvent *events,
               std::size_t maxEvents) noexcept;

  /**
   * @brief Reset the tracker to its initial state.
   */
  void reset() noexcept;

  /**
   * @brief Get the debug statistics.
   * @return The debug statistics.
   */
  [[nodiscard]] const StepDetectionDebugStats &getDebugStats() const noexcept {
    return _dbg;
  }

  /**
   * @brief Reset the debug statistics.
   */
  void resetDebugStats() noexcept;

  /**
   * @brief Format oscillation detector debug counters for USB status output.
   * @param buf Destination buffer.
   * @param size Destination buffer size.
   * @return Number of bytes written, or 0 on failure/truncation.
   */
  [[nodiscard]] std::size_t
  formatOscillationDebugReport(char *buf, std::size_t size) const noexcept;

private:
  static constexpr platform::TickUs MIN_STEP_INTERVAL = 300'000u;

  // Magnitude delta per sample (g) required to treat slope as non‑noise.
  static constexpr float SLOPE_HYSTERESIS_G = 0.0045f;

  float _prevMag{0.0f};
  float _prevSlope{0.0f};
  bool _havePrevMag{false};
  bool _havePrevSlope{false};
  OscLeg _leg{OscLeg::SeekRise};
  platform::TickUs _lastStepEmitUs{0u};
  bool _haveLastEmit{false};
  std::uint32_t _stepIndex{0u};
  StepDetectionDebugStats _dbg{};
};

} // namespace step_detection

#endif /* STEP_DETECTION_OSCILLATIONTRACKER_HPP */
