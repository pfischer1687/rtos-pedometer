/**
 * @file step_detection/OscillationTracker.hpp
 * @brief Step detection module tracking oscillation via filtered acceleration
 * magnitude peaks adjusted via an exponential moving average (EMA) baseline
 * with a refractory period as well as quiet and shoulder period condition
 * gates.
 */

#ifndef STEP_DETECTION_OSCILLATIONTRACKER_HPP
#define STEP_DETECTION_OSCILLATIONTRACKER_HPP

#include "platform/Platform.hpp"
#include "signal_processing/SignalProcessing.hpp"
#include <array>
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
 * @struct StepDetectionDebugStats
 * @brief Step detection debug statistics.
 */
struct StepDetectionDebugStats {
  std::uint32_t peaks{0u};
  std::uint32_t emitted{0u};
  std::uint32_t rejected{0u};
};

/**
 * @struct OscillationTrackerConfig
 * @brief Runtime tuning for EMA detrended peak step detection.
 */
struct OscillationTrackerConfig {
  float ema_alpha{0.02f};
  float mag_scale{4.35f};
  float peak_threshold{0.22f};
  float min_step_dt{0.18f};
  std::size_t quiet_pre_n{8u};
  float quiet_abs{0.044f};
  float idle_threshold_s{2.5f};
  float peak_consistency_ratio{0.6f};
};

/**
 * @class OscillationTracker
 * @brief Detrended local-maximum step detector (single-threaded per instance).
 */
class OscillationTracker {
public:
  /**
   * @brief Constructor for the OscillationTracker class.
   * @param config The configuration for the OscillationTracker.
   */
  explicit OscillationTracker(OscillationTrackerConfig config) noexcept;

  /**
   * @brief Process a single sample; on accept returns the step at peak sample
   * (i-1), not the current sample.
   * @param sample The sample to process.
   * @return Step event if a step was accepted, otherwise nullopt.
   */
  [[nodiscard]] std::optional<StepEvent>
  processSample(const signal_processing::ProcessedSample &sample) noexcept;

  /**
   * @brief Process a batch of samples and return the number of events detected.
   * @param samples The samples to process.
   * @param count The number of samples to process.
   * @param events The events to store the results in.
   * @param maxEvents The maximum number of events to store.
   * @return The number of events detected.
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
  static constexpr std::size_t MAX_QUIET_RING = 128u;

  /**
   * @brief Enforces a minimum time gap between accepted steps.
   * @param dt_s The time since the last step.
   * @return True if the peak passes the refractory period, false otherwise.
   */
  [[nodiscard]] bool passesRefractory(float dt_s) const noexcept;

  /**
   * @brief Requires recent motion before accepting a peak after an idle period.
   * @param sampleIndex The index of the sample.
   * @param dt_s The time since the last step.
   * @return True if the peak passes the quiet gate, false otherwise.
   * @details
   * - The peak detection part is `NUM_SAMPLES_FOR_PEAK` samples long.
   * - The peak is the middle sample in the peak detection part.
   * - The window for determining whether the motion preceding the peak was
   *   quiet or actively walking is `quiet_pre_n` samples long.
   */
  [[nodiscard]] bool passesQuietGate(std::size_t sampleIndex,
                                     float dt_s) const noexcept;

  /**
   * @brief Ensures peak amplitude consistency after idle periods.
   * @param xd_im1 The detrended signal value at the previous sample (i-1).
   * @param dt_s The time since the last step.
   * @return True if the peak passes the shoulder filter, false otherwise.
   */
  [[nodiscard]] bool passesShoulderFilter(float xd_im1,
                                          float dt_s) const noexcept;

  /**
   * @brief Check if the peak is accepted as a detected step.
   * @param sampleIndex The index of the sample.
   * @param dt_s The time since the last step.
   * @param xd_im1 The detrended signal value at the previous sample (i-1).
   * @return True if the peak is accepted, false otherwise.
   */
  [[nodiscard]] bool acceptPeak(std::size_t sampleIndex, float dt_s,
                                float xd_im1) const noexcept;

  OscillationTrackerConfig _cfg;
  float _baseline{0.0f};
  std::array<float, MAX_QUIET_RING> _xdRing{};
  std::size_t _ringCap{0u};
  std::size_t _sampleIndex{0u};
  bool _haveBaselineSeed{false};
  platform::TickUs _lastStepTsUs{0u};
  bool _haveLastStep{false};
  float _prevPeakValue{0.0f};
  bool _acceptedAny{false};
  std::size_t _stepIndex{0u};
  platform::TickUs _ts_im1{0u};
  StepDetectionDebugStats _dbg{};
};

} // namespace step_detection

#endif /* STEP_DETECTION_OSCILLATIONTRACKER_HPP */
