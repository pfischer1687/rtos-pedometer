/**
 * @file step_detection/StepDetector.hpp
 * @brief Step detection algorithm on processed magnitude/features.
 */

#ifndef STEP_DETECTION_STEPDETECTOR_HPP
#define STEP_DETECTION_STEPDETECTOR_HPP

#include "signal_processing/SignalProcessing.hpp"
#include <cstddef>
#include <cstdint>

namespace step_detection {

/**
 * @struct StepDetectorConfig
 * @brief Parameters for threshold and timing (fixed or tuned).
 */
struct StepDetectorConfig {
  float magnitudeThreshold{0.0f};
  uint32_t minStepIntervalMs{0};
  uint32_t maxStepIntervalMs{0};
};

/**
 * @struct StepEvent
 * @brief Callback or event: one step detected at timestamp.
 */
struct StepEvent {
  uint32_t timestampUs{0};
  uint32_t stepIndex{0}; // running count in current session
};

/**
 * @class StepDetector
 * @brief Step detector: stateful, processes stream of ProcessedSample.
 */
class StepDetector {
public:
  /**
   * @brief Constructor.
   */
  StepDetector() noexcept = default;

  /**
   * @brief Set configuration.
   * @param config Step detector configuration.
   */
  void setConfig(const StepDetectorConfig &config) noexcept;

  /**
   * @brief Process one sample; may produce a step (return true and fill \p
   * event).
   * @param sample Processed sample.
   * @param event Step event.
   * @return True if a step was detected.
   */
  bool process(const signal_processing::ProcessedSample &sample,
               StepEvent &event) noexcept;

  /**
   * @brief Process a batch; returns number of steps detected (events written to
   * \p events).
   * @param samples Processed samples.
   * @param count Number of samples to process.
   * @param events Step events.
   * @param maxEvents Maximum number of events to write.
   * @return Number of steps detected.
   */
  size_t processBatch(const signal_processing::ProcessedSample *samples,
                      size_t count, StepEvent *events,
                      size_t maxEvents) noexcept;

  /**
   * @brief Reset state (e.g. new session).
   */
  void reset() noexcept;

private:
  StepDetectorConfig _config{};
  uint32_t _lastStepTimeUs{0};
  uint32_t _stepCounter{0};
};

} // namespace step_detection

#endif /* STEP_DETECTION_STEPDETECTOR_HPP */
