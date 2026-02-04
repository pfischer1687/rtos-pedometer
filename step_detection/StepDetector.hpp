/** \file step_detection/StepDetector.hpp
 *  Step detection algorithm on processed magnitude/features. No hardware access.
 *  Ownership: consumes ProcessedSample stream; emits step events.
 */

#ifndef STEP_DETECTION_STEPDETECTOR_HPP
#define STEP_DETECTION_STEPDETECTOR_HPP

#include "signal_processing/SignalProcessing.hpp"
#include <cstdint>
#include <cstddef>

namespace step_detection {

/** Parameters for threshold and timing (fixed or tuned). */
struct StepDetectorConfig {
    float magnitudeThreshold;
    uint32_t minStepIntervalMs;
    uint32_t maxStepIntervalMs;
};

/** Callback or event: one step detected at timestamp. */
struct StepEvent {
    uint32_t timestampUs;
    uint32_t stepIndex;  // running count in current session
};

/** Step detector: stateful, processes stream of ProcessedSample. */
class StepDetector {
public:
    StepDetector() = default;

    void setConfig(const StepDetectorConfig& config);

    /** Process one sample; may produce a step (return true and fill \p event). */
    bool process(const signal_processing::ProcessedSample& sample, StepEvent& event);

    /** Process a batch; returns number of steps detected (events written to \p events). */
    size_t processBatch(const signal_processing::ProcessedSample* samples, size_t count,
                        StepEvent* events, size_t maxEvents);

    /** Reset state (e.g. new session). */
    void reset();
};

} // namespace step_detection

#endif /* STEP_DETECTION_STEPDETECTOR_HPP */
