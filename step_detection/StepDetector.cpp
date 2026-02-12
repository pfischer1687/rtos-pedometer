/**
 * @file step_detection/StepDetector.cpp
 * @brief Step detection implementation.
 *  Step detection implementation.
 */

#include "step_detection/StepDetector.hpp"

namespace step_detection {

void StepDetector::setConfig(const StepDetectorConfig &config) noexcept {}

bool StepDetector::process(const signal_processing::ProcessedSample &sample,
                           StepEvent &event) noexcept {
  return false;
}

size_t
StepDetector::processBatch(const signal_processing::ProcessedSample *samples,
                           size_t count, StepEvent *events,
                           size_t maxEvents) noexcept {
  return 0;
}

void StepDetector::reset() noexcept {}

} // namespace step_detection
