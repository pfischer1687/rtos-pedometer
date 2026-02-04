/** \file step_detection/StepDetector.cpp
 *  Step detection implementation.
 */

#include "step_detection/StepDetector.hpp"

namespace step_detection {

void StepDetector::setConfig(const StepDetectorConfig& config)
{
}

bool StepDetector::process(const signal_processing::ProcessedSample& sample, StepEvent& event)
{
    return false;
}

size_t StepDetector::processBatch(const signal_processing::ProcessedSample* samples, size_t count,
                                  StepEvent* events, size_t maxEvents)
{
    return 0;
}

void StepDetector::reset()
{
}

} // namespace step_detection
