/**
 * @file signal_processing/SignalProcessing.cpp
 * @brief Implementation of the SignalProcessor class.
 */

#include "signal_processing/SignalProcessing.hpp"

namespace signal_processing {

void SignalProcessor::setConfig(const FilterConfig &config) noexcept {}

void SignalProcessor::process(const imu::ImuSample &in,
                              ProcessedSample &out) noexcept {}

void SignalProcessor::processBatch(const imu::ImuSample *in,
                                   ProcessedSample *out,
                                   size_t count) noexcept {}

void SignalProcessor::reset() noexcept {}

} // namespace signal_processing
