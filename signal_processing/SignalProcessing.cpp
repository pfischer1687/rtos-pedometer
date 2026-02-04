/** \file signal_processing/SignalProcessing.cpp
 *  Filter and feature extraction implementation.
 */

#include "signal_processing/SignalProcessing.hpp"

namespace signal_processing {

void SignalProcessor::setConfig(const FilterConfig& config)
{
}

void SignalProcessor::process(const imu::ImuSample& in, ProcessedSample& out)
{
}

void SignalProcessor::processBatch(const imu::ImuSample* in, ProcessedSample* out, size_t count)
{
}

void SignalProcessor::reset()
{
}

} // namespace signal_processing
