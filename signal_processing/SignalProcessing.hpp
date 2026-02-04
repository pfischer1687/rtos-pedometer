/** \file signal_processing/SignalProcessing.hpp
 *  Filtering and feature extraction on IMU data. Algorithms only; no hardware access.
 *  Ownership: consumes raw samples, produces processed vectors or magnitudes.
 */

#ifndef SIGNAL_PROCESSING_SIGNALPROCESSING_HPP
#define SIGNAL_PROCESSING_SIGNALPROCESSING_HPP

#include "imu/Mpu6050Driver.hpp"
#include <cstddef>
#include <array>

namespace signal_processing {

/** Processed sample: filtered accel magnitude (or axis) and optional features. */
struct ProcessedSample {
    float magnitude;       // e.g. L2 norm of accel after high-pass
    float accelX;
    float accelY;
    float accelZ;
    uint32_t timestampUs;
};

/** Configuration for filter pipeline (fixed at init). */
struct FilterConfig {
    float highPassCutoffHz;
    float lowPassCutoffHz;
    uint32_t sampleRateHz;
};

/** Stateless filter pipeline. Process one or a batch of raw samples. */
class SignalProcessor {
public:
    SignalProcessor() = default;

    /** Set filter coefficients from config. */
    void setConfig(const FilterConfig& config);

    /** Process one raw sample into \p out. */
    void process(const imu::ImuSample& in, ProcessedSample& out);

    /** Process \p count samples from \p in into \p out. */
    void processBatch(const imu::ImuSample* in, ProcessedSample* out, size_t count);

    /** Reset filter state (e.g. after idle). */
    void reset();
};

} // namespace signal_processing

#endif /* SIGNAL_PROCESSING_SIGNALPROCESSING_HPP */
