/**
 * @file signal_processing/SignalProcessing.hpp
 * @brief Filtering and feature extraction on IMU data.
 */

#ifndef SIGNAL_PROCESSING_SIGNALPROCESSING_HPP
#define SIGNAL_PROCESSING_SIGNALPROCESSING_HPP

#include "imu/Mpu6050Driver.hpp"
#include <cstddef>

namespace signal_processing {

/**
 * @struct ProcessedSample
 * @brief Processed sample: filtered accel magnitude (or axis) and optional
 * features.
 */
struct ProcessedSample {
  float magnitude{0.0f}; // e.g. L2 norm of accel after high-pass
  float accelX{0.0f};
  float accelY{0.0f};
  float accelZ{0.0f};
  uint32_t timestampUs{0};
};

/**
 * @struct FilterConfig
 * @brief Configuration for filter pipeline (fixed at init).
 */
struct FilterConfig {
  float highPassCutoffHz{0.0f};
  float lowPassCutoffHz{0.0f};
  uint32_t sampleRateHz{0};
};

/**
 * @class SignalProcessor
 * @brief Stateless filter pipeline. Process one or a batch of raw samples.
 */
class SignalProcessor {
public:
  /**
   * @brief Constructor.
   */
  SignalProcessor() noexcept = default;

  /**
   * @brief Set filter coefficients from config.
   * @param config Filter configuration.
   */
  void setConfig(const FilterConfig &config) noexcept;

  /**
   * @brief Process one raw sample into \p out.
   * @param in Raw sample.
   * @param out Processed sample.
   */
  void process(const imu::ImuSample &in, ProcessedSample &out) noexcept;

  /**
   * @brief Process \p count samples from \p in into \p out.
   * @param in Raw samples.
   * @param out Processed samples.
   * @param count Number of samples to process.
   */
  void processBatch(const imu::ImuSample *in, ProcessedSample *out,
                    size_t count) noexcept;

  /**
   * @brief Reset filter state (e.g. after idle).
   */
  void reset() noexcept;
};

} // namespace signal_processing

#endif /* SIGNAL_PROCESSING_SIGNALPROCESSING_HPP */
