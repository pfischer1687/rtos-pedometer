/**
 * @file signal_processing/SignalProcessing.hpp
 * @brief Filtering and feature extraction on IMU data.
 * @details
 * - Not safe for sharing across RTOS threads, SPSC ownership required (no
 * locking).
 */

#ifndef SIGNAL_PROCESSING_SIGNALPROCESSING_HPP
#define SIGNAL_PROCESSING_SIGNALPROCESSING_HPP

#include "imu/Mpu6050Driver.hpp"
#include "platform/Platform.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace signal_processing {

namespace defaults {

/**
 * @brief Default acceleration scale.
 * @details
 * - IMU defaults to +/- 2g acceleration scale, which has conversion factor
 * 16384 LSB/g.
 */
constexpr float DEFAULT_ACC_SCALE_G_PER_LSB = 6.1035156e-5f;

/**
 * @brief Default high pass cutoff frequency.
 * @details
 * - Walking cadence is generally in the range of 0.5 to 3.0 Hz.
 */
constexpr float DEFAULT_HIGH_PASS_CUTOFF_HZ = 0.5f;

/**
 * @brief Default sample rate.
 * @details
 * - IMU default output rate is 100 Hz.
 */
constexpr uint16_t DEFAULT_SAMPLE_RATE_HZ = 100;

/**
 * @brief Default moving average window.
 * @details
 * - Moving average is used to smooth the magnitude of the acceleration.
 */
constexpr uint32_t DEFAULT_MOVING_AVERAGE_WINDOW = 7;

} // namespace defaults

/**
 * @struct ProcessedSample
 * @brief Filtered acceleration axes and smoothed magnitude.
 * @details
 * - Magnitude is a moving average of the L2 norm of the filtered axes.
 */
struct ProcessedSample {
  float magnitude{0.0f};
  float accelX{0.0f};
  float accelY{0.0f};
  float accelZ{0.0f};
  platform::TickUs timestampUs{0};
};

/**
 * @brief State of a single axis filter.
 * @details
 * - High-pass filter output.
 * - High-pass filter previous input.
 */
struct AxisState {
  float hpOut{0.0f};
  float hpPrevIn{0.0f};
};

/**
 * @struct FilterConfig
 * @brief Sample rate and cutoff frequencies for high and low pass filters (set
 * in setConfig()).
 * @details
 * - High-pass filter removes slow drift / gravity per axis via the following
 *   first-order infinite impulse response difference equation:
 *   - `y[n] = \alpha * (y[n-1] + x[n] - x[n-1])`
 *   - `\alpha = RC / (RC + dt)`
 *   - `RC = 1 / (2 * \pi * highPassCutoffHz)`
 *   - `dt = 1 / sampleRateHz`
 * - Low-pass filter is set as DLPF in IMU hardware configuration.
 * - Set a cutoff ≤ 0 to bypass the corresponding filter.
 * - Filters are applied sequentially (high-pass followed by low-pass).
 * - Moving average only applied to the magnitude of the acceleration.
 */
struct FilterConfig {
  float highPassCutoffHz{defaults::DEFAULT_HIGH_PASS_CUTOFF_HZ};
  uint16_t sampleRateHz{defaults::DEFAULT_SAMPLE_RATE_HZ};
  uint32_t movingAverageWindow{defaults::DEFAULT_MOVING_AVERAGE_WINDOW};
  float accelScale{defaults::DEFAULT_ACC_SCALE_G_PER_LSB};
};

/**
 * @brief Get the acceleration scale for a given range.
 * @param range The range to get the acceleration scale for.
 * @return The acceleration scale.
 */
[[nodiscard]] inline float getAccelScale(imu::AccelRange range) noexcept {
  switch (range) {
  case imu::AccelRange::PlusMinus2G:
    return 1.0f / 16384.0f;
  case imu::AccelRange::PlusMinus4G:
    return 1.0f / 8192.0f;
  case imu::AccelRange::PlusMinus8G:
    return 1.0f / 4096.0f;
  case imu::AccelRange::PlusMinus16G:
    return 1.0f / 2048.0f;
  default:
    return 1.0f / 16384.0f;
  }
}

/**
 * @brief L2 norm of the components of acceleration: `\sqrt{x^2+y^2+z^2}`.
 */
[[nodiscard]] inline float accelerationMagnitude(float x, float y,
                                                 float z) noexcept {
  const float s2 = x * x + y * y + z * z;
  return std::sqrtf(s2);
}

/**
 * @class SignalProcessor
 * @brief Filtering and smoothing of IMU acceleration data.
 * @details
 * - Safe only for single-thread ownership.
 * - Call setConfig() before processing.
 */
class SignalProcessor final {
public:
  SignalProcessor() noexcept = default;

  ~SignalProcessor() noexcept = default;
  SignalProcessor(const SignalProcessor &) = delete;
  SignalProcessor &operator=(const SignalProcessor &) = delete;
  SignalProcessor(SignalProcessor &&) = delete;
  SignalProcessor &operator=(SignalProcessor &&) = delete;

  /**
   * @brief Set the configuration for the signal processor.
   * @param config The configuration to set.
   * @details
   * - Invalid values are clamped to safe ranges.
   */
  void setConfig(const FilterConfig &config) noexcept;

  /**
   * @brief Process a single IMU sample.
   * @param in The IMU sample to process.
   * @param out The processed sample.
   * @details
   * - First-order infinite impulse response difference equation for high-pass
   * filter:
   *   - `y[n] = \alpha * (y[n-1] + x[n] - x[n-1])`
   *   - `\alpha = RC / (RC + dt)`
   *   - `RC = 1 / (2 * \pi * highPassCutoffHz)`
   *   - `dt = 1 / sampleRateHz`
   */
  void processOne(const imu::ImuSample &in, ProcessedSample &out) noexcept;

  /**
   * @brief Process a batch of IMU samples.
   * @param in The batch of IMU samples to process.
   * @param out The batch of processed samples.
   * @param count The number of samples to process.
   */
  void processBatch(const imu::ImuSample *in, ProcessedSample *out,
                    size_t count) noexcept;

  /**
   * @brief Reset the filters to their initial state.
   */
  void reset() noexcept;

private:
  static constexpr size_t MAX_MOVING_AVERAGE_WINDOW = 64u;

  /**
   * @brief Apply the configuration to the signal processor.
   * @details
   * - Computes derived parameters from the configuration.
   * - Must call after any change to _config.
   */
  void applyConfig() noexcept;

  /**
   * @brief Reset the filters to their initial state.
   */
  void resetFilters() noexcept;

  /**
   * @brief Process the acceleration component for a given axis.
   * @param in The IMU sample.
   * @param axis The axis to process.
   * @return The processed acceleration component.
   */
  float processAxis(const imu::ImuSample &in, size_t axis) noexcept;

  /**
   * @brief Update the magnitude using a circular buffer-based moving average.
   * @param mag The magnitude from which to update the moving average.
   * @return The updated magnitude.
   */
  float updateMagnitudeMovingAverage(float mag) noexcept;

  FilterConfig _config{};

  // Derived parameters computed from _config in applyConfig().
  // Must call applyConfig() after any change to _config.
  float _accelScale{0.0f};
  // Circular buffer. Only first _maWindow entries are used.
  uint32_t _maWindow{defaults::DEFAULT_MOVING_AVERAGE_WINDOW};

  float _alphaHp{0.0f};
  bool _hpBypass{true};
  AxisState _axis[imu::NUM_ACC_AXES]{};
  float _maBuf[MAX_MOVING_AVERAGE_WINDOW]{};
  float _maSum{0.0f};
  uint32_t _maIndex{0u};
  uint32_t _maCount{0u};
};

} // namespace signal_processing

#endif /* SIGNAL_PROCESSING_SIGNALPROCESSING_HPP */
