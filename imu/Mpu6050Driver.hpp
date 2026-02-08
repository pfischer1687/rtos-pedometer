/**
 * @file: imu/Mpu6050Driver.hpp
 * @brief: MPU-6050 driver over I2C. Hardware access only; no application logic.
 * Ownership: imu module owns register map and I2C transactions.
 */

#ifndef IMU_MPU6050DRIVER_HPP
#define IMU_MPU6050DRIVER_HPP

#include "platform/I2CProvider.hpp"
#include "platform/Platform.hpp"
#include <cstddef>
#include <cstdint>

namespace imu {

/**
 * @brief: Raw sample: accel XYZ (LSB), temp (LSB), gyro XYZ (LSB), timestamp
 * (us).
 */
struct ImuSample {
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t temp;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  platform::TickUs timestampUs;
};

/** @brief: Sample rate in Hz. */
enum class SampleRate : uint16_t {
  Hz100 = 100,
  Hz200 = 200,
  Hz500 = 500,
  Hz1000 = 1000,
};

/** @brief: Full-scale ranges for accel and gyro. */
struct ImuConfig {
  uint8_t accelScaleLsbPerG; // e.g. 16384 for +/- 2g
  float gyroScaleLsbPerDps;  // e.g. 65.5 for +/- 500 dps
  SampleRate sampleRate;
};

/** @brief: MPU-6050 driver. Blocking I2C read from non-ISR context only. */
class Mpu6050Driver {
public:
  explicit Mpu6050Driver(platform::I2CProvider &i2c);

  /** @brief: Configure and wake the device. */
  platform::Result init();

  /** @brief: Set sample rate and scales. */
  platform::Result setConfig(const ImuConfig &config);

  /** @brief: Read one sample (accel + temp + gyro) into \p sample. Call from
   * acquisition thread only. */
  platform::Result readSample(ImuSample &sample);

  /** @brief: Read multiple samples in one transaction if supported. */
  platform::Result readFifo(ImuSample *samples, size_t maxCount,
                            size_t &outCount);

private:
  platform::I2CProvider &_i2c;
  static constexpr uint8_t MPU6050_I2C_ADDR = 0x68;
};

} // namespace imu

#endif /* IMU_MPU6050DRIVER_HPP */
