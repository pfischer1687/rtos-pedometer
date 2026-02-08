/**
 * @file imu/Mpu6050AccelDriver.hpp
 * @brief Minimal MPU-6050 accelerometer driver (no gyro, no DMP).
 *
 * Responsibilities:
 *  - Owns register programming
 *  - Provides raw accel samples in LSB
 *  - No application logic
 */

#ifndef IMU_MPU6050_ACCEL_DRIVER_HPP
#define IMU_MPU6050_ACCEL_DRIVER_HPP

#include "platform/I2CProvider.hpp"
#include "platform/Platform.hpp"
#include <cstdint>

namespace imu {

/**
 * @brief Raw accelerometer sample (LSB).
 *
 * Units:
 *  - accel: signed 16-bit, scale depends on AFS_SEL
 *  - timestamp: platform microseconds
 */
struct AccelSample {
  int16_t x;
  int16_t y;
  int16_t z;
  platform::TickUs timestampUs;
};

/**
 * @brief Accelerometer full-scale range (datasheet AFS_SEL).
 */
enum class AccelRange : uint8_t {
  PlusMinus2G = 0,  // 16384 LSB/g
  PlusMinus4G = 1,  // 8192  LSB/g
  PlusMinus8G = 2,  // 4096  LSB/g
  PlusMinus16G = 3, // 2048  LSB/g
};

/**
 * @brief Digital low-pass filter configuration.
 *
 * Applies to accel signal conditioning.
 */
enum class AccelDlpf : uint8_t {
  Hz5 = 6,
  Hz10 = 5,
  Hz20 = 4,
  Hz42 = 3,
  Hz98 = 2,
  Hz188 = 1,
  Off = 0,
};

/**
 * @brief Output data rate configuration.
 *
 * Accel internal rate is 1 kHz.
 */
struct AccelOdr {
  uint16_t hz;     // Desired output rate (e.g. 100 Hz)
  uint8_t divider; // SMPLRT_DIV value
};

/**
 * @brief MPU-6050 accelerometer configuration.
 */
struct AccelConfig {
  AccelRange range;
  AccelDlpf dlpf;
  AccelOdr odr;
};

/**
 * @brief MPU-6050 accelerometer-only driver.
 *
 * Threading:
 *  - Blocking I2C
 *  - Call from acquisition thread only
 */
class Mpu6050AccelDriver {
public:
  explicit Mpu6050AccelDriver(platform::I2CProvider &i2c);

  platform::Result init();
  platform::Result configure(const AccelConfig &config);
  platform::Result readSample(AccelSample &sample);

private:
  platform::I2CProvider &_i2c;
  static constexpr uint8_t I2C_ADDR = 0x68;
};

} // namespace imu

#endif // IMU_MPU6050_ACCEL_DRIVER_HPP
