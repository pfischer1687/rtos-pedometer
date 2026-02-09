/**
 * @file imu/Mpu6050Driver.hpp
 * @brief MPU-6050 IMU driver
 *
 * NOTE:
 *  - This interface is currently accelerometer-focused, gyroscope samples
 * are not included for pedometer MVP, but may be added in the future for gait
 * analysis, etc.
 */

#ifndef IMU_MPU6050_DRIVER_HPP
#define IMU_MPU6050_DRIVER_HPP

#include "platform/I2CProvider.hpp"
#include "platform/Platform.hpp"
#include <array>
#include <cstdint>

namespace imu {

/**
 * @brief Raw IMU sample (device LSB units).
 *
 * NOTE:
 *  - Accelerometer outputs are signed 16-bit values from ACCEL_*OUT registers.
 *  - Scaling to physical units is intentionally left to higher layers.
 *  - Timestamp is captured at read time, not sample time.
 */
struct ImuSample {
  std::array<int16_t, 3> accel; // X, Y, Z
  platform::TickUs timestampUs;
};

/**
 * @brief Accelerometer full-scale range (CONFIG.AFS_SEL).
 */
enum class AccelRange : uint8_t {
  PlusMinus2G = 0,  // 16384 LSB/g
  PlusMinus4G = 1,  // 8192  LSB/g
  PlusMinus8G = 2,  // 4096  LSB/g
  PlusMinus16G = 3, // 2048  LSB/g
};

/**
 * @brief Digital Low-Pass Filter configuration (CONFIG.DLPF_CFG).
 *
 * NOTE:
 *  - Also controls gyro internal sampling rate (1 kHz vs 8 kHz).
 *  - Accelerometer ADC rate remains 1 kHz.
 */
enum class DlpfConfig : uint8_t {
  Off = 0, // ~260 Hz accel bandwidth, 0 ms delay
  Hz184 = 1,
  Hz94 = 2,
  Hz44 = 3,
  Hz21 = 4,
  Hz10 = 5,
  Hz5 = 6,
};

/**
 * @brief Output Data Rate for sensor registers / FIFO / DMP.
 *
 * Internally mapped to SMPLRT_DIV.
 * Effective accel ODR = min(1 kHz, gyro_rate / (1 + divider)).
 */
enum class OutputDataRate : uint16_t {
  Hz1000 = 1000,
  Hz500 = 500,
  Hz200 = 200,
  Hz100 = 100,
  Hz50 = 50,
};

/**
 * @brief MPU-6050 configuration.
 */
struct ImuConfig {
  AccelRange range;
  DlpfConfig dlpf;
  OutputDataRate odr;
};

/**
 * @brief MPU-6050 driver.
 */
class Mpu6050Driver {
public:
  explicit Mpu6050Driver(platform::I2CProvider &i2c);

  /**
   * @brief Initialize device and bring it out of reset.
   */
  platform::Result init();

  /**
   * @brief Configure accelerometer and sampling behavior.
   *
   * Safe to call only after init().
   */
  platform::Result configure(const ImuConfig &config);

  /**
   * @brief Read one accelerometer sample.
   *
   * Reads ACCEL_XOUT_H .. ACCEL_ZOUT_L in a single burst.
   */
  platform::Result readSample(ImuSample &sample);

private:
  static constexpr uint8_t DefaultI2cAddress = 0x68;
  static constexpr uint8_t WhoAmIValue = 0x68;

  /**
   * @brief MPU-6050 register map (subset).
   */
  enum class Register : uint8_t {
    SMPLRT_DIV = 0x19,
    CONFIG = 0x1A,
    ACCEL_CONFIG = 0x1C,
    PWR_MGMT_1 = 0x6B,
    ACCEL_XOUT_H = 0x3B,
    WHO_AM_I = 0x75,
  };

  /**
   * @brief Write a single register.
   * @param reg Register address
   * @param value Register value
   * @return Result
   */
  platform::Result writeReg(Register reg, uint8_t value);

  /**
   * @brief Read multiple registers.
   * @param start Register address
   * @param buf Buffer to store the registers
   * @param len Number of registers to read
   * @return Result
   */
  platform::Result readRegs(Register start, uint8_t *buf, size_t len);

  /**
   * @brief Convert OutputDataRate to sample rate divider.
   * @param odr OutputDataRate
   * @return Sample rate divider
   *
   * NOTE:
   *  SampleRate = gyro_rate / (1 + SMPLRT_DIV)
   */
  static uint8_t odrToDivider(OutputDataRate odr);

  platform::I2CProvider &_i2c;
};

} // namespace imu

#endif // IMU_MPU6050_DRIVER_HPP
