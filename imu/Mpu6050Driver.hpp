/**
 * @file imu/Mpu6050Driver.hpp
 * @brief MPU-6050 IMU driver.
 * @details
 * - This interface is currently accelerometer-focused for the pedometer MVP.
 * - Gyroscope samples may be added in the future for gait analysis.
 */

#ifndef IMU_MPU6050_DRIVER_HPP
#define IMU_MPU6050_DRIVER_HPP

#include "platform/I2CProvider.hpp"
#include "platform/Platform.hpp"

#include <array>
#include <cstdint>

namespace imu {

/**
 * @struct ImuSample
 * @brief Raw IMU sample (device LSB units).
 * @details
 * - Accelerometer outputs are signed 16-bit values from ACCEL_*OUT registers.
 * - Scaling to physical units is left to higher layers.
 * - Timestamp is captured at read time, not sample time.
 */
struct ImuSample {
  std::array<int16_t, 3> accel{{0, 0, 0}}; // X, Y, Z
  platform::TickUs timestampUs{0};
};

/**
 * @enum AccelRange
 * @brief Accelerometer full-scale range (CONFIG.AFS_SEL).
 */
enum class AccelRange : uint8_t {
  PlusMinus2G = 0,  // 16384 LSB/g
  PlusMinus4G = 1,  // 8192  LSB/g
  PlusMinus8G = 2,  // 4096  LSB/g
  PlusMinus16G = 3, // 2048  LSB/g
};

/**
 * @enum DlpfConfig
 * @brief Digital Low-Pass Filter configuration (CONFIG.DLPF_CFG).
 * @details
 * - Also controls gyro internal sampling rate (1 kHz vs 8 kHz).
 * - Accelerometer ADC rate remains 1 kHz.
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
 * @enum OutputDataRate
 * @brief Output Data Rate for sensor registers / FIFO / DMP.
 * @details
 * - Internally mapped to SMPLRT_DIV.
 * - Effective accel ODR = min(1 kHz, gyro_rate / (1 + divider)).
 */
enum class OutputDataRate : uint16_t {
  Hz1000 = 1000,
  Hz500 = 500,
  Hz200 = 200,
  Hz100 = 100,
  Hz50 = 50,
};

/**
 * @struct ImuConfig
 * @brief MPU-6050 configuration.
 */
struct ImuConfig {
  AccelRange range{AccelRange::PlusMinus2G};
  DlpfConfig dlpf{DlpfConfig::Hz44};
  OutputDataRate odr{OutputDataRate::Hz100};
};

/**
 * @enum DriverState
 * @brief Explicit driver state for the MPU-6050 state machine.
 * @details
 * Transitions:
 * - Uninitialized -> Initialized: init()
 * - Initialized -> Configured: configure()
 * - Configured -> Sampling: startSampling()
 * - Sampling -> Configured: stopSampling()
 * - Any -> Uninitialized: reset()
 */
enum class DriverState : uint8_t {
  Uninitialized,
  Initialized,
  Configured,
  Sampling,
};

/**
 * @class Mpu6050Driver
 * @brief MPU-6050 driver.
 */
class Mpu6050Driver {
public:
  /**
   * @brief Constructor.
   * @param i2c I2C provider.
   * @param tickSource Microsecond tick source.
   */
  explicit Mpu6050Driver(platform::II2CProvider &i2c,
                         platform::ITickSource &tickSource) noexcept;

  /**
   * @brief Initialize device and bring it out of reset.
   * @return Result.
   * @details
   * - Must succeed before configure() may be called.
   * - Transitions state from Uninitialized to Initialized on success.
   */
  platform::Result init() noexcept;

  /**
   * @brief Configure accelerometer and sampling behavior.
   * @param config Configuration.
   * @return Result.
   * @details
   * - Must be called only after init() has succeeded.
   * - Transitions state from Initialized to Configured on success.
   */
  platform::Result configure(const ImuConfig &config) noexcept;

  /**
   * @brief Reset driver to Uninitialized state.
   * @return Result.
   * @details
   * - init() must be called again before configure() and startSampling() may be
   * called.
   */
  platform::Result reset() noexcept;

  /**
   * @brief Start sampling.
   * @return Result.
   * @details
   * - Must be called only after configure() has succeeded.
   * - Transitions state from Configured to Sampling.
   */
  platform::Result startSampling() noexcept;

  /**
   * @brief Stop sampling and return to Configured state.
   * @return Result.
   * @details
   * - Transitions state from Sampling to Configured.
   */
  platform::Result stopSampling() noexcept;

  /**
   * @brief Notify the driver that data-ready was asserted.
   * @details
   * - ISR-safe: no blocking or allocation.
   */
  void notifyDataReadyFromIsr() noexcept;

  /**
   * @brief Consume the data-ready signal.
   * @return True if data-ready had been set since last consume; false
   * otherwise.
   */
  bool consumeDataReady() noexcept;

  /**
   * @brief Read one accelerometer sample.
   * @param sample Sample to read into.
   * @return Result.
   * @details
   * - Only valid when state is Sampling.
   */
  platform::Result readSample(ImuSample &sample) noexcept;

  /**
   * @brief Return current driver state.
   * @return DriverState: Current state.
   */
  DriverState getState() const noexcept { return _state; }

  /**
   * @brief Check if driver is initialized.
   * @return bool.
   */
  bool isInitialized() const noexcept {
    return _state != DriverState::Uninitialized;
  }

private:
  /**
   * @enum Register
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
   * @param timeoutMs Timeout in milliseconds.
   * @return Result
   */
  platform::Result writeReg(Register reg, uint8_t value,
                            uint32_t timeoutMs) noexcept;

  /**
   * @brief Read multiple registers.
   * @param start Register address
   * @param buf Buffer to store the registers
   * @param len Number of registers to read
   * @param timeoutMs Timeout in milliseconds.
   * @return Result
   */
  platform::Result readRegs(Register start, uint8_t *buf, size_t len,
                            uint32_t timeoutMs) noexcept;

  /**
   * @brief Convert OutputDataRate to sample rate divider.
   * @param odr OutputDataRate
   * @return Sample rate divider
   * @details
   *  SampleRate = gyro_rate / (1 + SMPLRT_DIV)
   */
  static uint8_t odrToDivider(OutputDataRate odr) noexcept;

  platform::II2CProvider &_i2c;
  platform::ITickSource &_tickSource;
  DriverState _state{DriverState::Uninitialized};
  volatile bool _dataReadyFlag{false};
};

} // namespace imu

#endif // IMU_MPU6050_DRIVER_HPP
