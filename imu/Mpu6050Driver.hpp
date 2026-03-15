/**
 * @file imu/Mpu6050Driver.hpp
 * @brief MPU-6050 IMU driver.
 * @details
 * - This interface is currently accelerometer-focused for the pedometer MVP.
 * - Gyroscope samples may be added in the future for gait analysis.
 */

#ifndef IMU_MPU6050_DRIVER_HPP
#define IMU_MPU6050_DRIVER_HPP

#include "platform/Gpio.hpp"
#include "platform/I2CProvider.hpp"
#include "platform/Platform.hpp"
#include <array>
#include <atomic>
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
 * @brief Get sample buffer length.
 * @return Sample buffer length.
 * @details
 * - According to MPU-6050 register map:
 *   - 0x3B: ACCEL_XOUT_H
 *   - 0x3C: ACCEL_XOUT_L
 *   - ...
 *   - 0x40: ACCEL_ZOUT_L
 *   - 0x41: TEMP_OUT_H
 *   - 0x42: TEMP_OUT_L
 *   - 0x43: GYRO_XOUT_H
 *   - ...
 *   - 0x48: GYRO_ZOUT_L
 * - For accel only, we need the first 6 bytes. To include the gyroscope we'd
 * need 14.
 */
constexpr uint8_t SAMPLE_BUF_LEN = 6u;

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
 * - Any -> Fault: enterFaultState()
 */
enum class DriverState : uint8_t {
  Uninitialized,
  Initialized,
  Configured,
  Sampling,
  Fault
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
   * @param timer Timer.
   * @param i2cAddr7Bit I2C address (7-bit).
   */
  explicit Mpu6050Driver(platform::II2CProvider &i2c, platform::ITimer &timer,
                         uint8_t i2cAddr7Bit) noexcept;

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
   * @brief Enter fault state.
   * @details
   * - Transitions state from any to Fault.
   */
  void enterFaultState() noexcept;

  /**
   * @brief Notify the driver that data-ready was asserted.
   * @details
   * - ISR-safe: no blocking or allocation.
   * - Only valid when state is Sampling.
   * - Sets data-ready flag and captures ISR timestamp for sample timestamp.
   */
  void notifyDataReadyFromIsr() noexcept;

  /**
   * @brief Static callback for data-ready interrupt.
   * @param ctx Context pointer.
   */
  static void notifyDataReadyFromIsrStatic(void *ctx) noexcept;

  /**
   * @brief Attach the data-ready input.
   * @param gpio Data-ready input.
   */
  void attachDataReadyInput(platform::IDataReadyInput &gpio) noexcept;

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
   * - Returns DataNotReady if data is not ready.
   * - Captures ISR timestamp for sample timestamp.
   */
  platform::Result readSample(ImuSample &sample) noexcept;

  /**
   * @brief Return current driver state.
   * @return DriverState: Current state.
   */
  DriverState getState() const noexcept { return _state; }

  /**
   * @brief Check if driver is ready for initialization.
   * @return bool.
   */
  bool isReadyForInit() const noexcept {
    return _state == DriverState::Uninitialized;
  }

  /**
   * @brief Check if driver is ready for configuration.
   * @return bool.
   */
  bool isReadyForConfig() const noexcept {
    return _state == DriverState::Initialized;
  }

  /**
   * @brief Check if driver is ready for sampling.
   * @return bool.
   */
  bool isReadyForSampling() const noexcept {
    return (_state == DriverState::Configured) && (_dataReadyInput != nullptr);
  }

  /**
   * @brief Check if driver is sampling.
   * @return bool.
   */
  bool isSampling() const noexcept { return _state == DriverState::Sampling; }

  /**
   * @brief Check if driver is in fault state.
   * @return bool.
   */
  bool isFault() const noexcept { return _state == DriverState::Fault; }

  /**
   * @brief Perform a health check of the MPU-6050.
   * @return Result
   */
  platform::Result healthCheck() noexcept;

private:
  /**
   * @enum Register
   * @brief MPU-6050 register map (subset).
   */
  enum class Register : uint8_t {
    INT_ENABLE = 0x38,
    INT_STATUS = 0x3A,
    SMPLRT_DIV = 0x19,
    CONFIG = 0x1A,
    ACCEL_CONFIG = 0x1C,
    PWR_MGMT_1 = 0x6B,
    PWR_MGMT_2 = 0x6C,
    ACCEL_XOUT_H = 0x3B,
    WHO_AM_I = 0x75,
  };

  /**
   * @brief Write a single register.
   * @param reg Register address
   * @param value Register value
   * @return Result
   */
  platform::Result writeReg(Register reg, uint8_t value) noexcept;

  /**
   * @brief Read multiple registers.
   * @param start Register address
   * @param buf Buffer to store the registers
   * @param len Number of registers to read
   * @return Result
   */
  platform::Result readRegs(Register start, uint8_t *buf, size_t len) noexcept;

  /**
   * @brief Write a single register and verify the value.
   * @param reg Register address
   * @param value Register value
   * @return Result
   */
  platform::Result writeRegVerified(Register reg, uint8_t value) noexcept;

  /**
   * @brief Read multiple registers with retry.
   * @param start Register address
   * @param buf Buffer to store the registers
   * @param len Number of registers to read
   * @return Result
   */
  platform::Result readRegsWithRetry(Register start, uint8_t *buf,
                                     size_t len) noexcept;

  /**
   * @brief Helper function to set the driver to fault state and return the
   * result.
   * @param r Result
   * @return Result
   */
  platform::Result fault(platform::Result r) noexcept;

  /**
   * @brief Current driver state.
   * @details
   * - Main context owns state transitions.
   * - ISR reads _state and does not modify it.
   * - Assuming single-core Cortex-M7 (STM32 Nucleo F767ZI).
   */
  DriverState _state{DriverState::Uninitialized};

  platform::II2CProvider &_i2c;
  platform::ITimer &_timer;
  uint8_t _i2cAddr7Bit{0};
  platform::IDataReadyInput *_dataReadyInput{nullptr};
  std::atomic<bool> _dataReadyFlag{false};
  std::atomic<platform::TickUs> _lastDataReadyTimestampUs{0};
};

} // namespace imu

#endif // IMU_MPU6050_DRIVER_HPP
