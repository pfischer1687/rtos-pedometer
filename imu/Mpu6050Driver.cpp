/**
 * @file imu/Mpu6050Driver.cpp
 * @brief MPU-6050 driver implementation.
 */

#include "imu/Mpu6050Driver.hpp"
#include <atomic>
#include <cstring>
#include <optional>

namespace imu {

namespace {

/**
 * @brief MPU-6050 WHO_AM_I expected value.
 */
constexpr uint8_t MPU6050_WHO_AM_I_EXPECTED = 0x68u;

/**
 * @brief DEVICE_RESET bit mask.
 * @details
 * Datasheet:
 * - 0x6B: PWR_MGMT_1
 * - Bit7: DEVICE_RESET
 * - 0x01 << 7 = 0x80
 */
constexpr uint8_t DEVICE_RESET_BIT = 0x80u;

/**
 * @brief DEVICE_RESET delay in milliseconds.
 */
constexpr uint32_t DEVICE_RESET_DELAY_MS = 100u;

/**
 * @brief Default value for PWR_MGMT_1 register.
 * @details
 * Datasheet:
 * - 0x6B: PWR_MGMT_1
 * - Default value: 0x40
 */
constexpr uint8_t PWR_MGMT_1_DEFAULT_VALUE = 0x40u;

/**
 * @brief Data Ready enable bit mask.
 * @details
 * Datasheet:
 * - 0x38: INT_ENABLE
 * - Bit0: DATA_RDY_EN
 */
constexpr uint8_t DATA_RDY_EN_BIT = 0x01u;

/**
 * @brief Data Ready interrupt bit mask.
 * @details
 * Datasheet:
 * - 0x3A: INT_STATUS
 * - Bit0: DATA_RDY_INT
 */
constexpr uint8_t DATA_RDY_INT_BIT = 0x01u;

/**
 * @brief Maximum number of attempts to read registers.
 */
constexpr uint8_t MAX_READ_RETRY_ATTEMPTS = 2u;

/**
 * @brief Backoff delay in microseconds between read retry attempts.
 */
constexpr uint32_t READ_RETRY_BACKOFF_US = 50u;

/**
 * @brief Clock source selection bit mask.
 * @details
 * Datasheet:
 * - 0x6B: PWR_MGMT_1
 * - Bit[2:0]: CLKSEL[2:0]
 * - CLKSEL = 1 -> PLL with X axis gyroscope reference
 * - "It is highly recommended that the device be configured to use one of the
 * gyroscopes as the clock reference for improved stability."
 */
constexpr uint8_t CLKSEL_PLL_XGYRO = 0x01u;

/**
 * @brief Gyroscope disable bit mask.
 * @details
 * Datasheet:
 * - 0x6C: PWR_MGMT_2
 * - Bit[2:0]: STBY_XG, STBY_YG, STBY_ZG
 * - (0x01 << 3) - 1 = 0x07
 */
constexpr uint8_t GYRO_DISABLE_BIT = 0x07u;

/**
 * @enum GyroscopeSampleRate
 * @brief Gyroscope sample rate.
 * @details
 * From the MPU-6050 datasheet:
 * - DLPFCFG = 0 + SAMPLERATEDIV = 0 -> 8 kHz
 * - DLPFCFG=1,2,3,4,5, or 6 + SAMPLERATEDIV = 0 -> 1 kHz
 */
enum class GyroscopeSampleRate : uint16_t {
  Hz1000 = 1000u,
  Hz8000 = 8000u,
};

/**
 * @brief Get gyroscope sample rate from DlpfConfig.
 * @param dlpf DlpfConfig
 * @return GyroscopeSampleRate.
 */
constexpr GyroscopeSampleRate getGyroscopeSampleRate(DlpfConfig dlpf) noexcept {
  if (dlpf == DlpfConfig::Off) {
    return GyroscopeSampleRate::Hz8000;
  }
  return GyroscopeSampleRate::Hz1000;
}

/**
 * @brief Compute the divider for the gyroscope sample rate.
 * @param odr OutputDataRate
 * @param dlpf DlpfConfig
 * @return optional<uint8_t>.
 * @details
 *  - SampleRate = gyro_rate / (1 + SMPLRT_DIV)
 *  - SMPLRT_DIV = (gyro_rate / SampleRate) - 1
 *  - max: (8000 Hz / 50 Hz) - 1 = 159
 *  - min: (1000 Hz / 1000 Hz) - 1 = 0
 */
constexpr std::optional<uint8_t> computeSmplrtDiv(OutputDataRate odr,
                                                  DlpfConfig dlpf) noexcept {
  const uint32_t gyroRate = static_cast<uint32_t>(getGyroscopeSampleRate(dlpf));
  const uint32_t requested = static_cast<uint32_t>(odr);

  if (requested == 0u || requested > gyroRate || (gyroRate % requested) != 0u) {
    return std::nullopt;
  }

  const uint32_t rawDivider = gyroRate / requested;

  const uint32_t maxDivider = 0x01 << 8;
  if (rawDivider == 0u || rawDivider > maxDivider) {
    return std::nullopt;
  }

  return static_cast<uint8_t>(rawDivider - 1u);
}

/**
 * @brief Decode a 16-bit sample from a buffer.
 * @param buf Buffer pointer
 * @param index Index of the sample to decode.
 * @return Decoded sample.
 * @details
 * From the MPU-6050 register map:
 * - 0x3B: ACCEL_XOUT_H = ACCEL_XOUT[15:8]
 * - 0x3C: ACCEL_XOUT_L = ACCEL_XOUT[7:0]
 * - ...
 * - 0x40: ACCEL_ZOUT_L = ACCEL_ZOUT[7:0]
 * Each sample is given by two 2's complement bytes in big endian order, so here
 * we decode each two byte chunk into a 16-bit 2's complement sample.
 */
inline int16_t decode16BitsFromSampleBuffer(const uint8_t *buf,
                                            size_t index) noexcept {
  const uint16_t high = static_cast<uint16_t>(buf[index]);
  const uint16_t low = static_cast<uint16_t>(buf[index + 1]);
  return static_cast<int16_t>((high << 8) | low);
}

/**
 * @brief Decode a sample from a buffer.
 * @param buf Buffer pointer
 * @param sample Sample to decode into.
 * @details
 * - Currently acceleration-focused only.
 */
void decodeSampleFromBuffer(const uint8_t *buf, ImuSample &sample) noexcept {
  for (size_t i = 0; i < 3; i++) {
    sample.accel[i] = decode16BitsFromSampleBuffer(buf, i * 2);
  }
}

/**
 * @brief Position of AFS_SEL bit in ACCEL_CONFIG register.
 * @details
 * Datasheet:
 * - 0x1C: ACCEL_CONFIG
 * - Bits [0:2] are reserved.
 * - Bits [3:4] are AFS_SEL[1:0].
 */
constexpr uint8_t ACCEL_CONFIG_AFS_SEL_POS = 3u;

/**
 * @brief Accelerometer full-scale range bit mask.
 * @details
 * Datasheet:
 * - 0x1C: ACCEL_CONFIG
 * - Bits [0:2] are reserved.
 * - Bits [3:4] are AFS_SEL[1:0].
 * - (0x01 << 2) - 1 = 0x03
 */
constexpr uint8_t ACCEL_CONFIG_AFS_SEL_MASK = 0x03u;

/**
 * @brief Encode accelerometer range into ACCEL_CONFIG register value.
 * @param range AccelRange
 * @return ACCEL_CONFIG register value.
 */
inline constexpr uint8_t encodeAccelRange(imu::AccelRange range) noexcept {
  return (static_cast<uint8_t>(range) & ACCEL_CONFIG_AFS_SEL_MASK)
         << ACCEL_CONFIG_AFS_SEL_POS;
}

} // anonymous namespace

Mpu6050Driver::Mpu6050Driver(platform::II2CProvider &i2c,
                             platform::ITimer &timer,
                             uint8_t i2cAddr7Bit) noexcept
    : _state(DriverState::Uninitialized), _i2c(i2c), _timer(timer),
      _i2cAddr7Bit(i2cAddr7Bit), _dataReadyFlag(false),
      _lastDataReadyTimestampUs(0u) {}

platform::Result Mpu6050Driver::writeReg(Register reg, uint8_t value) noexcept {
  const uint8_t tx[2] = {static_cast<uint8_t>(reg), value};
  return _i2c.write(_i2cAddr7Bit, tx, 2);
}

platform::Result Mpu6050Driver::readRegs(Register start, uint8_t *buf,
                                         size_t len) noexcept {
  if (buf == nullptr || len == 0u) {
    return platform::Result::InvalidArgument;
  }

  const uint8_t tx[1] = {static_cast<uint8_t>(start)};
  return _i2c.transfer(_i2cAddr7Bit, tx, 1, buf, len);
}

platform::Result Mpu6050Driver::readRegsWithRetry(Register start, uint8_t *buf,
                                                  size_t len) noexcept {
  platform::Result lastReadResult = platform::Result::HardwareFault;

  for (uint8_t i = 0; i <= MAX_READ_RETRY_ATTEMPTS; ++i) {
    lastReadResult = readRegs(start, buf, len);
    if (platform::isOk(lastReadResult)) {
      return lastReadResult;
    }

    _timer.delayUs(READ_RETRY_BACKOFF_US);
  }

  return lastReadResult;
}

platform::Result Mpu6050Driver::writeRegVerified(Register reg,
                                                 uint8_t value) noexcept {
  const platform::Result writeResult = writeReg(reg, value);
  if (!platform::isOk(writeResult)) {
    return writeResult;
  }

  uint8_t readBack = 0;
  const platform::Result readResult = readRegsWithRetry(reg, &readBack, 1);
  if (!platform::isOk(readResult)) {
    return readResult;
  }
  if (readBack != value) {
    return platform::Result::HardwareFault;
  }

  return platform::Result::Ok;
}

void Mpu6050Driver::enterFaultState() noexcept {
  if (_state == DriverState::Fault) {
    return;
  }

  _dataReadyFlag.store(false, std::memory_order_release);
  _state = DriverState::Fault;
}

platform::Result Mpu6050Driver::fault(platform::Result r) noexcept {
  enterFaultState();
  return r;
}

platform::Result Mpu6050Driver::healthCheck() noexcept {
  uint8_t whoAmI = 0;
  const platform::Result whoAmIResult =
      readRegs(Register::WHO_AM_I, &whoAmI, 1);
  if (!platform::isOk(whoAmIResult))
    return whoAmIResult;
  if (whoAmI != MPU6050_WHO_AM_I_EXPECTED)
    return platform::Result::HardwareFault;

  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::reset() noexcept {
  _dataReadyFlag.store(false, std::memory_order_release);
  _lastDataReadyTimestampUs.store(0u, std::memory_order_release);

  const platform::Result disableIntResult =
      writeReg(Register::INT_ENABLE, 0x00u);
  if (!platform::isOk(disableIntResult))
    return fault(disableIntResult);

  const platform::Result resetPwrMgmt1Result =
      writeReg(Register::PWR_MGMT_1, DEVICE_RESET_BIT);
  if (!platform::isOk(resetPwrMgmt1Result))
    return fault(resetPwrMgmt1Result);

  _timer.delayMs(DEVICE_RESET_DELAY_MS);

  const platform::Result healthCheckResult = healthCheck();
  if (!platform::isOk(healthCheckResult))
    return fault(healthCheckResult);

  uint8_t pwrMgmt1 = 0;
  const platform::Result checkFullResetResult =
      readRegs(Register::PWR_MGMT_1, &pwrMgmt1, 1);
  if (!platform::isOk(checkFullResetResult) ||
      pwrMgmt1 != PWR_MGMT_1_DEFAULT_VALUE)
    return fault(platform::Result::HardwareFault);

  _state = DriverState::Uninitialized;
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::init() noexcept {
  if (_state != DriverState::Uninitialized) {
    return platform::Result::InvalidState;
  }

  const platform::Result resetResult = reset();
  if (!platform::isOk(resetResult))
    return fault(resetResult);

  const platform::Result wakeResult =
      writeRegVerified(Register::PWR_MGMT_1, CLKSEL_PLL_XGYRO);
  if (!platform::isOk(wakeResult))
    return fault(wakeResult);

  _state = DriverState::Initialized;
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::configure(const ImuConfig &config) noexcept {
  if (_state != DriverState::Initialized) {
    return platform::Result::InvalidState;
  }

  const std::optional<uint8_t> smplRtDiv =
      computeSmplrtDiv(config.odr, config.dlpf);
  if (!smplRtDiv.has_value()) {
    return platform::Result::InvalidArgument;
  }

  const platform::Result configDlpfResult =
      writeRegVerified(Register::CONFIG, static_cast<uint8_t>(config.dlpf));
  if (!platform::isOk(configDlpfResult))
    return fault(configDlpfResult);

  const platform::Result configAccelRangeResult =
      writeRegVerified(Register::ACCEL_CONFIG, encodeAccelRange(config.range));
  if (!platform::isOk(configAccelRangeResult))
    return fault(configAccelRangeResult);

  const platform::Result configSmpLrtDivResult =
      writeRegVerified(Register::SMPLRT_DIV, smplRtDiv.value());
  if (!platform::isOk(configSmpLrtDivResult))
    return fault(configSmpLrtDivResult);

  const platform::Result disableGyroResult =
      writeRegVerified(Register::PWR_MGMT_2, GYRO_DISABLE_BIT);
  if (!platform::isOk(disableGyroResult))
    return fault(disableGyroResult);

  _state = DriverState::Configured;
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::startSampling() noexcept {
  if (_state != DriverState::Configured) {
    return platform::Result::InvalidState;
  }

  const platform::Result enableIntResult =
      writeRegVerified(Register::INT_ENABLE, DATA_RDY_EN_BIT);
  if (!platform::isOk(enableIntResult))
    return fault(enableIntResult);

  _dataReadyFlag.store(false, std::memory_order_release);
  _state = DriverState::Sampling;
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::stopSampling() noexcept {
  if (_state != DriverState::Sampling) {
    return platform::Result::InvalidState;
  }

  const platform::Result disableIntResult =
      writeReg(Register::INT_ENABLE, 0x00u);
  if (!platform::isOk(disableIntResult))
    return fault(disableIntResult);

  _dataReadyFlag.store(false, std::memory_order_release);
  _state = DriverState::Configured;
  return platform::Result::Ok;
}

void Mpu6050Driver::notifyDataReadyFromIsr() noexcept {
  if (_state != DriverState::Sampling) {
    return;
  }

  _lastDataReadyTimestampUs.store(_timer.nowUs(), std::memory_order_release);
  _dataReadyFlag.store(true, std::memory_order_release);
}

bool Mpu6050Driver::consumeDataReady() noexcept {
  return _dataReadyFlag.exchange(false, std::memory_order_acq_rel);
}

platform::Result Mpu6050Driver::readSample(ImuSample &sample) noexcept {
  if (_state != DriverState::Sampling) {
    return platform::Result::InvalidState;
  }

  uint8_t intStatus = 0u;
  const platform::Result intStatusResult =
      readRegs(Register::INT_STATUS, &intStatus, 1);
  if (!platform::isOk(intStatusResult))
    return fault(intStatusResult);
  if ((intStatus & DATA_RDY_INT_BIT) == 0u) {
    return platform::Result::DataNotReady;
  }

  uint8_t buf[SAMPLE_BUF_LEN];
  platform::Result readResult =
      readRegsWithRetry(Register::ACCEL_XOUT_H, buf, SAMPLE_BUF_LEN);
  if (!platform::isOk(readResult))
    return fault(readResult);

  decodeSampleFromBuffer(buf, sample);
  sample.timestampUs =
      _lastDataReadyTimestampUs.load(std::memory_order_acquire);
  return platform::Result::Ok;
}

} // namespace imu
