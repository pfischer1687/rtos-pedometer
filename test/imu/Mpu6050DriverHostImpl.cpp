/**
 * @file test/imu/Mpu6050DriverHostImpl.cpp
 * @brief Full Mpu6050Driver implementation for host unit tests only.
 * @details Used instead of imu/Mpu6050Driver.cpp in the host test build so
 * tests link and run without modifying firmware sources. Implements WHO_AM_I
 * check, state machine, configure register writes, and readSample burst read.
 */

#include "imu/Mpu6050Driver.hpp"
#include "platform/Platform.hpp"
#include <cstddef>
#include <cstdint>

namespace imu {

namespace {

constexpr uint8_t MPU6050_DEFAULT_ADDR = 0x68u;

} // namespace

Mpu6050Driver::Mpu6050Driver(platform::II2CProvider &i2c,
                             platform::ITickSource &tickSource) noexcept
    : _i2c(i2c), _tickSource(tickSource) {}

platform::Result Mpu6050Driver::init() noexcept {
  uint8_t whoAmI = 0u;
  platform::Result r = readRegs(Register::WHO_AM_I, &whoAmI, 1, 100u);
  if (!platform::isOk(r)) {
    return r;
  }
  if (whoAmI != 0x68u) {
    return platform::Result::HardwareFault;
  }
  _state = DriverState::Initialized;
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::configure(const ImuConfig &config) noexcept {
  const uint8_t accelConfig =
      static_cast<uint8_t>(static_cast<uint8_t>(config.range) << 3);
  platform::Result r = writeReg(Register::ACCEL_CONFIG, accelConfig, 100u);
  if (!platform::isOk(r)) {
    return r;
  }
  r = writeReg(Register::CONFIG, static_cast<uint8_t>(config.dlpf), 100u);
  if (!platform::isOk(r)) {
    return r;
  }
  const uint8_t div = odrToDivider(config.odr);
  r = writeReg(Register::SMPLRT_DIV, div, 100u);
  if (!platform::isOk(r)) {
    return r;
  }
  _state = DriverState::Configured;
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::reset() noexcept {
  _state = DriverState::Uninitialized;
  _dataReadyFlag = false;
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::startSampling() noexcept {
  if (_state != DriverState::Configured) {
    return platform::Result::InvalidState;
  }
  _state = DriverState::Sampling;
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::stopSampling() noexcept {
  _state = DriverState::Configured;
  return platform::Result::Ok;
}

void Mpu6050Driver::notifyDataReadyFromIsr() noexcept { _dataReadyFlag = true; }

bool Mpu6050Driver::consumeDataReady() noexcept {
  const bool was = _dataReadyFlag;
  _dataReadyFlag = false;
  return was;
}

platform::Result Mpu6050Driver::readSample(ImuSample &sample) noexcept {
  if (_state != DriverState::Sampling) {
    return platform::Result::InvalidState;
  }
  uint8_t raw[6];
  platform::Result r = readRegs(Register::ACCEL_XOUT_H, raw, sizeof(raw), 100u);
  if (!platform::isOk(r)) {
    return r;
  }
  sample.accel[0] = static_cast<int16_t>(static_cast<uint16_t>(raw[0]) << 8u |
                                         static_cast<uint16_t>(raw[1]));
  sample.accel[1] = static_cast<int16_t>(static_cast<uint16_t>(raw[2]) << 8u |
                                         static_cast<uint16_t>(raw[3]));
  sample.accel[2] = static_cast<int16_t>(static_cast<uint16_t>(raw[4]) << 8u |
                                         static_cast<uint16_t>(raw[5]));
  sample.timestampUs = _tickSource.nowUs();
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::writeReg(Register reg, uint8_t value,
                                         uint32_t timeoutMs) noexcept {
  (void)timeoutMs;
  uint8_t buf[2] = {static_cast<uint8_t>(reg), value};
  return _i2c.write(MPU6050_DEFAULT_ADDR, buf, 2);
}

platform::Result Mpu6050Driver::readRegs(Register start, uint8_t *buf,
                                         size_t len,
                                         uint32_t timeoutMs) noexcept {
  (void)timeoutMs;
  const uint8_t reg = static_cast<uint8_t>(start);
  return _i2c.transfer(MPU6050_DEFAULT_ADDR, &reg, 1, buf, len);
}

uint8_t Mpu6050Driver::odrToDivider(OutputDataRate odr) noexcept {
  return static_cast<uint8_t>(1000u / static_cast<uint16_t>(odr) - 1u);
}

} // namespace imu
