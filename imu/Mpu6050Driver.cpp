/**
 * @file imu/Mpu6050Driver.cpp
 * @brief MPU-6050 driver implementation.
 */

#include "imu/Mpu6050Driver.hpp"

namespace imu {

Mpu6050Driver::Mpu6050Driver(platform::II2CProvider &i2c,
                             platform::ITickSource &tickSource) noexcept
    : _i2c(i2c), _tickSource(tickSource) {}

platform::Result Mpu6050Driver::init() noexcept { return platform::Result::Ok; }

platform::Result Mpu6050Driver::configure(const ImuConfig &config) noexcept {
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::readSample(ImuSample &sample) noexcept {
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::writeReg(Register reg, uint8_t value,
                                         uint32_t timeoutMs) noexcept {
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::readRegs(Register start, uint8_t *buf,
                                         size_t len,
                                         uint32_t timeoutMs) noexcept {
  return platform::Result::Ok;
}

uint8_t Mpu6050Driver::odrToDivider(OutputDataRate odr) noexcept {
  return static_cast<uint16_t>(1000 / static_cast<uint16_t>(odr) - 1);
}

} // namespace imu
