/**
 * @file: imu/Mpu6050Driver.cpp
 *
 * @brief: MPU-6050 driver implementation.
 */

#include "imu/Mpu6050Driver.hpp"

namespace imu {

Mpu6050Driver::Mpu6050Driver(platform::I2CProvider &i2c) : _i2c(i2c) {}

platform::Result Mpu6050Driver::init() { return platform::Result::Ok; }

platform::Result Mpu6050Driver::configure(const ImuConfig &config) {
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::readSample(ImuSample &sample) {
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::writeReg(Register reg, uint8_t value) {
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::readRegs(Register start, uint8_t *buf,
                                         size_t len) {
  return platform::Result::Ok;
}

uint8_t Mpu6050Driver::odrToDivider(OutputDataRate odr) {
  return static_cast<uint16_t>(1000 / static_cast<uint16_t>(odr) - 1);
}

} // namespace imu
