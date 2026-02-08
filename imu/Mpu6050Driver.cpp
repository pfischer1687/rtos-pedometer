/**
 * @file: imu/Mpu6050Driver.cpp
 * @brief: MPU-6050 driver implementation.
 */

#include "imu/Mpu6050Driver.hpp"

namespace imu {

Mpu6050Driver::Mpu6050Driver(platform::I2CProvider &i2c) : _i2c(i2c) {}

platform::Result Mpu6050Driver::init() { return platform::Result::Ok; }

platform::Result Mpu6050Driver::setConfig(const ImuConfig &config) {
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::readSample(ImuSample &sample) {
  return platform::Result::Ok;
}

platform::Result Mpu6050Driver::readFifo(ImuSample *samples, size_t maxCount,
                                         size_t &outCount) {
  outCount = 0;
  return platform::Result::Ok;
}

} // namespace imu
