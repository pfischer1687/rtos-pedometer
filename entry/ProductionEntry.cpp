/**
 * @file entry/ProductionEntry.cpp
 * @brief Entry point for the firmware.
 */

#include "app/Application.hpp"
#include "entry/FirmwareEntry.hpp"
#include "imu/Mpu6050Driver.hpp"
#include "platform/Gpio.hpp"
#include "platform/I2CProvider.hpp"
#include "platform/Platform.hpp"

namespace entry {

namespace {

constexpr uint8_t IMU_I2C_ADDR_7_BIT = 0x68u;

} // namespace

[[noreturn]] int firmware_entry() {
  static imu::Mpu6050Driver imu(platform::i2c(), platform::timer(),
                                IMU_I2C_ADDR_7_BIT);
  imu.attachDataReadyInput(platform::dataReadyInput());

  static app::Application application(imu, platform::watchdog());
  application.run();
}

} // namespace entry
