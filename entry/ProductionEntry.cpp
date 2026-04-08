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

constexpr uint8_t kImuI2cAddr7Bit = 0x68u;

} // namespace

[[noreturn]] int firmware_entry() {
  static imu::Mpu6050Driver imu(platform::i2c(), platform::timer(),
                                kImuI2cAddr7Bit);
  imu.attachDataReadyInput(platform::dataReadyInput());

  static app::Application application(imu);
  application.run();
}

} // namespace entry
