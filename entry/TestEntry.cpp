/**
 * @file entry/TestEntry.cpp
 * @brief Minimal test implementation of firmware entry (infinite loop only).
 */

#include "entry/FirmwareEntry.hpp"
#include "entry/HITLProtocol.hpp"
#include "imu/Mpu6050Driver.hpp"
#include "platform/Gpio.hpp"
#include "platform/I2CProvider.hpp"
#include "platform/Platform.hpp"
#include "platform/StringUtils.hpp"
#include "usb/UsbInterface.hpp"
#include "usb/UsbTransport.hpp"
#include <cstring>

namespace entry {

namespace {

/**
 * @brief MPU-6050 I2C address (7-bit).
 */
constexpr uint8_t I2C_ADDR_7_BIT = 0x68u;

} // anonymous namespace

[[noreturn]] int firmware_entry() {
  imu::Mpu6050Driver imu(platform::i2c(), platform::timer(), I2C_ADDR_7_BIT);
  imu.attachDataReadyInput(platform::dataReadyInput());

  usb::UsbTransport transport;
  usb::UsbInterface usbInterface(transport);

  usbInterface.sendResponse("BOOT TEST_MODE");
  usbInterface.printPrompt();

  char buffer[usb::USB_CMD_MAX_LEN];
  constexpr char prefix[] = "ECHO: ";
  constexpr size_t prefix_len = sizeof(prefix) - 1;
  char response[prefix_len + usb::USB_CMD_MAX_LEN + 1];

  while (true) {
    if (usbInterface.poll(buffer, sizeof(buffer))) {
      std::string_view trimmed = platform::str::trim(buffer);
      size_t line_len = trimmed.size();
      std::memcpy(response, prefix, prefix_len);
      std::memcpy(response + prefix_len, trimmed.data(), line_len);
      response[prefix_len + line_len] = '\0';
      usbInterface.sendResponse(response);

      const ParsedCommand parsedCmd = parseCommand(trimmed);
      dispatchHITLCommand(usbInterface, imu, parsedCmd);
      usbInterface.printPrompt();
    }
  }
}

} // namespace entry
