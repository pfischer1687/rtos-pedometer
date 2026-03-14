/**
 * @file entry/TestEntry.cpp
 * @brief Minimal test implementation of firmware entry (infinite loop only).
 */

#include "entry/FirmwareEntry.hpp"
#include "usb/UsbInterface.hpp"
#include "usb/UsbTransport.hpp"
#include <cstring>

namespace entry {

[[noreturn]] int firmware_entry() {
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
      size_t line_len = std::strlen(buffer);
      std::memcpy(response, prefix, prefix_len);
      std::memcpy(response + prefix_len, buffer, line_len);
      response[prefix_len + line_len] = '\0';
      usbInterface.sendResponse(response);
      usbInterface.printPrompt();
    }
  }
}

} // namespace entry
