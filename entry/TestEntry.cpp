/**
 * @file entry/TestEntry.cpp
 * @brief Minimal test implementation of firmware entry (infinite loop only).
 */

#include "entry/FirmwareEntry.hpp"
#include "usb/UsbInterface.hpp"
#include "usb/UsbTransport.hpp"

namespace entry {

[[noreturn]] int firmware_entry() {
  usb::UsbTransport transport;
  usb::UsbInterface usbInterface(transport);
  char buffer[usb::USB_CMD_MAX_LEN];

  while (true) {
    if (usbInterface.poll(buffer, sizeof(buffer))) {
      usbInterface.sendResponse(buffer);
    }
  }
}

} // namespace entry
