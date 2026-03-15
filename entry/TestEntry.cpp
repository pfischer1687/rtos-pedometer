/**
 * @file entry/TestEntry.cpp
 * @brief Minimal test implementation of firmware entry (infinite loop only).
 */

#include "entry/FirmwareEntry.hpp"
#include "entry/HITLProtocol.hpp"
#include "usb/UsbInterface.hpp"
#include "usb/UsbTransport.hpp"
#include <cstring>

namespace entry {

namespace {

/**
 * @brief Trim leading and trailing whitespace (space or tab) from a buffer.
 * @param buffer Input C-string buffer.
 * @return string_view pointing to the trimmed content.
 */
constexpr std::string_view trimBuffer(const char *buffer) noexcept {
  if (!buffer)
    return {};

  const char *begin = buffer;
  const char *end = buffer;
  while (*end != '\0')
    ++end;

  while (begin < end && (*begin == ' ' || *begin == '\t'))
    ++begin;

  const char *last = end;
  while (last > begin && (*(last - 1) == ' ' || *(last - 1) == '\t'))
    --last;

  return std::string_view(begin, last - begin);
}

} // anonymous namespace

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
      std::string_view trimmed = trimBuffer(buffer);
      size_t line_len = trimmed.size();
      std::memcpy(response, prefix, prefix_len);
      std::memcpy(response + prefix_len, trimmed.data(), line_len);
      response[prefix_len + line_len] = '\0';
      usbInterface.sendResponse(response);

      const std::optional<HITLCommand> cmd = parseHITLCommand(trimmed);
      dispatchHITLCommand(usbInterface, cmd);
      usbInterface.printPrompt();
    }
  }
}

} // namespace entry
