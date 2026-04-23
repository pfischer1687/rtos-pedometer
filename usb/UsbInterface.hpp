/**
 * @file usb/UsbInterface.hpp
 * @brief USB serial interface for inspection and control.
 */

#ifndef USB_USBINTERFACE_HPP
#define USB_USBINTERFACE_HPP

#include "usb/IUsbTransport.hpp"
#include <array>
#include <cstddef>
#include <string_view>

namespace usb {

struct ParsedLine {
  std::string_view name;
  std::array<std::string_view, 4> args{};
  std::size_t argCount = 0u;
};

/**
 * @brief Maximum length of one command line (bytes).
 */
inline constexpr size_t USB_CMD_MAX_LEN = 64u;

/**
 * @brief Maximum length of one response message (bytes).
 */
inline constexpr size_t USB_RESPONSE_MSG_MAX_BYTES = 384u;

/**
 * @brief Parse a line into a ParsedLine.
 * @param line The line to parse.
 * @return The parsed line.
 */
[[nodiscard]] ParsedLine parseLine(std::string_view line) noexcept;

/**
 * @class UsbInterface
 * @brief USB interface: line-based command parser, response output.
 */
class UsbInterface {
public:
  explicit UsbInterface(IUsbTransport &transport) noexcept;

  UsbInterface(const UsbInterface &) = delete;
  UsbInterface &operator=(const UsbInterface &) = delete;
  UsbInterface(UsbInterface &&) = delete;
  UsbInterface &operator=(UsbInterface &&) = delete;

  /**
   * @brief Poll for a new line from USB.
   * @param line_buffer Caller-provided buffer to store line
   * @param max_len Maximum length of the buffer
   * @return true if a line was received, false otherwise
   */
  [[nodiscard]] bool poll(char *line_buffer, size_t max_len) noexcept;

  /**
   * @brief Print the prompt to the USB.
   */
  void printPrompt() noexcept;

  /**
   * @brief Send raw response string.
   * @param msg Response string.
   */
  void sendResponse(const char *msg) noexcept;

  /**
   * @brief Send error message.
   * @param msg Error message.
   */
  void sendError(const char *msg) noexcept;

private:
  /**
   * @brief Handle backspace character.
   * @param c The character to handle.
   * @return true if the character was handled, false otherwise.
   */
  bool handleBackspace(char c) noexcept;

  /**
   * @brief Handle newline character.
   * @param c The character to handle.
   * @param line_buffer The line buffer to store the line.
   * @param max_len The maximum length of the line buffer.
   * @return true if the character was handled, false otherwise.
   */
  bool handleNewline(char c, char *line_buffer, std::size_t max_len) noexcept;

  /**
   * @brief Handle normal character.
   * @param c The character to handle.
   */
  void handleNormalChar(char c) noexcept;

  IUsbTransport &_transport;
  char _rx_buffer[USB_CMD_MAX_LEN]{};
  std::size_t _rx_index{0};
  bool _discarding{false};
  bool _lastWasCR{false};
};

} // namespace usb

#endif /* USB_USBINTERFACE_HPP */
