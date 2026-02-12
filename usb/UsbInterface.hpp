/**
 * @file usb/UsbInterface.hpp
 * @brief USB serial interface for inspection and control.
 */

#ifndef USB_USBINTERFACE_HPP
#define USB_USBINTERFACE_HPP

#include "platform/Platform.hpp"
#include "session/SessionManager.hpp"
#include <cstddef>
#include <cstdint>

namespace usb {

/**
 * @brief Maximum length of one command line (bytes).
 */
constexpr size_t USB_CMD_MAX_LEN = 64;

/**
 * @enum CommandId
 * @brief Parsed command for the application.
 */
enum class CommandId : uint8_t {
  None,
  Start,
  Stop,
  Pause,
  Resume,
  GetStatus,
  GetSteps,
  SetConfig,
  Unknown,
};

/**
 * @struct ParsedCommand
 * @brief One parsed command with optional argument.
 */
struct ParsedCommand {
  CommandId id;
  uint32_t arg; // e.g. value for SetConfig
};

/**
 * @class UsbInterface
 * @brief USB interface: line-based command parser, response output.
 */
class UsbInterface {
public:
  /**
   * @brief Constructor.
   */
  UsbInterface() noexcept = default;

  /**
   * @brief Initialize serial (e.g. USBSerial or BufferedSerial).
   * @return Result of initialization.
   */
  platform::Result init() noexcept;

  /**
   * @brief Poll for incoming data; parse and return one command if complete.
   * @param cmd Parsed command.
   * @return True if a command was parsed, false otherwise.
   */
  bool poll(ParsedCommand &cmd) noexcept;

  /**
   * @brief Send status line (step count, state).
   * @param snapshot Session snapshot.
   */
  void sendStatus(const session::SessionSnapshot &snapshot) noexcept;

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
};

} // namespace usb

#endif /* USB_USBINTERFACE_HPP */
