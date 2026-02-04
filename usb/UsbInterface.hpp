/** \file usb/UsbInterface.hpp
 *  USB serial interface for inspection and control. Non-real-time; commands from host.
 *  Ownership: owns USB/serial handle; parses commands; delegates actions to app/session.
 */

#ifndef USB_USBINTERFACE_HPP
#define USB_USBINTERFACE_HPP

#include "platform/Platform.hpp"
#include "session/SessionManager.hpp"
#include <cstddef>
#include <cstdint>

namespace usb {

/** Maximum length of one command line (bytes). */
constexpr size_t USB_CMD_MAX_LEN = 64;

/** Parsed command for the application. */
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

/** One parsed command with optional argument. */
struct ParsedCommand {
    CommandId id;
    uint32_t arg;  // e.g. value for SetConfig
};

/** USB interface: line-based command parser, response output. */
class UsbInterface {
public:
    UsbInterface() = default;

    /** Initialize serial (e.g. USBSerial or BufferedSerial). */
    platform::Result init();

    /** Poll for incoming data; parse and return one command if complete. */
    bool poll(ParsedCommand& cmd);

    /** Send status line (step count, state). */
    void sendStatus(const session::SessionSnapshot& snapshot);

    /** Send raw response string. */
    void sendResponse(const char* msg);

    /** Send error message. */
    void sendError(const char* msg);
};

} // namespace usb

#endif /* USB_USBINTERFACE_HPP */
