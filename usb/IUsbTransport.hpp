/**
 * @file usb/IUsbTransport.hpp
 * @brief Transport interface for USB.
 */

#ifndef USB_IUSBTRANSPORT_HPP
#define USB_IUSBTRANSPORT_HPP

#include <cstdint>
#include <optional>

namespace usb {

/**
 * @class IUsbTransport
 * @brief USB transport interface.
 */
class IUsbTransport {
public:
  virtual ~IUsbTransport() = default;

  IUsbTransport(const IUsbTransport &) = delete;
  IUsbTransport &operator=(const IUsbTransport &) = delete;
  IUsbTransport(IUsbTransport &&) = delete;
  IUsbTransport &operator=(IUsbTransport &&) = delete;

  /**
   * @brief Read a character via USB.
   * @return The character read, or std::nullopt if no character is available.
   */
  virtual std::optional<uint8_t> readChar() noexcept = 0;

  /**
   * @brief Write a character via USB.
   * @param c The character to write.
   */
  virtual void writeChar(char c) noexcept = 0;

  /**
   * @brief Write a line via USB.
   * @param msg The message to write.
   */
  virtual void writeLine(const char *msg) noexcept = 0;

protected:
  IUsbTransport() = default;
};

} // namespace usb

#endif /* USB_IUSBTRANSPORT_HPP */
