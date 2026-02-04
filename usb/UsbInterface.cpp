/** \file usb/UsbInterface.cpp
 *  USB command parsing and response implementation.
 */

#include "usb/UsbInterface.hpp"

namespace usb {

platform::Result UsbInterface::init()
{
    return platform::Result::Ok;
}

bool UsbInterface::poll(ParsedCommand& cmd)
{
    cmd.id = CommandId::None;
    return false;
}

void UsbInterface::sendStatus(const session::SessionSnapshot& snapshot)
{
}

void UsbInterface::sendResponse(const char* msg)
{
}

void UsbInterface::sendError(const char* msg)
{
}

} // namespace usb
