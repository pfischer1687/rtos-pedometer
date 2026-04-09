/**
 * @file usb/Command.cpp
 * @brief Shared USB command line parsing implementation.
 */

#include "usb/Command.hpp"
#include "platform/StringUtils.hpp"

namespace usb {

namespace {

struct CommandEntry {
  std::string_view name;
  CommandId id;
};

constexpr CommandEntry commandTable[] = {
    {"PING", CommandId::Ping},
    {"INIT", CommandId::Init},
    {"CONFIGURE", CommandId::Configure},
    {"START", CommandId::Start},
    {"STOP", CommandId::Stop},
    {"READ_N_BYTES", CommandId::ReadNBytes},
    {"RESET", CommandId::Reset},
    {"STATUS", CommandId::Status},
};

constexpr std::size_t COMMAND_COUNT =
    sizeof(commandTable) / sizeof(commandTable[0]);

} // anonymous namespace

ParsedCommand parseCommand(std::string_view line) noexcept {
  ParsedCommand out{};
  out.id = CommandId::None;
  out.argCount = 0u;

  std::string_view rest = platform::str::trim(line);
  if (rest.empty())
    return out;

  const std::string_view name = platform::str::nextToken(rest);
  out.name = name;
  if (name.empty())
    return out;

  for (std::size_t i = 0; i < COMMAND_MAX_ARGS; ++i) {
    const std::string_view arg = platform::str::nextToken(rest);
    if (arg.empty())
      break;

    out.args[i] = arg;
    out.argCount = i + 1u;
  }

  for (std::size_t i = 0; i < COMMAND_COUNT; ++i) {
    if (platform::str::iequals(name, commandTable[i].name)) {
      out.id = commandTable[i].id;
      break;
    }
  }

  return out;
}

} // namespace usb
