/**
 * @file app/Command.cpp
 * @brief Implementation of USB command parsing.
 */

#include "app/Command.hpp"
#include "platform/StringUtils.hpp"

namespace app {

namespace {

struct CommandEntry {
  std::string_view name;
  CommandId id;
};

constexpr CommandEntry commandTable[] = {
    {"START", CommandId::Start},
    {"STOP", CommandId::Stop},
    {"STATUS", CommandId::Status},
    {"DEBUG_STATUS", CommandId::DebugStatus},
    {"DEBUG_START", CommandId::DebugStart},
    {"RESET", CommandId::Reset},
};

constexpr std::size_t COMMAND_COUNT =
    sizeof(commandTable) / sizeof(commandTable[0]);

} // anonymous namespace

std::optional<CommandId> parseCommand(const usb::ParsedLine &line) noexcept {
  if (line.name.empty())
    return std::nullopt;

  for (std::size_t i = 0; i < COMMAND_COUNT; ++i) {
    if (platform::str::iequals(line.name, commandTable[i].name)) {
      return commandTable[i].id;
    }
  }
  return std::nullopt;
}

} // namespace app
