/**
 * @file entry/ProductionEntry.cpp
 * @brief Entry point for the firmware.
 */

#include "app/Application.hpp"
#include "entry/FirmwareEntry.hpp"
#include "mbed.h"

namespace entry {

[[noreturn]] int firmware_entry() {
  app::run();

  while (true) {
    __NOP();
  }
}

} // namespace entry
