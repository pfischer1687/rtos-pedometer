/**
 * @file entry/TestEntry.cpp
 * @brief Minimal test implementation of firmware entry (infinite loop only).
 */

#include "entry/FirmwareEntry.hpp"
#include "mbed.h"

namespace entry {

[[noreturn]] int firmware_entry() {
  while (true) {
    __NOP();
  }
}

} // namespace entry
