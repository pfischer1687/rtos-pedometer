/**
 * @file: app/Application.cpp
 * @brief: Thread creation, queue wiring, and main loop or thread entry points.
 */

#include "app/Application.hpp"
#include "platform/Platform.hpp"

namespace app {

[[noreturn]] void run() noexcept {
  auto &timer = platform::timer();

  while (true) {
    platform::TickUs t = timer.nowUs();
    (void)t;
  }
}

} // namespace app
