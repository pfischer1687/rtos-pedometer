/**
 * @file main.cpp
 * @brief Entry point for the motion monitoring system.
 *
 * @details
 * Initializes the application and starts the RTOS scheduler.
 * Does not return.
 */

#include "app/Application.hpp"
#include "mbed.h"

[[noreturn]] int main() {
  app::run();

  while (true) {
    __NOP();
  }
}
