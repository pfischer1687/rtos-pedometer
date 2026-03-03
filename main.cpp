/**
 * @file main.cpp
 * @brief Entry point for the motion monitoring system.
 *
 * @details
 * Initializes the application and starts the RTOS scheduler.
 * Does not return.
 */

#include "entry/FirmwareEntry.hpp"

[[noreturn]] int main() { entry::firmware_entry(); }
