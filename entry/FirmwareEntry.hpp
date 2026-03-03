/**
 * @file entry/FirmwareEntry.hpp
 * @brief Neutral interface for firmware entry (production or test).
 */

#ifndef ENTRY_FIRMWARE_ENTRY_HPP
#define ENTRY_FIRMWARE_ENTRY_HPP

namespace entry {
/**
 * @brief Entry point for the firmware.
 *
 * @details
 * This function is implemented in the production or test entry point file.
 */
[[noreturn]] int firmware_entry();
} // namespace entry

#endif // ENTRY_FIRMWARE_ENTRY_HPP
