/**
 * @file rtc/RtcSessionStore.hpp
 * @brief STM32 backup-domain snapshot for session persistence (BKP layout).
 */

#ifndef RTC_RTCSESSIONSTORE_HPP
#define RTC_RTCSESSIONSTORE_HPP

#include "platform/Platform.hpp"
#include <cstdint>

namespace rtc {

/**
 * @brief Snapshot of the session state.
 */
struct Snapshot {
  bool active;
  std::uint32_t steps;
  platform::TickUs startUs;
  platform::TickUs endUs;
};

/**
 * @brief Load the snapshot from the RTC backup registers.
 * @param out Snapshot.
 * @return True if the snapshot was loaded successfully, false otherwise.
 */
[[nodiscard]] bool load(Snapshot &out) noexcept;

/**
 * @brief Store the snapshot to the RTC backup registers.
 * @param s Snapshot.
 */
void store(const Snapshot &s) noexcept;

/**
 * @brief Invalidate the snapshot by clearing all RTC backup registers.
 */
void invalidate() noexcept;

} // namespace rtc

#endif // RTC_RTCSESSIONSTORE_HPP
