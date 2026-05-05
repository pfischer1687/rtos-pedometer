/**
 * @file rtc/RtcSessionStore.cpp
 * @brief Backup register snapshot load/store (STM32F7 RTC BKP0R–BKP6R).
 */

#include "rtc/RtcSessionStore.hpp"
#include "stm32f7xx.h"
#include <cstdint>

namespace rtc {

namespace {

// BKP2R + BKP6R must both match; BKP5R checksum covers BKP0..4 (active, steps,
// startUs, endUs).
constexpr std::uint32_t RTC_SESSION_MAGIC = 0x52545353u;  // 'RTSS'
constexpr std::uint32_t RTC_SESSION_MAGIC2 = 0xA53C9653u; // second marker

[[nodiscard]] constexpr std::uint32_t
rtcSnapshotChecksum(std::uint32_t active01, std::uint32_t steps,
                    std::uint32_t startUs, std::uint32_t endUs) noexcept {
  std::uint32_t c = RTC_SESSION_MAGIC ^ RTC_SESSION_MAGIC2;
  c ^= active01 * 0x9E3779B9u;
  c ^= steps * 0x85EBCA6Bu;
  c ^= startUs;
  c ^= endUs;
  c ^= (c << 7u) | (c >> 25u);
  return c;
}

[[nodiscard]] static bool
isValidSnapshot(std::uint32_t magic1, std::uint32_t magic2,
                std::uint32_t activeRaw, std::uint32_t steps,
                std::uint32_t startUs, std::uint32_t endUs,
                std::uint32_t checksumStored) noexcept {
  if (magic1 != RTC_SESSION_MAGIC || magic2 != RTC_SESSION_MAGIC2) {
    return false;
  }
  if (activeRaw > 1u) {
    return false;
  }
  return rtcSnapshotChecksum(activeRaw, steps, startUs, endUs) ==
         checksumStored;
}

} // namespace

bool load(Snapshot &out) noexcept {
  const std::uint32_t magic1 = RTC->BKP2R;
  const std::uint32_t magic2 = RTC->BKP6R;
  const std::uint32_t activeRaw = RTC->BKP0R;
  const std::uint32_t steps = RTC->BKP1R;
  const platform::TickUs startUs = RTC->BKP3R;
  const platform::TickUs endUs = RTC->BKP4R;
  const std::uint32_t checksumStored = RTC->BKP5R;

  if (!isValidSnapshot(magic1, magic2, activeRaw, steps, startUs, endUs,
                       checksumStored)) {
    return false;
  }

  out.active = (activeRaw == 1u);
  out.steps = steps;
  out.startUs = startUs;
  out.endUs = endUs;
  return true;
}

void store(const Snapshot &s) noexcept {
  const std::uint32_t activeRaw = s.active ? 1u : 0u;
  const std::uint32_t checksum =
      rtcSnapshotChecksum(activeRaw, s.steps, s.startUs, s.endUs);

  RTC->BKP0R = activeRaw;
  RTC->BKP1R = s.steps;
  RTC->BKP3R = s.startUs;
  RTC->BKP4R = s.endUs;
  RTC->BKP2R = RTC_SESSION_MAGIC;
  RTC->BKP6R = RTC_SESSION_MAGIC2;
  RTC->BKP5R = checksum;
}

void invalidate() noexcept {
  RTC->BKP0R = 0u;
  RTC->BKP1R = 0u;
  RTC->BKP2R = 0u;
  RTC->BKP3R = 0u;
  RTC->BKP4R = 0u;
  RTC->BKP5R = 0u;
  RTC->BKP6R = 0u;
}

} // namespace rtc
