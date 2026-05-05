/**
 * @file session/SessionManager.hpp
 * @brief Deterministic finite automaton for managing session state.
 *
 * @details
 * - Thread safety: Thread safe since all public functions lock internally.
 * - ISR safety: Not ISR safe due to rtos/Mutex.
 */

#ifndef SESSION_SESSIONMANAGER_HPP
#define SESSION_SESSIONMANAGER_HPP

#include "platform/Platform.hpp"
#include "rtos/Mutex.h"
#include <cstddef>
#include <cstdint>

namespace session {

/**
 * @enum SessionState
 * @brief Production session lifecycle: Idle / Active.
 */
enum class SessionState : uint8_t {
  Idle,
  Active,
};

/**
 * @struct SessionMetrics
 * @brief Pedometer metrics for a given session.
 */
struct SessionMetrics {
  SessionState state{SessionState::Idle};
  uint32_t stepCount{0};
  platform::TickUs startTimestampUs{0};
  platform::TickUs endTimestampUs{0};
};

/**
 * @struct SessionReport
 * @brief Pedometer report for a given session.
 */
struct SessionReport {
  uint32_t steps{0};
  uint32_t durationMs{0};
  uint32_t calories{0};
  float distanceMeters{0.0f};
};

/**
 * @class SessionManager
 * @brief Session lifecycle and metrics.
 */
class SessionManager {
public:
  SessionManager() noexcept = default;

  /**
   * @brief Begin step counting session (Idle → Active). Fails if not Idle.
   * @param startTimestampUs Session start timestamp.
   * @return True if session started successfully, false otherwise.
   */
  [[nodiscard]] bool startSession(platform::TickUs startTimestampUs) noexcept;

  /**
   * @brief End step counting session (Active → Idle).
   * @param stopTimestampUs Session stop timestamp.
   * @return True if session stopped successfully, false otherwise.
   * @details
   * - The state is returned to idle, but the metrics are still available.
   */
  [[nodiscard]] bool stopSession(platform::TickUs stopTimestampUs) noexcept;

  /**
   * @brief Accept a step if the session is active and the peak time is not
   * before the session start.
   * @param stepTimeUs Step peak timestamp (microseconds).
   * @return True if the step was counted, false otherwise.
   */
  [[nodiscard]] bool acceptStep(platform::TickUs stepTimeUs) noexcept;

  /**
   * @brief Get the step count.
   * @return Step count.
   */
  [[nodiscard]] uint32_t getStepCount() const noexcept;

  /**
   * @brief Check if the session is active.
   * @return True if the session is active, false otherwise.
   */
  [[nodiscard]] bool isActive() const noexcept;

  /**
   * @brief Get the session report.
   * @return Session report.
   */
  [[nodiscard]] SessionReport getReport() const noexcept;

  /**
   * @brief Format the session report.
   * @param buf Buffer to format the report into.
   * @param size Size of the buffer.
   * @return The number of characters written to the buffer.
   */
  [[nodiscard]] std::size_t formatReport(char *buf,
                                         std::size_t size) const noexcept;

  /**
   * @brief Restore metrics from RTC backup snapshot after MCU reset.
   * @param recording True if snapshot had an active session.
   * @param stepCount Step count from snapshot.
   * @param startTimestampUs Session start timestamp from snapshot.
   * @param endTimestampUs Session end timestamp from snapshot (0 if active).
   * @return False if snapshot fields were inconsistent (manager left idle).
   */
  [[nodiscard]] bool
  restoreFromRtcSnapshot(bool recording, std::uint32_t stepCount,
                         platform::TickUs startTimestampUs,
                         platform::TickUs endTimestampUs) noexcept;

  /**
   * @brief Full reset: idle, zero steps, cleared timestamps (RESET command).
   */
  void resetToIdle() noexcept;

  /**
   * @brief Read fields persisted to RTC under one lock.
   * @param outActive Output active flag.
   * @param outStepCount Output step count.
   * @param outStartUs Output session start timestamp (us).
   * @param outEndUs Output session end when idle; 0 when active.
   */
  void getRtcPersistSnapshot(bool *outActive, std::uint32_t *outStepCount,
                             platform::TickUs *outStartUs,
                             platform::TickUs *outEndUs) const noexcept;

private:
  mutable rtos::Mutex _mutex{};
  SessionMetrics _metrics{};
};

} // namespace session

#endif /* SESSION_SESSIONMANAGER_HPP */
