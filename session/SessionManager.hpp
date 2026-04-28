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
#include "step_detection/StepDetector.hpp"
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
 * @struct SessionDebugStats
 * @brief Metrics collected for debugging step detection configuration during
 * tuning sessions.
 */
struct SessionDebugStats {
  uint32_t acceptedStepCount{0};
  uint32_t rejectedStepCount{0};
  step_detection::StepDetectorDebugStats detectorStats{};
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
   * @brief Handler for step detection events.
   * @param event Step detection event.
   */
  void onStep(const step_detection::StepEvent &event) noexcept;

  /**
   * @brief Set the step detector debug stats.
   * @param stats Step detector debug stats.
   */
  void setStepDetectorDebugStats(
      const step_detection::StepDetectorDebugStats &stats) noexcept;

  /**
   * @brief Clear debug counters and stored detector snapshot.
   */
  void resetDebugStats() noexcept;

  /**
   * @brief Get the session debug stats.
   * @return Session debug stats.
   */
  [[nodiscard]] SessionDebugStats getDebugStats() const noexcept;

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
   * @brief Format the session debug report.
   * @param buf Buffer to format the debug report into.
   * @param size Size of the buffer.
   * @return The number of characters written to the buffer.
   */
  [[nodiscard]] std::size_t formatDebugReport(char *buf,
                                              std::size_t size) const noexcept;

private:
  mutable rtos::Mutex _mutex{};
  SessionMetrics _metrics{};
  SessionDebugStats _debug{};
};

} // namespace session

#endif /* SESSION_SESSIONMANAGER_HPP */
