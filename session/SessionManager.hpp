/**
 * @file session/SessionManager.hpp
 * @brief Session lifecycle, mode (normal vs tuning), and production metrics.
 *
 * Invariants (enforced by implementation):
 * - Valid pairs: (Idle, Normal), (Idle, Tuning), (Active, Normal).
 * - Invalid: (Active, Tuning). startTuning/stopTuning apply only when
 *   SessionState == Idle.
 * - Active sessions exist only in Normal mode; tuning is Idle-only and must not
 *   alter production SessionMetrics.
 * - Session timing uses timestamps and step event times only.
 *
 * Thread safety: _metrics, _sessionMode, and _tuning are guarded by _mutex.
 * Do not call these APIs from ISR context (rtos::Mutex is not ISR-safe).
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
 * @brief Session lifecycle: Idle / Active.
 */
enum class SessionState : uint8_t {
  Idle,
  Active,
};

/**
 * @enum SessionMode
 * @brief Normal = production aggregation; Tuning = Idle-only diagnostics.
 */
enum class SessionMode : uint8_t {
  Normal,
  Tuning,
};

/**
 * @struct SessionMetrics
 * @brief Authoritative production session fields only (no derived distance /
 * calories).
 */
struct SessionMetrics {
  SessionState state{SessionState::Idle};
  uint32_t stepCount{0};
  platform::TickUs startTimestampUs{0};
  platform::TickUs endTimestampUs{0};
  platform::TickUs lastStepTimestampUs{0};
  uint32_t durationMs{0};
};

/**
 * @struct TuningSnapshot
 * @brief Idle-only diagnostic bucket; isolated from production SessionMetrics.
 * - acceptedStepCount: app-layer accepted steps (e.g. confidence pass).
 * - rejectedStepCount: app-layer rejections only (e.g. confidence fail).
 * - detectorStats: copied StepDetector funnel stats (signal-layer rejects).
 */
struct TuningSnapshot {
  uint32_t acceptedStepCount{0};
  uint32_t rejectedStepCount{0};
  step_detection::StepDetectorDebugStats detectorStats{};
};

/**
 * @struct SessionSnapshot
 * @brief Legacy snapshot; derived fields filled by getSnapshot() (external
 * policy for distance/calories).
 */
struct SessionSnapshot {
  SessionState state{SessionState::Idle};
  uint32_t stepCount{0};
  platform::TickUs sessionStartTimeUs{0};
  uint32_t totalDurationMs{0};
  platform::TickUs endTimestampUs{0};
  platform::TickUs lastStepTimestampUs{0};
};

struct SessionReport {
  uint32_t steps{0};
  uint32_t durationMs{0};
  uint32_t calories{0};
  float distanceMeters{0.0f};
};

/**
 * @class SessionManager
 * @brief Policy + aggregation: production metrics vs isolated tuning state.
 */
class SessionManager {
public:
  SessionManager() noexcept = default;

  /**
   * @brief Begin production session (Normal mode, Active). Fails if not
   * eligible (e.g. Idle + Tuning or non-Idle).
   */
  [[nodiscard]] bool startSession(platform::TickUs startTimestampUs) noexcept;

  /**
   * @brief Same as startSession using platform::getTimeUs().
   */
  [[nodiscard]] bool startSession() noexcept;

  /**
   * @brief End production session and return to Idle.
   */
  [[nodiscard]] bool stopSession(platform::TickUs stopTimestampUs) noexcept;

  /**
   * @brief Stop using last relevant session timestamp.
   */
  [[nodiscard]] bool stopSession() noexcept;

  /**
   * @brief Enter tuning mode. Only when SessionState == Idle.
   */
  [[nodiscard]] bool startTuning(platform::TickUs timestampUs) noexcept;

  /**
   * @brief Leave tuning mode. Only when SessionState == Idle.
   */
  [[nodiscard]] bool stopTuning(platform::TickUs timestampUs) noexcept;

  /**
   * @brief Step notification: Normal + Active updates production metrics;
   * Tuning updates _tuning only (confidence policy applied in implementation).
   */
  void onStep(const step_detection::StepEvent &event) noexcept;

  /**
   * @brief Update detector funnel stats during tuning (Idle + Tuning only).
   */
  void updateDetectorStats(
      const step_detection::StepDetectorDebugStats &stats) noexcept;

  [[nodiscard]] SessionMetrics getMetrics() const noexcept;

  [[nodiscard]] TuningSnapshot getTuningSnapshot() const noexcept;

  void start() noexcept;

  void stop() noexcept;

  void getSnapshot(SessionSnapshot &out) const noexcept;

  [[nodiscard]] uint32_t getStepCount() const noexcept;

  [[nodiscard]] SessionState getState() const noexcept;

  [[nodiscard]] bool isActive() const noexcept;

  [[nodiscard]] SessionReport getReport() const noexcept;

  [[nodiscard]] std::size_t formatReport(char *buf,
                                         std::size_t size) const noexcept;

  [[nodiscard]] SessionMode getSessionMode() const noexcept;

private:
  static constexpr platform::TickUs US_PER_MS = 1000u;

  void updateDurationLocked(platform::TickUs referenceTimestampUs) noexcept;

  mutable rtos::Mutex _mutex{};
  SessionMode _sessionMode{SessionMode::Normal};
  SessionMetrics _metrics{};
  platform::TickUs _lastTuningTransitionUs{0};
  TuningSnapshot _tuning{};
};

} // namespace session

#endif /* SESSION_SESSIONMANAGER_HPP */
