/**
 * @file session/SessionManager.cpp
 * @brief Session state and step aggregation implementation.
 */

#include "session/SessionManager.hpp"
#include <cstdio>

namespace session {

namespace {

static constexpr platform::TickUs US_PER_MS = 1000u;
static constexpr float CONFIDENCE_THRESHOLD = 0.5f;
static constexpr float CALORIES_PER_STEP = 0.04f;
static constexpr float METERS_PER_STEP = 0.762f;

/**
 * @brief Check if the session is idle.
 * @param s Session state.
 * @return True if the session is idle, false otherwise.
 */
[[nodiscard]] constexpr bool isIdle(SessionState s) noexcept {
  return s == SessionState::Idle;
}

/**
 * @brief Check if the session is active.
 * @param s Session state.
 * @return True if the session is active, false otherwise.
 */
[[nodiscard]] constexpr bool isActive(SessionState s) noexcept {
  return s == SessionState::Active;
}

/**
 * @brief Check if the session can be started.
 * @param state Session state.
 * @return True if the session can be started, false otherwise.
 */
[[nodiscard]] constexpr bool canStartSession(SessionState state) noexcept {
  return isIdle(state);
}

/**
 * @brief Check if the session can be stopped.
 * @param state Session state.
 * @return True if the session can be stopped, false otherwise.
 */
[[nodiscard]] constexpr bool canStopSession(SessionState state) noexcept {
  return isActive(state);
}

/**
 * @brief Reset the session metrics for a new session and set the state to
 * active.
 * @param m Session metrics.
 * @param startUs Session start timestamp.
 */
void resetMetricsForNewSession(SessionMetrics &m,
                               platform::TickUs startUs) noexcept {
  m.state = SessionState::Active;
  m.stepCount = 0u;
  m.startTimestampUs = startUs;
  m.endTimestampUs = 0u;
}

} // anonymous namespace

bool SessionManager::startSession(platform::TickUs startTimestampUs) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (!canStartSession(_metrics.state)) {
    return false;
  }

  resetMetricsForNewSession(_metrics, startTimestampUs);
  return true;
}

bool SessionManager::stopSession(platform::TickUs stopTimestampUs) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (!canStopSession(_metrics.state)) {
    return false;
  }

  _metrics.endTimestampUs = stopTimestampUs;
  _metrics.state = SessionState::Idle;
  return true;
}

void SessionManager::onStep(const step_detection::StepEvent &event) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (_metrics.state != SessionState::Active) {
    return;
  }

  const bool accepted = event.confidence >= CONFIDENCE_THRESHOLD;

  if (accepted) {
    ++_debug.acceptedStepCount;
    ++_metrics.stepCount;
  } else {
    ++_debug.rejectedStepCount;
  }
}

void SessionManager::setStepDetectorDebugStats(
    const step_detection::StepDetectorDebugStats &stats) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  _debug.detectorStats = stats;
}

void SessionManager::resetDebugStats() noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  _debug = SessionDebugStats{};
}

SessionDebugStats SessionManager::getDebugStats() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _debug;
}

uint32_t SessionManager::getStepCount() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _metrics.stepCount;
}

bool SessionManager::isActive() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _metrics.state == SessionState::Active;
}

SessionReport SessionManager::getReport() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  SessionReport report{};
  report.steps = _metrics.stepCount;

  bool noSessionsHaveBeenStarted =
      _metrics.startTimestampUs == 0u ||
      (_metrics.state == SessionState::Idle &&
       _metrics.endTimestampUs <= _metrics.startTimestampUs);
  if (noSessionsHaveBeenStarted) {
    report.durationMs = 0u;
    report.calories = 0u;
    report.distanceMeters = 0.0f;
    return report;
  }

  platform::TickUs endUs = _metrics.state == SessionState::Active
                               ? platform::getTimeUs()
                               : _metrics.endTimestampUs;
  const platform::TickUs elapsedUs =
      platform::elapsed(_metrics.startTimestampUs, endUs);

  report.durationMs = static_cast<uint32_t>(elapsedUs / US_PER_MS);
  report.calories = static_cast<uint32_t>(
      static_cast<float>(_metrics.stepCount) * CALORIES_PER_STEP);
  report.distanceMeters =
      static_cast<float>(_metrics.stepCount) * METERS_PER_STEP;

  return report;
}

std::size_t SessionManager::formatReport(char *buf,
                                         std::size_t size) const noexcept {
  if (buf == nullptr || size == 0u) {
    return 0u;
  }

  const SessionReport report = getReport();

  const int n = std::snprintf(
      buf, size, "SESSION steps=%lu duration_ms=%lu kcal=%lu dist_m=%.2f",
      static_cast<unsigned long>(report.steps),
      static_cast<unsigned long>(report.durationMs),
      static_cast<unsigned long>(report.calories), report.distanceMeters);
  if (n <= 0 || static_cast<std::size_t>(n) >= size) {
    return 0u;
  }

  return static_cast<std::size_t>(n);
}

std::size_t SessionManager::formatDebugReport(char *buf,
                                              std::size_t size) const noexcept {
  if (buf == nullptr || size == 0u) {
    return 0u;
  }

  const SessionDebugStats d = getDebugStats();
  const step_detection::StepDetectorDebugStats &st = d.detectorStats;

  const int n =
      std::snprintf(buf, size,
                    "DEBUG accepted=%lu rejected=%lu peaks=%lu emitted=%lu "
                    "r_np=%lu r_slope=%lu r_valley=%lu r_base=%lu r_int=%lu",
                    static_cast<unsigned long>(d.acceptedStepCount),
                    static_cast<unsigned long>(d.rejectedStepCount),
                    static_cast<unsigned long>(st.peaks),
                    static_cast<unsigned long>(st.emitted),
                    static_cast<unsigned long>(st.rejectNoPeak),
                    static_cast<unsigned long>(st.rejectSlope),
                    static_cast<unsigned long>(st.rejectValley),
                    static_cast<unsigned long>(st.rejectBaseline),
                    static_cast<unsigned long>(st.rejectInterval));
  if (n <= 0 || static_cast<std::size_t>(n) >= size) {
    return 0u;
  }

  return static_cast<std::size_t>(n);
}

} // namespace session
