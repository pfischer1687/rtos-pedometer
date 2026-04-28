/**
 * @file session/SessionManager.cpp
 * @brief Session state and step aggregation implementation.
 */

#include "session/SessionManager.hpp"
#include <cstdio>

namespace session {

namespace {

static constexpr platform::TickUs US_PER_MS = 1000u;
static constexpr float CALORIES_PER_STEP = 0.04f;
static constexpr float METERS_PER_STEP = 0.762f;

/**
 * @brief Check if the session is active.
 * @param s Session state.
 * @return True if the session is active, false otherwise.
 */
[[nodiscard]] constexpr bool isActive(SessionState s) noexcept {
  return s == SessionState::Active;
}

/**
 * @brief Check if the session can be stopped.
 * @param state Session state.
 * @return True if the session can be stopped, false otherwise.
 */
[[nodiscard]] constexpr bool canStopSession(SessionState state) noexcept {
  return isActive(state);
}

} // anonymous namespace

void SessionManager::resetMetricsForNewSession(
    platform::TickUs startTimestampUs) noexcept {
  _metrics.state = SessionState::Active;
  _metrics.stepCount = 0u;
  _metrics.startTimestampUs = startTimestampUs;
  _metrics.endTimestampUs = 0u;
  _debug.stepsReceived = 0u;
  _debug.acceptedStepCount = 0u;
  _debug.stepsDropped = 0u;
  _haveLastAcceptedStep = false;
  _lastAcceptedStepTimeUs = 0u;
}

bool SessionManager::startSession(platform::TickUs startTimestampUs) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (_metrics.state == SessionState::Active) {
    return true;
  }
  resetMetricsForNewSession(startTimestampUs);
  ++_debug.sessionResets;
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

// Only application path: Application::handleStepEventsUpTo (step mail drain).
void SessionManager::onStep(const step_detection::StepEvent &event) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  ++_debug.stepsReceived;
  if (_metrics.state != SessionState::Active) {
    ++_debug.stepsDropped;
    return;
  }
  if (_haveLastAcceptedStep &&
      (event.timestampUs - _lastAcceptedStepTimeUs) < MIN_STEP_INTERVAL_US) {
    ++_debug.stepsDropped;
    return;
  }

  ++_metrics.stepCount;
  _debug.acceptedStepCount = _metrics.stepCount;
  _lastAcceptedStepTimeUs = event.timestampUs;
  _haveLastAcceptedStep = true;
}

void SessionManager::setStepDetectorDebugStats(
    const step_detection::StepDetectorDebugStats &stats) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  _debug.detectorStats = stats;
}

void SessionManager::resetDebugStats() noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  const uint32_t steps = _metrics.stepCount;
  _debug = SessionDebugStats{};
  _debug.stepsReceived = steps;
  _debug.acceptedStepCount = steps;
}

SessionDebugStats SessionManager::getDebugStats() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _debug;
}

uint32_t SessionManager::getStepCount() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _metrics.stepCount;
}

uint32_t SessionManager::getStepsReceived() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _debug.stepsReceived;
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

  const int n = std::snprintf(
      buf, size,
      "DEBUG acc=%lu sr=%lu sd=%lu srst=%lu "
      "peaks=%lu em=%lu r_np=%lu ib=%lu",
      static_cast<unsigned long>(d.acceptedStepCount),
      static_cast<unsigned long>(d.stepsReceived),
      static_cast<unsigned long>(d.stepsDropped),
      static_cast<unsigned long>(d.sessionResets),
      static_cast<unsigned long>(st.peaks),
      static_cast<unsigned long>(st.emitted),
      static_cast<unsigned long>(st.rejectNoPeak),
      static_cast<unsigned long>(st.intervalBlocked));
  if (n <= 0 || static_cast<std::size_t>(n) >= size) {
    return 0u;
  }

  return static_cast<std::size_t>(n);
}

} // namespace session
