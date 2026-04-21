/**
 * @file session/SessionManager.cpp
 * @brief Session state, mode, and step aggregation implementation.
 */

#include "session/SessionManager.hpp"

#include <cstdio>

namespace session {

namespace {

// MUST stay consistent with Application STEP_CONF_THRESHOLD
constexpr float kConfidenceThreshold = 128.0f / 255.0f;
constexpr float kCaloriesPerStep = 0.04f;
constexpr float kMetersPerStep = 0.762f;

[[nodiscard]] constexpr bool isIdle(SessionState s) noexcept {
  return s == SessionState::Idle;
}

[[nodiscard]] constexpr bool isActive(SessionState s) noexcept {
  return s == SessionState::Active;
}

[[nodiscard]] constexpr bool canStartSession(SessionState state,
                                             SessionMode mode) noexcept {
  return isIdle(state) && mode == SessionMode::Normal;
}

[[nodiscard]] constexpr bool canStopSession(SessionState state,
                                            SessionMode mode) noexcept {
  return isActive(state) && mode == SessionMode::Normal;
}

[[nodiscard]] constexpr bool canStartTuning(SessionState state,
                                            SessionMode mode) noexcept {
  return isIdle(state) && mode == SessionMode::Normal;
}

[[nodiscard]] constexpr bool canStopTuning(SessionState state,
                                           SessionMode mode) noexcept {
  return isIdle(state) && mode == SessionMode::Tuning;
}

void resetForNewSession(SessionMetrics &m, platform::TickUs startUs) noexcept {
  m.state = SessionState::Active;
  m.stepCount = 0u;
  m.startTimestampUs = startUs;
  m.endTimestampUs = 0u;
  m.lastStepTimestampUs = startUs;
  m.durationMs = 0u;
}

} // namespace

void SessionManager::updateDurationLocked(
    platform::TickUs referenceTimestampUs) noexcept {
  if (referenceTimestampUs <= _metrics.startTimestampUs) {
    _metrics.durationMs = 0u;
    return;
  }

  const platform::TickUs elapsedUs =
      platform::elapsed(_metrics.startTimestampUs, referenceTimestampUs);
  _metrics.durationMs = elapsedUs / US_PER_MS;
}

bool SessionManager::startSession(platform::TickUs startTimestampUs) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (!canStartSession(_metrics.state, _sessionMode)) {
    return false;
  }

  resetForNewSession(_metrics, startTimestampUs);
  return true;
}

bool SessionManager::startSession() noexcept {
  return startSession(platform::getTimeUs());
}

bool SessionManager::stopSession(platform::TickUs stopTimestampUs) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (!canStopSession(_metrics.state, _sessionMode)) {
    return false;
  }

  _metrics.endTimestampUs = stopTimestampUs;
  updateDurationLocked(stopTimestampUs);
  // Keep finalized metrics retrievable after stop; return lifecycle to Idle.
  _metrics.state = SessionState::Idle;
  return true;
}

bool SessionManager::stopSession() noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (!canStopSession(_metrics.state, _sessionMode)) {
    return false;
  }

  const platform::TickUs stopTimestampUs = (_metrics.lastStepTimestampUs != 0u)
                                               ? _metrics.lastStepTimestampUs
                                               : _metrics.startTimestampUs;
  _metrics.endTimestampUs = stopTimestampUs;
  updateDurationLocked(stopTimestampUs);
  // Keep finalized metrics retrievable after stop; return lifecycle to Idle.
  _metrics.state = SessionState::Idle;
  return true;
}

bool SessionManager::startTuning(platform::TickUs timestampUs) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (!canStartTuning(_metrics.state, _sessionMode)) {
    return false;
  }

  _sessionMode = SessionMode::Tuning;
  _tuning = TuningSnapshot{};
  _lastTuningTransitionUs = timestampUs;
  return true;
}

bool SessionManager::stopTuning(platform::TickUs timestampUs) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (!canStopTuning(_metrics.state, _sessionMode)) {
    return false;
  }

  _sessionMode = SessionMode::Normal;
  _lastTuningTransitionUs = timestampUs;
  return true;
}

void SessionManager::onStep(const step_detection::StepEvent &event) noexcept {
  rtos::ScopedMutexLock lock(_mutex);

  if (_sessionMode == SessionMode::Tuning) {
    if (_metrics.state != SessionState::Idle)
      return;

    if (event.confidence >= kConfidenceThreshold) {
      if (_tuning.acceptedStepCount < UINT32_MAX) {
        ++_tuning.acceptedStepCount;
      }
    } else if (_tuning.rejectedStepCount < UINT32_MAX) {
      ++_tuning.rejectedStepCount;
    }
    return;
  }

  if (_sessionMode != SessionMode::Normal ||
      _metrics.state != SessionState::Active) {
    return;
  }

  if (_metrics.stepCount < UINT32_MAX) {
    ++_metrics.stepCount;
  }
  _metrics.lastStepTimestampUs = event.timestampUs;
  updateDurationLocked(event.timestampUs);
}

void SessionManager::updateDetectorStats(
    const step_detection::StepDetectorDebugStats &stats) noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (_sessionMode != SessionMode::Tuning ||
      _metrics.state != SessionState::Idle) {
    return;
  }
  _tuning.detectorStats = stats;
}

SessionMetrics SessionManager::getMetrics() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _metrics;
}

TuningSnapshot SessionManager::getTuningSnapshot() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  if (_sessionMode != SessionMode::Tuning ||
      _metrics.state != SessionState::Idle) {
    return TuningSnapshot{};
  }
  return _tuning;
}

void SessionManager::start() noexcept { (void)startSession(); }

void SessionManager::stop() noexcept { (void)stopSession(); }

void SessionManager::getSnapshot(SessionSnapshot &out) const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  out.state = _metrics.state;
  out.stepCount = _metrics.stepCount;
  out.sessionStartTimeUs = _metrics.startTimestampUs;
  out.totalDurationMs = _metrics.durationMs;
  out.endTimestampUs = _metrics.endTimestampUs;
  out.lastStepTimestampUs = _metrics.lastStepTimestampUs;
}

uint32_t SessionManager::getStepCount() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _metrics.stepCount;
}

SessionState SessionManager::getState() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _metrics.state;
}

bool SessionManager::isActive() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _metrics.state == SessionState::Active;
}

SessionReport SessionManager::getReport() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  SessionReport report{};
  report.steps = _metrics.stepCount;
  report.durationMs = _metrics.durationMs;
  report.calories = static_cast<uint32_t>(
      static_cast<float>(_metrics.stepCount) * kCaloriesPerStep);
  report.distanceMeters =
      static_cast<float>(_metrics.stepCount) * kMetersPerStep;
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

SessionMode SessionManager::getSessionMode() const noexcept {
  rtos::ScopedMutexLock lock(_mutex);
  return _sessionMode;
}

} // namespace session
