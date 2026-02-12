/**
 * @file session/SessionManager.cpp
 * @brief Session state and step aggregation implementation.
 */

#include "session/SessionManager.hpp"

namespace session {

void SessionManager::start() noexcept {}

void SessionManager::pause() noexcept {}

void SessionManager::resume() noexcept {}

void SessionManager::stop() noexcept {}

void SessionManager::onStep(const step_detection::StepEvent &event) noexcept {}

void SessionManager::getSnapshot(SessionSnapshot &out) const noexcept {}

uint32_t SessionManager::getStepCount() const noexcept { return 0; }

SessionState SessionManager::getState() const noexcept {
  return SessionState::Idle;
}

} // namespace session
