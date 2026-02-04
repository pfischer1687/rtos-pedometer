/** \file session/SessionManager.cpp
 *  Session state and step aggregation implementation.
 */

#include "session/SessionManager.hpp"

namespace session {

void SessionManager::start()
{
}

void SessionManager::pause()
{
}

void SessionManager::resume()
{
}

void SessionManager::stop()
{
}

void SessionManager::onStep(const step_detection::StepEvent& event)
{
}

void SessionManager::getSnapshot(SessionSnapshot& out) const
{
}

uint32_t SessionManager::getStepCount() const
{
    return 0;
}

SessionState SessionManager::getState() const
{
    return SessionState::Idle;
}

} // namespace session
