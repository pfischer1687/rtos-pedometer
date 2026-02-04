/** \file session/SessionManager.hpp
 *  Session state, step count aggregation, and persistence. Application logic only.
 *  Ownership: owns session state; consumes step events; exposes count and state to USB/app.
 */

#ifndef SESSION_SESSIONMANAGER_HPP
#define SESSION_SESSIONMANAGER_HPP

#include "step_detection/StepDetector.hpp"
#include <cstdint>

namespace session {

/** Session state for the motion monitoring system. */
enum class SessionState : uint8_t {
    Idle,
    Active,
    Paused,
    Stopped,
};

/** Snapshot of current session for inspection (USB, logging). */
struct SessionSnapshot {
    SessionState state;
    uint32_t stepCount;
    uint32_t sessionStartTimeUs;
    uint32_t totalDurationMs;
};

/** Session manager: state machine, step count, optional persistence. */
class SessionManager {
public:
    SessionManager() = default;

    /** Start a new session (reset step count, set state Active). */
    void start();

    /** Pause session (retain count). */
    void pause();

    /** Resume from pause. */
    void resume();

    /** Stop session (finalize, optionally persist). */
    void stop();

    /** Notify one step (called from step detection pipeline). */
    void onStep(const step_detection::StepEvent& event);

    /** Get current snapshot (thread-safe via queue or copy). */
    void getSnapshot(SessionSnapshot& out) const;

    /** Return current step count. */
    uint32_t getStepCount() const;

    /** Return current state. */
    SessionState getState() const;
};

} // namespace session

#endif /* SESSION_SESSIONMANAGER_HPP */
