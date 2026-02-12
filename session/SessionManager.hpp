/**
 * @file session/SessionManager.hpp
 * @brief Session state, step count aggregation, and persistence
 */

#ifndef SESSION_SESSIONMANAGER_HPP
#define SESSION_SESSIONMANAGER_HPP

#include "step_detection/StepDetector.hpp"
#include <cstdint>

namespace session {

/**
 * @enum SessionState
 * @brief Session state for the motion monitoring system.
 */
enum class SessionState : uint8_t {
  Idle,
  Active,
  Paused,
  Stopped,
};

/**
 * @struct SessionSnapshot
 * @brief Snapshot of current session for inspection (USB, logging).
 */
struct SessionSnapshot {
  SessionState state{SessionState::Idle};
  uint32_t stepCount{0};
  uint32_t sessionStartTimeUs{0};
  uint32_t totalDurationMs{0};
};

/**
 * @class SessionManager
 * @brief Session manager: state machine, step count, optional persistence.
 */
class SessionManager {
public:
  /**
   * @brief Constructor.
   */
  SessionManager() noexcept = default;

  /**
   * @brief Start a new session (reset step count, set state Active).
   */
  void start() noexcept;

  /**
   * @brief Pause session (retain count).
   */
  void pause() noexcept;

  /**
   * @brief Resume from pause.
   */
  void resume() noexcept;

  /**
   * @brief Stop session (finalize, optionally persist).
   */
  void stop() noexcept;

  /**
   * @brief Notify one step (called from step detection pipeline).
   * @param event Step event.
   */
  void onStep(const step_detection::StepEvent &event) noexcept;

  /**
   * @brief Get current snapshot (thread-safe via queue or copy).
   * @param out Snapshot output.
   */
  void getSnapshot(SessionSnapshot &out) const noexcept;

  /**
   * @brief Return current step count.
   * @return Step count.
   */
  uint32_t getStepCount() const noexcept;

  /**
   * @brief Return current state.
   * @return Current state.
   */
  SessionState getState() const noexcept;
};

} // namespace session

#endif /* SESSION_SESSIONMANAGER_HPP */
