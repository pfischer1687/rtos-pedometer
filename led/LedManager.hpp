/** \file led/LedManager.hpp
 *  LED indication (e.g. heartbeat, session active, step blink). Non-critical path.
 *  Ownership: owns LED pins and blink state; driven by app or session events.
 */

#ifndef LED_LEDMANAGER_HPP
#define LED_LEDMANAGER_HPP

#include "platform/Platform.hpp"

namespace led {

/** LED pattern or state. */
enum class LedPattern : uint8_t {
    Off,
    On,
    Heartbeat,
    SessionActive,
    StepBlink,
};

/** LED manager: set pattern, tick for heartbeat/blink (call from low-priority thread). */
class LedManager {
public:
    LedManager() = default;

    /** Initialize GPIO for LED(s). */
    platform::Result init();

    /** Set current pattern. */
    void setPattern(LedPattern pattern);

    /** Call periodically (e.g. 10 ms) to update blink state. */
    void tick();
};

} // namespace led

#endif /* LED_LEDMANAGER_HPP */
