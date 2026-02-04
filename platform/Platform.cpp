/** \file platform/Platform.cpp
 *  Implementation of HAL abstractions, timing, and watchdog.
 */

#include "platform/Platform.hpp"

namespace platform {

TickUs TickSource::nowUs()
{
    return 0;
}

Result Watchdog::start(uint32_t timeoutMs)
{
    return Result::Ok;
}

Result Watchdog::stop()
{
    return Result::Ok;
}

void Watchdog::kick()
{
}

Result init()
{
    return Result::Ok;
}

} // namespace platform
