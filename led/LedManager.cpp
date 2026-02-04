/** \file led/LedManager.cpp
 *  LED pattern and tick implementation.
 */

#include "led/LedManager.hpp"

namespace led {

platform::Result LedManager::init()
{
    return platform::Result::Ok;
}

void LedManager::setPattern(LedPattern pattern)
{
}

void LedManager::tick()
{
}

} // namespace led
