/** \file app/Application.cpp
 *  Thread creation, queue wiring, and main loop or thread entry points.
 */

#include "app/Application.hpp"
#include "platform/Platform.hpp"

namespace app {

void run()
{
    platform::init();
}

} // namespace app
