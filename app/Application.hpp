/** \file app/Application.hpp
 *  RTOS threads, queues, and system wiring. Owns thread creation and module
 * coordination. Ownership: app owns all threads and inter-thread communication;
 * no hardware or algorithm logic.
 */

#ifndef APP_APPLICATION_HPP
#define APP_APPLICATION_HPP

namespace app {

/** Application entry: initialize platform and modules, create threads, start
 * scheduler. Does not return (runs until power off).
 */
void run();

} // namespace app

#endif /* APP_APPLICATION_HPP */
