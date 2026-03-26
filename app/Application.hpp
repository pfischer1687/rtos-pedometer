/**
 * @file app/Application.hpp
 * @brief RTOS threads, IPC payloads, and system wiring.
 */

#ifndef APP_APPLICATION_HPP
#define APP_APPLICATION_HPP

#include <cstdint>

namespace app {

/**
 * @brief Raw IMU data frame as message with sequence.
 */
struct RawImuDataFrame {
  uint32_t sequence{0};
  uint32_t timestampUs{0};
  int16_t accelX{0};
  int16_t accelY{0};
  int16_t accelZ{0};
};

/**
 * @brief Processed IMU data frame.
 */
struct ProcessedImuDataFrame {
  uint32_t sequence{0};
  uint32_t sourceTimestampUs{0};
  int32_t accelMagnitudeMilliG{0};
};

/**
 * @brief Step detection event.
 */
struct StepDetectionEvent {
  uint32_t sequence{0};
  uint32_t peakTimeUs{0};
  uint8_t confidence{0};
};

/**
 * @brief Session / state-machine notification.
 */
struct SessionNotification {
  uint32_t sequence{0};
  uint8_t state{0};
  uint32_t stepCount{0};
};

/**
 * @brief Command captured by the USB command thread.
 */
struct UsbCommand {
  char line[64]{};
};

/**
 * @brief Application entry point.
 */
[[noreturn]] void run() noexcept;
} // namespace app

#endif // APP_APPLICATION_HPP
