/**
 * @file app/Application.hpp
 * @brief RTOS threads, IPC payloads, and system wiring.
 * @details
 * Owns thread creation and module coordination. The app layer owns threads and
 * inter-thread communication; hardware drivers call
 * @ref signalSensorAcquisitionFromIsr from the IMU data-ready ISR (or a thin
 * platform wrapper) using event flags only — no queues from ISR.
 */

#ifndef APP_APPLICATION_HPP
#define APP_APPLICATION_HPP

#include <cstdint>

namespace app {

/**
 * @brief Raw sensor frame handed off from acquisition to signal processing.
 */
struct SensorFrameMsg {
  uint32_t sequence{0};
  uint32_t timestampUs{0};
  int16_t accelX{0};
  int16_t accelY{0};
  int16_t accelZ{0};
};

/**
 * @brief Placeholder output of magnitude / filter pipeline.
 */
struct ProcessedSignalMsg {
  uint32_t sequence{0};
  uint32_t sourceTimestampUs{0};
  int32_t magnitudeMilliPlaceholder{0};
};

/**
 * @brief Placeholder step-detection event.
 */
struct StepEventMsg {
  uint32_t sequence{0};
  uint32_t peakTimeUs{0};
  uint8_t confidencePlaceholder{0};
};

/**
 * @brief Placeholder session / state-machine notification.
 */
struct SessionNoticeMsg {
  uint32_t sequence{0};
  uint8_t statePlaceholder{0};
  uint32_t stepCountPlaceholder{0};
};

/**
 * @brief Line captured by the USB command thread (fixed size, no heap).
 */
struct UsbCommandMsg {
  char line[64]{};
};

/**
 * @brief ISR-safe: signal the sensor acquisition thread (event flag only).
 * @details Call from IMU data-ready ISR after driver bookkeeping, or from a
 * test harness that simulates an ISR edge.
 */
void signalSensorAcquisitionFromIsr() noexcept;

/**
 * @brief Application entry point.
 * @details
 * Creates RTOS objects, starts worker threads, then blocks forever. Does not
 * return.
 */
[[noreturn]] void run() noexcept;

} // namespace app

#endif /* APP_APPLICATION_HPP */
