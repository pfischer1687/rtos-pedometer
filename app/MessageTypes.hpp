/**
 * @file app/MessageTypes.hpp
 * @brief Production app-layer message types.
 */

#ifndef APP_MESSAGETYPES_HPP
#define APP_MESSAGETYPES_HPP

#include "imu/Mpu6050Driver.hpp"
#include "platform/Platform.hpp"
#include "usb/UsbInterface.hpp"

namespace app {

namespace message_types {

/**
 * @brief Raw IMU data frame as message with sequence.
 */
struct RawImuDataFrame {
  uint32_t sequence{0};
  imu::ImuSample sample{};
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
  uint8_t confidence{0};
};

/**
 * @brief Outbound text line for the USB thread (from session / workers).
 */
struct UsbResponse {
  char msg[usb::USB_RESPONSE_MSG_MAX_BYTES]{};
};

/**
 * @brief IMU event type.
 */
enum class ImuEventType : uint8_t {
  None = 0,
  MailAllocFail,
  ReadFail,
  HardwareFault,
  Timeout,
};

/**
 * @brief IMU event.
 */
struct ImuEvent {
  uint32_t timestampUs;
  ImuEventType type;
  platform::Result result;
};

/**
 * @brief IMU health snapshot.
 */
struct ImuHealthSnapshot {
  uint32_t totalSamples{0};
  uint32_t dropCount{0};
  uint32_t totalEvents{0};
  uint32_t bufferedEvents{0};
};

} // namespace message_types

} // namespace app

#endif // APP_MESSAGETYPES_HPP
