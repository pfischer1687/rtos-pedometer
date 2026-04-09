/**
 * @file app/Application.hpp
 * @brief RTOS threads, IPC payloads, and Application class wiring.
 */

#ifndef APP_APPLICATION_HPP
#define APP_APPLICATION_HPP

#include "rtos/EventFlags.h"
#include "rtos/Mail.h"
#include "rtos/Thread.h"

#include "imu/Mpu6050Driver.hpp"
#include "usb/Command.hpp"
#include "usb/UsbInterface.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

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
  usb::ParsedCommand parsed{};
  char inputText[usb::USB_CMD_MAX_LEN]{};
};

/**
 * @brief Outbound text line for the USB thread (from session / workers).
 */
struct UsbResponse {
  char msg[usb::USB_CMD_MAX_LEN];
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
  uint32_t eventCount{0};
};

/**
 * @class Application
 * @brief Owns RTOS threads, mail queues, event flags, and drop counters for the
 * motion pipeline.
 */
class Application {
public:
  /**
   * @brief Constructor.
   * @param imu IMU driver.
   */
  explicit Application(imu::Mpu6050Driver &imu) noexcept;

  ~Application() = default;
  Application(const Application &) = delete;
  Application &operator=(const Application &) = delete;
  Application(Application &&) = delete;
  Application &operator=(Application &&) = delete;

  /**
   * @brief Start worker threads and block forever.
   */
  [[noreturn]] void run() noexcept;

  /**
   * @brief ISR-safe: wake the IMU acquisition thread (event flags only).
   */
  void signalSensorAcquisitionFromIsr() noexcept;

  /**
   * @brief Get IMU health snapshot.
   * @return IMU health snapshot.
   */
  ImuHealthSnapshot getImuHealthSnapshot() const noexcept;

private:
  /**
   * @brief Configuration constants.
   */
  struct Config {
    static constexpr uint32_t MAIL_DEPTH = 32u;

    static constexpr uint32_t EVENT_IMU_DATA_READY = 1u << 0;
    static constexpr uint32_t EVENT_LED_UPDATE = 1u << 1;

    static constexpr std::size_t IMU_THREAD_STACK_DEPTH = 3072u;
    static constexpr std::size_t SIGNAL_THREAD_STACK_DEPTH = 3072u;
    static constexpr std::size_t STEP_THREAD_STACK_DEPTH = 2048u;
    static constexpr std::size_t SESSION_THREAD_STACK_DEPTH = 2048u;
    static constexpr std::size_t USB_THREAD_STACK_DEPTH = 4096u;
    static constexpr std::size_t LED_THREAD_STACK_DEPTH = 1536u;

    static constexpr std::size_t IMU_EVENT_LOG_SIZE = 32u;
  };

  /**
   * @brief Thread stack type.
   */
  template <std::size_t N> struct alignas(std::max_align_t) ThreadStack {
    unsigned char data[N]{};
  };

  /**
   * @brief Start all threads.
   */
  void startThreads() noexcept;

  /**
   * @brief Try to acquire and send IMU sample.
   */
  void tryAcquireAndSendImuSample() noexcept;

  /**
   * @brief IMU data acquisition thread.
   */
  void imuDataAcquisitionThread();

  /**
   * @brief IMU data processing thread.
   */
  void imuDataProcessingThread();

  /**
   * @brief Step detection thread.
   */
  void stepDetectionThread();

  /**
   * @brief Session manager thread.
   */
  void sessionManagerThread();

  /**
   * @brief USB command thread.
   */
  void usbCommandThread();

  /**
   * @brief LED manager thread.
   */
  void ledManagerThread();

  /**
   * @brief Log IMU event.
   * @param type Event type.
   * @param result Result.
   * @param timestampUs Timestamp.
   */
  void logImuEvent(ImuEventType type, platform::Result result,
                   uint32_t timestampUs) noexcept;

  /**
   * @brief Queue a response for the USB thread (non-blocking; drops if full).
   */
  void enqueueUsbResponse(std::string_view text) noexcept;

  imu::Mpu6050Driver &_imu;

  // Single-writer (IMU thread only), lock-free ring buffer for IMU events.
  ImuEvent _imuEventLog[Config::IMU_EVENT_LOG_SIZE]{};
  std::atomic<uint32_t> _imuEventWriteIdx{0};

  ThreadStack<Config::IMU_THREAD_STACK_DEPTH> _stackImu{};
  ThreadStack<Config::SIGNAL_THREAD_STACK_DEPTH> _stackSignal{};
  ThreadStack<Config::STEP_THREAD_STACK_DEPTH> _stackStep{};
  ThreadStack<Config::SESSION_THREAD_STACK_DEPTH> _stackSession{};
  ThreadStack<Config::USB_THREAD_STACK_DEPTH> _stackUsb{};
  ThreadStack<Config::LED_THREAD_STACK_DEPTH> _stackLed{};

  rtos::EventFlags _isrToSensorFlags;
  rtos::EventFlags _sessionToLedFlags;

  rtos::Mail<RawImuDataFrame, Config::MAIL_DEPTH> _sensorToSignalMail;
  rtos::Mail<ProcessedImuDataFrame, Config::MAIL_DEPTH> _signalToStepMail;
  rtos::Mail<StepDetectionEvent, Config::MAIL_DEPTH> _stepToSessionMail;
  rtos::Mail<UsbCommand, Config::MAIL_DEPTH> _usbToSessionMail;
  rtos::Mail<UsbResponse, Config::MAIL_DEPTH> _sessionToUsbMail;

  std::atomic<uint32_t> _imuSeq{0};
  std::atomic<uint32_t> _imuDropCount{0};
  std::atomic<uint32_t> _signalDropCount{0};
  std::atomic<uint32_t> _stepDropCount{0};
  std::atomic<uint32_t> _usbDropCount{0};
  std::atomic<uint8_t> _ledState{0};

  rtos::Thread _imuThread;
  rtos::Thread _signalThread;
  rtos::Thread _stepThread;
  rtos::Thread _sessionThread;
  rtos::Thread _usbThread;
  rtos::Thread _ledThread;
};

} // namespace app

#endif // APP_APPLICATION_HPP
