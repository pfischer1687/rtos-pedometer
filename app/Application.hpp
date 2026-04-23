/**
 * @file app/Application.hpp
 * @brief RTOS threads, IPC payloads, and Application class wiring.
 */

#ifndef APP_APPLICATION_HPP
#define APP_APPLICATION_HPP

#include "app/Command.hpp"
#include "app/MessageTypes.hpp"
#include "imu/Mpu6050Driver.hpp"
#include "rtos/EventFlags.h"
#include "rtos/Mail.h"
#include "rtos/Thread.h"
#include "session/SessionManager.hpp"
#include "signal_processing/SignalProcessing.hpp"
#include "step_detection/StepDetector.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace app {

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
   * @param watchdog Watchdog.
   */
  explicit Application(imu::Mpu6050Driver &imu,
                       platform::IWatchdog &watchdog) noexcept;

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
   * @brief Get IMU health snapshot.
   * @return IMU health snapshot.
   */
  message_types::ImuHealthSnapshot getImuHealthSnapshot() const noexcept;

  /**
   * @brief Thread implementing a watchdog for IMU data silence.
   */
  void watchdogThread();

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
  void logImuEvent(message_types::ImuEventType type, platform::Result result,
                   uint32_t timestampUs) noexcept;

  /**
   * @brief Write a response for the USB thread.
   * @param text Response text.
   * @param slot Response slot.
   */
  void writeUsbResponse(std::string_view text,
                        message_types::UsbResponse *slot) noexcept;

  imu::Mpu6050Driver &_imu;
  platform::IWatchdog &_watchdog;
  signal_processing::SignalProcessor _signalProcessor;
  step_detection::StepDetector _stepDetector;
  session::SessionManager _sessionManager;

  // Single-writer (IMU thread only), lock-free ring buffer for IMU events.
  message_types::ImuEvent _imuEventLog[Config::IMU_EVENT_LOG_SIZE]{};
  std::atomic<uint32_t> _imuEventCount{0};

  ThreadStack<Config::IMU_THREAD_STACK_DEPTH> _stackImu{};
  ThreadStack<Config::SIGNAL_THREAD_STACK_DEPTH> _stackSignal{};
  ThreadStack<Config::STEP_THREAD_STACK_DEPTH> _stackStep{};
  ThreadStack<Config::SESSION_THREAD_STACK_DEPTH> _stackSession{};
  ThreadStack<Config::USB_THREAD_STACK_DEPTH> _stackUsb{};
  ThreadStack<Config::LED_THREAD_STACK_DEPTH> _stackLed{};

  rtos::EventFlags _sessionToLedFlags;

  rtos::Mail<message_types::RawImuDataFrame, Config::MAIL_DEPTH>
      _sensorToSignalMail;
  rtos::Mail<message_types::ProcessedImuDataFrame, Config::MAIL_DEPTH>
      _signalToStepMail;
  rtos::Mail<message_types::StepDetectionEvent, Config::MAIL_DEPTH>
      _stepToSessionMail;
  rtos::Mail<CommandId, Config::MAIL_DEPTH> _usbToSessionMail;
  rtos::Mail<message_types::UsbResponse, Config::MAIL_DEPTH> _sessionToUsbMail;

  std::atomic<uint32_t> _imuSeq{0};
  std::atomic<uint32_t> _imuDropCount{0};
  std::atomic<uint32_t> _signalDropCount{0};
  std::atomic<uint32_t> _stepDropCount{0};
  std::atomic<uint32_t> _sessionDropCount{0};
  std::atomic<uint32_t> _usbDropCount{0};
  std::atomic<uint8_t> _ledState{0};

  rtos::Thread _imuThread;
  rtos::Thread _signalThread;
  rtos::Thread _stepThread;
  rtos::Thread _sessionThread;
  rtos::Thread _usbThread;
  rtos::Thread _ledThread;
  rtos::Thread _watchdogThread;

  std::atomic<uint32_t> _lastImuTickUs{0};
};

} // namespace app

#endif // APP_APPLICATION_HPP
