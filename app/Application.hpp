/**
 * @file app/Application.hpp
 * @brief RTOS threads, IPC payloads, and Application class wiring.
 */

#ifndef APP_APPLICATION_HPP
#define APP_APPLICATION_HPP

#include "app/Command.hpp"
#include "app/MessageTypes.hpp"
#include "imu/Mpu6050Driver.hpp"
#include "led/RecordingLed.hpp"
#include "rtos/EventFlags.h"
#include "rtos/Mail.h"
#include "rtos/Thread.h"
#include "session/SessionManager.hpp"
#include "signal_processing/SignalProcessing.hpp"
#include "step_detection/OscillationTracker.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace app {

/**
 * @enum ImuState
 * @brief IMU health and recovery
 * @details
 * - watchdog single-owner
 * - acquire thread is read-only
 */
enum class ImuState : std::uint8_t { Healthy, Recovering };

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
   * @brief Initialize and configure IMU hardware.
   */
  void bringUpHardware() noexcept;

  /**
   * @brief Start worker threads and block forever.
   */
  [[noreturn]] void run() noexcept;

  /**
   * @brief Get IMU health snapshot.
   * @return IMU health snapshot.
   */
  message_types::ImuHealthSnapshot getImuHealthSnapshot() const noexcept;

private:
  /**
   * @brief Configuration constants.
   */
  struct Config {
    static constexpr uint32_t MAIL_DEPTH = 32u;

    static constexpr uint32_t EVENT_USB_WAKE_SESSION = 1u << 0;
    static constexpr uint32_t EVENT_STEP_WAKE_SESSION = 1u << 1;
    static constexpr uint32_t EVENT_SESSION_WAKE_MASK =
        EVENT_USB_WAKE_SESSION | EVENT_STEP_WAKE_SESSION;

    static constexpr std::size_t IMU_THREAD_STACK_DEPTH = 2048u;
    static constexpr std::size_t SIGNAL_THREAD_STACK_DEPTH = 2048u;
    static constexpr std::size_t STEP_THREAD_STACK_DEPTH = 2048u;
    static constexpr std::size_t SESSION_THREAD_STACK_DEPTH = 2048u;
    static constexpr std::size_t USB_THREAD_STACK_DEPTH = 2048u;
    static constexpr std::size_t LED_THREAD_STACK_DEPTH = 2048u;

    static constexpr std::size_t IMU_EVENT_LOG_SIZE = 32u;

    static constexpr std::uint32_t SESSION_MAX_USB_PER_LOOP = 12u;
    static constexpr std::uint32_t SESSION_MAX_STEPS_PER_LOOP = 12u;
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
   * @brief Thread implementing a watchdog for IMU data silence.
   */
  void watchdogThread();

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

  /**
   * @brief Wait for session wakeup.
   * @return True if session woke up, false if timeout.
   */
  bool waitOnSessionWakeup() noexcept;

  /**
   * @brief Handle USB commands up to a maximum.
   * @param max Maximum number of commands to handle.
   * @return Number of commands handled.
   */
  std::uint32_t handleUsbCommandsUpTo(std::uint32_t max) noexcept;

  /**
   * @brief Handle step events up to a maximum.
   * @param max Maximum number of events to handle.
   * @return Number of events handled.
   */
  std::uint32_t handleStepEventsUpTo(std::uint32_t max) noexcept;

  /**
   * @brief Update session state and LED.
   */
  void updateSessionStateAndLED() noexcept;

  /**
   * @brief Check if IMU is live.
   * @param now Current time.
   * @return True if IMU is live, false otherwise.
   */
  [[nodiscard]] bool isImuLive(std::uint32_t now) const noexcept;

  /**
   * @brief Check if session is healthy.
   * @param now Current time.
   * @return True if session is healthy, false otherwise.
   */
  [[nodiscard]] bool isSessionHealthy(std::uint32_t now) const noexcept;

  /**
   * @brief Handle IMU state.
   * @param now Current time.
   * @param s IMU state.
   */
  void handleImuState(std::uint32_t now, ImuState s) noexcept;

  /**
   * @brief Lossy SPSC ring
   * @details
   * - Signal thread pushes numeric samples only.
   * - USB thread formats and sends.
   * - If USB falls behind, newest samples are dropped at the producer
   *   (real-time path stays bounded).
   */
  struct DspDebugSample {
    std::uint32_t timestampUs{};
    float ax{};
    float ay{};
    float az{};
    float mag{};
    float slope{};
  };

  static constexpr std::uint32_t DSP_DEBUG_RING_CAP = 32u;
  static constexpr std::uint32_t DSP_DEBUG_RING_MASK = DSP_DEBUG_RING_CAP - 1u;
  static_assert((DSP_DEBUG_RING_CAP & (DSP_DEBUG_RING_CAP - 1u)) == 0u &&
                    DSP_DEBUG_RING_CAP >= 8u,
                "DSP debug ring capacity must be a power of two and >= 8");

  /**
   * @brief Push a DSP debug sample to the ring.
   * @param s DSP debug sample.
   */
  void pushDspDebugSampleLossy(const DspDebugSample &s) noexcept;

  /**
   * @brief Drain the DSP debug ring to the USB interface.
   * @param iface USB interface.
   */
  void drainDspDebugRingToUsb(usb::UsbInterface &iface) noexcept;

  DspDebugSample _dspDbgRing[DSP_DEBUG_RING_CAP]{};
  std::atomic<std::uint32_t> _dspDbgIn{0u};
  std::atomic<std::uint32_t> _dspDbgOut{0u};

  imu::Mpu6050Driver &_imu;
  platform::IWatchdog &_watchdog;
  signal_processing::SignalProcessor _signalProcessor;
  step_detection::OscillationTracker _oscillationTracker;
  session::SessionManager _sessionManager;
  led::RecordingLed _recordingLed{};

  // Single-producer (IMU thread only), lock-free ring buffer for IMU events.
  message_types::ImuEvent _imuEventLog[Config::IMU_EVENT_LOG_SIZE]{};
  std::atomic<uint32_t> _imuEventCount{0u};

  ThreadStack<Config::IMU_THREAD_STACK_DEPTH> _stackImu{};
  ThreadStack<Config::SIGNAL_THREAD_STACK_DEPTH> _stackSignal{};
  ThreadStack<Config::STEP_THREAD_STACK_DEPTH> _stackStep{};
  ThreadStack<Config::SESSION_THREAD_STACK_DEPTH> _stackSession{};
  ThreadStack<Config::USB_THREAD_STACK_DEPTH> _stackUsb{};
  ThreadStack<Config::LED_THREAD_STACK_DEPTH> _stackLed{};

  rtos::EventFlags _sessionSignal;

  rtos::Mail<message_types::RawImuDataFrame, Config::MAIL_DEPTH>
      _sensorToSignalMail;
  rtos::Mail<message_types::ProcessedImuDataFrame, Config::MAIL_DEPTH>
      _signalToStepMail;
  rtos::Mail<message_types::StepDetectionEvent, Config::MAIL_DEPTH>
      _stepToSessionMail;
  rtos::Mail<CommandId, Config::MAIL_DEPTH> _usbToSessionMail;
  rtos::Mail<message_types::UsbResponse, Config::MAIL_DEPTH> _sessionToUsbMail;

  std::atomic<uint32_t> _imuSeq{0u};
  std::atomic<uint32_t> _imuDropCount{0u};
  std::atomic<uint32_t> _signalDropCount{0u};
  std::atomic<uint32_t> _stepDropCount{0u};
  std::atomic<uint32_t> _sessionDropCount{0u};
  std::atomic<uint32_t> _usbDropCount{0u};
  std::atomic<led::LedState> _ledState{led::LedState::Idle};
  std::atomic<uint32_t> _sessionHeartbeatUs{0u};
  std::atomic<uint32_t> _ledVersion{0u};
  std::atomic<uint32_t> _lastImuTickUs{0u};
  std::atomic<ImuState> _imuState{ImuState::Healthy};
  std::atomic<bool> _dspDebugStreamEnabled{false};

  rtos::Thread _imuThread;
  rtos::Thread _signalThread;
  rtos::Thread _stepThread;
  rtos::Thread _sessionThread;
  rtos::Thread _usbThread;
  rtos::Thread _ledThread;
  rtos::Thread _watchdogThread;
};

} // namespace app

#endif // APP_APPLICATION_HPP
