/**
 * @file app/Application.cpp
 * @brief Application class: RTOS threads, mail queues, and IPC wiring.
 */

#include "app/Application.hpp"
#include "app/MailHandle.hpp"
#include "imu/Mpu6050Driver.hpp"
#include "mbed_assert.h"
#include "platform/Callback.h"
#include "platform/Platform.hpp"
#include "rtos/EventFlags.h"
#include "rtos/Kernel.h"
#include "rtos/Mail.h"
#include "rtos/ThisThread.h"
#include "rtos/Thread.h"
#include "session/SessionManager.hpp"
#include "usb/UsbInterface.hpp"
#include "usb/UsbTransport.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string_view>

namespace app {

namespace {

constexpr std::chrono::milliseconds SESSION_POLL_INTERVAL = 25ms;
constexpr std::chrono::milliseconds THREAD_SLEEP_INTERVAL = 1ms;
constexpr float MILLI_G_PER_G = 1000.0f;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 2000;
constexpr std::chrono::milliseconds WATCHDOG_KICK_INTERVAL = 500ms;
constexpr uint32_t WATCHDOG_MAX_IMU_SILENCE_US = 2'000'000u;
constexpr float WATCHDOG_MAX_IMU_DROPPED_FRACTION = 0.5f;

} // anonymous namespace

Application::Application(imu::Mpu6050Driver &imu,
                         platform::IWatchdog &watchdog) noexcept
    : _imu(imu), _watchdog(watchdog), _sessionToLedFlags("led_evt"),
      _imuThread(osPriorityRealtime, Config::IMU_THREAD_STACK_DEPTH,
                 _stackImu.data, "imu_acq"),
      _signalThread(osPriorityHigh, Config::SIGNAL_THREAD_STACK_DEPTH,
                    _stackSignal.data, "sig_proc"),
      _stepThread(osPriorityHigh, Config::STEP_THREAD_STACK_DEPTH,
                  _stackStep.data, "step_det"),
      _sessionThread(osPriorityBelowNormal, Config::SESSION_THREAD_STACK_DEPTH,
                     _stackSession.data, "session"),
      _usbThread(osPriorityLow, Config::USB_THREAD_STACK_DEPTH, _stackUsb.data,
                 "usb_cmd"),
      _ledThread(osPriorityLow, Config::LED_THREAD_STACK_DEPTH, _stackLed.data,
                 "led_mgr") {
  signal_processing::FilterConfig config{
      .highPassCutoffHz =
          signal_processing::defaults::DEFAULT_HIGH_PASS_CUTOFF_HZ,
      .sampleRateHz = static_cast<uint16_t>(_imu.getImuConfig().odr),
      .movingAverageWindow =
          signal_processing::defaults::DEFAULT_MOVING_AVERAGE_WINDOW,
      .accelScale =
          signal_processing::getAccelScale(_imu.getImuConfig().range)};

  _signalProcessor.setConfig(config);

  step_detection::StepDetectorConfig stepCfg{};
  _stepDetector.setConfig(stepCfg);

  _recordingLed.apply(led::LedState::Idle);
}

void Application::bringUpHardware() noexcept {
  MBED_ASSERT(platform::isOk(_imu.init()));

  imu::ImuConfig config{};
  MBED_ASSERT(platform::isOk(_imu.configure(config)));

  MBED_ASSERT(platform::isOk(_imu.startSampling()));
}

void Application::logImuEvent(message_types::ImuEventType type,
                              platform::Result result,
                              uint32_t timestampUs) noexcept {
  const uint32_t count = _imuEventCount.fetch_add(1, std::memory_order_relaxed);
  const uint32_t idx = count % Config::IMU_EVENT_LOG_SIZE;
  _imuEventLog[idx] = message_types::ImuEvent{
      .timestampUs = timestampUs, .type = type, .result = result};
}

void Application::tryAcquireAndSendImuSample() noexcept {
  MailHandle<message_types::RawImuDataFrame, Config::MAIL_DEPTH> mailHandle{
      _sensorToSignalMail};
  if (!mailHandle) [[unlikely]] {
    _imuDropCount.fetch_add(1, std::memory_order_relaxed);
    logImuEvent(message_types::ImuEventType::MailAllocFail,
                platform::Result::Busy, platform::getTimeUs());
    return;
  }

  imu::ImuSample sample{};
  platform::Result sampleResult = _imu.readSample(sample);
  if (!platform::isOk(sampleResult)) [[unlikely]] {
    if (sampleResult == platform::Result::DataNotReady) {
      return;
    }

    _imuDropCount.fetch_add(1, std::memory_order_relaxed);
    logImuEvent(message_types::ImuEventType::ReadFail, sampleResult,
                platform::getTimeUs());
    return;
  }

  message_types::RawImuDataFrame *msg = mailHandle.getMsg();
  msg->sequence = _imuSeq.fetch_add(1, std::memory_order_relaxed);
  msg->sample = sample;

  _lastImuTickUs.store(sample.timestampUs, std::memory_order_relaxed);
  _sensorToSignalMail.put(mailHandle.releaseMsg());
}

void Application::imuDataAcquisitionThread() {
  while (true) {
    if (!_imu.consumeDataReady()) {
      continue;
    }

    tryAcquireAndSendImuSample();
  }
}

void Application::imuDataProcessingThread() {
  signal_processing::ProcessedSample processed{};

  while (true) {
    message_types::RawImuDataFrame *rawIn = _sensorToSignalMail.try_get_for(
        rtos::Kernel::Clock::duration_u32::max());
    if (!rawIn) {
      continue;
    }

    MailHandle<message_types::RawImuDataFrame, Config::MAIL_DEPTH> in{
        _sensorToSignalMail, rawIn};
    MailHandle<message_types::ProcessedImuDataFrame, Config::MAIL_DEPTH> out{
        _signalToStepMail};
    if (!out) [[unlikely]] {
      _signalDropCount.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    message_types::RawImuDataFrame *inMsg = in.getMsg();
    message_types::ProcessedImuDataFrame *outMsg = out.getMsg();

    _signalProcessor.processOne(inMsg->sample, processed);

    outMsg->sequence = inMsg->sequence;
    outMsg->sourceTimestampUs = inMsg->sample.timestampUs;
    outMsg->accelMagnitudeMilliG =
        static_cast<int32_t>(processed.magnitude * MILLI_G_PER_G);

    _signalToStepMail.put(out.releaseMsg());
  }
}

void Application::stepDetectionThread() {
  while (true) {
    message_types::ProcessedImuDataFrame *rawIn =
        _signalToStepMail.try_get_for(rtos::Kernel::wait_for_u32_forever);
    if (!rawIn) {
      continue;
    }

    MailHandle<message_types::ProcessedImuDataFrame, Config::MAIL_DEPTH> in{
        _signalToStepMail, rawIn};
    message_types::ProcessedImuDataFrame *inMsg = in.getMsg();

    const float magG =
        static_cast<float>(inMsg->accelMagnitudeMilliG) / MILLI_G_PER_G;
    const step_detection::StepDecision decision =
        _stepDetector.processSample(magG, inMsg->sourceTimestampUs);
    if (!decision.event) {
      continue;
    }

    MailHandle<message_types::StepDetectionEvent, Config::MAIL_DEPTH> out{
        _stepToSessionMail};
    if (!out) [[unlikely]] {
      _stepDropCount.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    message_types::StepDetectionEvent *outMsg = out.getMsg();
    // NOTE: sequence corresponds to triggering sample (_m2), not peak sample
    // (_m1)
    outMsg->sequence = inMsg->sequence;
    outMsg->peakTimeUs = decision.event->timestampUs;
    outMsg->confidence = decision.event->confidence;

    _stepToSessionMail.put(out.releaseMsg());
  }
}

void Application::writeUsbResponse(std::string_view text,
                                   message_types::UsbResponse *slot) noexcept {
  constexpr std::size_t cap = sizeof(slot->msg) - 1u;
  const std::size_t n = text.size() < cap ? text.size() : cap;
  std::memcpy(slot->msg, text.data(), n);
  slot->msg[n] = '\0';
}

void Application::sessionManagerThread() {
  while (true) {
    CommandId *rawInUsb;
    while ((rawInUsb = _usbToSessionMail.try_get())) {
      MailHandle<CommandId, Config::MAIL_DEPTH> inUsb{_usbToSessionMail,
                                                      rawInUsb};

      MailHandle<message_types::UsbResponse, Config::MAIL_DEPTH> out{
          _sessionToUsbMail};
      if (!out) [[unlikely]] {
        _sessionDropCount.fetch_add(1, std::memory_order_relaxed);
        continue;
      }

      switch (*inUsb.getMsg()) {
      case CommandId::Start: {
        const bool ok = _sessionManager.startSession(platform::getTimeUs());
        writeUsbResponse(ok ? "OK" : "FAIL", out.getMsg());
        break;
      }
      case CommandId::Stop: {
        const bool ok = _sessionManager.stopSession(platform::getTimeUs());
        writeUsbResponse(ok ? "OK" : "FAIL", out.getMsg());
        break;
      }
      case CommandId::Status: {
        char buf[sizeof(message_types::UsbResponse::msg)];
        const std::size_t n = _sessionManager.formatReport(buf, sizeof(buf));
        std::string_view text = n > 0u ? std::string_view(buf, n) : "FAIL";
        writeUsbResponse(text, out.getMsg());
        break;
      }
      case CommandId::DebugStatus: {
        char buf[sizeof(message_types::UsbResponse::msg)];
        _sessionManager.setStepDetectorDebugStats(
            _stepDetector.getDebugStats());
        const std::size_t n =
            _sessionManager.formatDebugReport(buf, sizeof(buf));
        std::string_view text = n > 0u ? std::string_view(buf, n) : "FAIL";
        writeUsbResponse(text, out.getMsg());
        break;
      }
      case CommandId::Reset:
        _imuDropCount.store(0, std::memory_order_relaxed);
        _imuSeq.store(0, std::memory_order_relaxed);
        _imuEventCount.store(0, std::memory_order_relaxed);
        _sessionManager.resetDebugStats();
        _stepDetector.resetDebugStats();
        (void)_sessionManager.stopSession(platform::getTimeUs());
        writeUsbResponse("OK", out.getMsg());
        break;
      default:
        writeUsbResponse("INVALID_COMMAND", out.getMsg());
        break;
      }

      _sessionToUsbMail.put(out.releaseMsg());
    }

    const led::LedState newState = _sessionManager.isActive()
                                       ? led::LedState::Active
                                       : led::LedState::Idle;
    const led::LedState prev = _ledState.load(std::memory_order_relaxed);
    if (newState != prev) {
      _ledState.store(newState, std::memory_order_release);
      _sessionToLedFlags.set(Config::EVENT_LED_UPDATE);
    }

    message_types::StepDetectionEvent *rawInStep =
        _stepToSessionMail.try_get_for(SESSION_POLL_INTERVAL);
    if (!rawInStep) {
      continue;
    }
    MailHandle<message_types::StepDetectionEvent, Config::MAIL_DEPTH> inStep{
        _stepToSessionMail, rawInStep};
    message_types::StepDetectionEvent *inMsgStep = inStep.getMsg();

    step_detection::StepEvent evt{};
    evt.timestampUs = inMsgStep->peakTimeUs;
    evt.confidence = inMsgStep->confidence;

    _sessionManager.onStep(evt);
  }
}

void Application::usbCommandThread() {
  usb::UsbTransport transport;
  usb::UsbInterface iface(transport);

  iface.sendResponse("BOOT RTOS_THREADS");
  iface.printPrompt();

  char lineBuf[usb::USB_CMD_MAX_LEN];

  while (true) {
    message_types::UsbResponse *resp;
    while ((resp = _sessionToUsbMail.try_get())) {
      MailHandle<message_types::UsbResponse, Config::MAIL_DEPTH> inSess{
          _sessionToUsbMail, resp};
      message_types::UsbResponse *inMsg = inSess.getMsg();

      iface.sendResponse(inMsg->msg);
    }

    if (!iface.poll(lineBuf, sizeof(lineBuf))) {
      rtos::ThisThread::sleep_for(THREAD_SLEEP_INTERVAL);
      continue;
    }

    const usb::ParsedLine pl = usb::parseLine(std::string_view(lineBuf));
    if (std::optional<CommandId> cmd = parseCommand(pl)) {
      MailHandle<CommandId, Config::MAIL_DEPTH> out{_usbToSessionMail};
      if (!out) [[unlikely]] {
        _usbDropCount.fetch_add(1, std::memory_order_relaxed);
        iface.sendResponse("CMD_QUEUE_FULL");
        iface.printPrompt();
        continue;
      }

      *out.getMsg() = *cmd;
      _usbToSessionMail.put(out.releaseMsg());
    }

    iface.sendResponse("UNKNOWN_COMMAND");
    iface.printPrompt();
  }
}

void Application::ledManagerThread() {
  while (true) {
    const uint32_t w = _sessionToLedFlags.wait_any_for(
        Config::EVENT_LED_UPDATE, rtos::Kernel::wait_for_u32_forever, true);
    if ((w & osFlagsError) != 0u) {
      continue;
    }

    const led::LedState state = _ledState.load(std::memory_order_acquire);
    _recordingLed.apply(state);
  }
}

message_types::ImuHealthSnapshot
Application::getImuHealthSnapshot() const noexcept {
  const uint32_t seq = _imuSeq.load(std::memory_order_relaxed);
  const uint32_t drop = _imuDropCount.load(std::memory_order_relaxed);
  const uint32_t events = _imuEventCount.load(std::memory_order_relaxed);

  const uint32_t buffered = (events < Config::IMU_EVENT_LOG_SIZE)
                                ? events
                                : Config::IMU_EVENT_LOG_SIZE;

  return message_types::ImuHealthSnapshot{.totalSamples = seq,
                                          .dropCount = drop,
                                          .totalEvents = events,
                                          .bufferedEvents = buffered};
}

void Application::watchdogThread() {
  while (true) {
    const uint32_t lastImuTickUs =
        _lastImuTickUs.load(std::memory_order_relaxed);
    const uint32_t imuSilenceUs =
        platform::elapsed(lastImuTickUs, platform::getTimeUs());
    const bool imuAlive = (imuSilenceUs <= WATCHDOG_MAX_IMU_SILENCE_US);

    const message_types::ImuHealthSnapshot imuHealth = getImuHealthSnapshot();
    const float droppedFraction = static_cast<float>(imuHealth.dropCount) /
                                  static_cast<float>(imuHealth.totalSamples);
    const bool imuHealthy =
        (droppedFraction < WATCHDOG_MAX_IMU_DROPPED_FRACTION);

    if (imuAlive && imuHealthy) [[likely]] {
      _watchdog.kick();
    }

    rtos::ThisThread::sleep_for(WATCHDOG_KICK_INTERVAL);
  }
}

void Application::startThreads() noexcept {
  using mbed::callback;

  MBED_ASSERT(_watchdog.start(WATCHDOG_TIMEOUT_MS) == platform::Result::Ok);

  MBED_ASSERT(_imuThread.start(callback(
                  this, &Application::imuDataAcquisitionThread)) == osOK);
  MBED_ASSERT(_signalThread.start(callback(
                  this, &Application::imuDataProcessingThread)) == osOK);
  MBED_ASSERT(_stepThread.start(
                  callback(this, &Application::stepDetectionThread)) == osOK);
  MBED_ASSERT(_sessionThread.start(
                  callback(this, &Application::sessionManagerThread)) == osOK);
  MBED_ASSERT(
      _usbThread.start(callback(this, &Application::usbCommandThread)) == osOK);
  MBED_ASSERT(
      _ledThread.start(callback(this, &Application::ledManagerThread)) == osOK);
  MBED_ASSERT(_watchdogThread.start(
                  callback(this, &Application::watchdogThread)) == osOK);
}

[[noreturn]] void Application::run() noexcept {
  bringUpHardware();
  startThreads();

  while (true) {
    rtos::ThisThread::sleep_for(rtos::Kernel::wait_for_u32_forever);
  }
}

} // namespace app
