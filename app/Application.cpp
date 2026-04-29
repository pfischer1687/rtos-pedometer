/**
 * @file app/Application.cpp
 * @brief Application class: RTOS threads, mail queues, and IPC wiring.
 */

#include "app/Application.hpp"
#include "app/MailHandle.hpp"
#include "cmsis_os2.h"
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
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string_view>

namespace app {

namespace {

constexpr rtos::Kernel::Clock::duration_u32 IMU_DATA_NOT_READY_BACKOFF_MS{1u};
constexpr rtos::Kernel::Clock::duration_u32 SESSION_SIGNAL_WAIT_MS{200u};
constexpr rtos::Kernel::Clock::duration_u32 USB_IDLE_POLL_MS{1u};
constexpr rtos::Kernel::Clock::duration_u32 WATCHDOG_KICK_INTERVAL_MS{500u};
constexpr rtos::Kernel::Clock::duration_u32 LED_BACKOFF_MAX_MS{32u};

constexpr uint32_t WATCHDOG_TIMEOUT_MS = 2000;                   // 2s
constexpr uint32_t WATCHDOG_MAX_IMU_SILENCE_US = 20'000'000u;    // 20s
constexpr uint32_t WATCHDOG_MAX_SESSION_SILENCE_US = 5'000'000u; // 5s

} // namespace

Application::Application(imu::Mpu6050Driver &imu,
                         platform::IWatchdog &watchdog) noexcept
    : _imu(imu), _watchdog(watchdog), _sessionSignal("app_sess"),
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
      .lowPassCutoffHz =
          signal_processing::defaults::DEFAULT_LOW_PASS_CUTOFF_HZ,
      .sampleRateHz = static_cast<uint16_t>(_imu.getImuConfig().odr),
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
  const std::uint32_t n = _imuEventCount.load(std::memory_order_acquire);
  const std::uint32_t idx = n % Config::IMU_EVENT_LOG_SIZE;
  _imuEventLog[idx] = message_types::ImuEvent{
      .timestampUs = timestampUs, .type = type, .result = result};
  _imuEventCount.store(n + 1u, std::memory_order_release);
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
  const platform::Result sampleResult = _imu.readSample(sample);
  if (!platform::isOk(sampleResult)) [[unlikely]] {
    if (sampleResult == platform::Result::DataNotReady) {
      return;
    }

    _imuDropCount.fetch_add(1, std::memory_order_relaxed);
    logImuEvent(message_types::ImuEventType::ReadFail, sampleResult,
                platform::getTimeUs());
    return;
  }

  _lastImuTickUs.store(platform::getTimeUs(), std::memory_order_release);

  message_types::RawImuDataFrame *msg = mailHandle.getMsg();
  msg->sequence = _imuSeq.fetch_add(1, std::memory_order_relaxed);
  msg->sample = sample;

  if (_sensorToSignalMail.put(mailHandle.releaseMsg()) != osOK) {
    _imuDropCount.fetch_add(1, std::memory_order_relaxed);
  }
}

void Application::imuDataAcquisitionThread() {
  while (true) {
    tryAcquireAndSendImuSample();
    rtos::ThisThread::sleep_for(IMU_DATA_NOT_READY_BACKOFF_MS);
  }
}

void Application::imuDataProcessingThread() {
  signal_processing::ProcessedSample processed{};

  while (true) {
    message_types::RawImuDataFrame *rawIn =
        _sensorToSignalMail.try_get_for(rtos::Kernel::wait_for_u32_forever);
    if (!rawIn) [[unlikely]] {
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
    outMsg->accelMagnitudeG = processed.magnitude;

    _signalToStepMail.put(out.releaseMsg());
  }
}

void Application::stepDetectionThread() {
  while (true) {
    message_types::ProcessedImuDataFrame *rawIn =
        _signalToStepMail.try_get_for(rtos::Kernel::wait_for_u32_forever);
    if (!rawIn) [[unlikely]] {
      continue;
    }

    MailHandle<message_types::ProcessedImuDataFrame, Config::MAIL_DEPTH> in{
        _signalToStepMail, rawIn};
    message_types::ProcessedImuDataFrame *inMsg = in.getMsg();

    const step_detection::StepDecision decision = _stepDetector.processSample(
        inMsg->accelMagnitudeG, inMsg->sourceTimestampUs);
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
    _sessionSignal.set(Config::EVENT_STEP_WAKE_SESSION);
  }
}

void Application::writeUsbResponse(std::string_view text,
                                   message_types::UsbResponse *slot) noexcept {
  constexpr std::size_t cap = sizeof(slot->msg) - 1u;
  const std::size_t n = text.size() < cap ? text.size() : cap;
  std::memcpy(slot->msg, text.data(), n);
  slot->msg[n] = '\0';
}

bool Application::waitOnSessionWakeup() noexcept {
  const std::uint32_t w = _sessionSignal.wait_any_for(
      Config::EVENT_SESSION_WAKE_MASK, SESSION_SIGNAL_WAIT_MS, true);

  return (w & osFlagsError) == 0u;
}

std::uint32_t
Application::handleUsbCommandsUpTo(const std::uint32_t max) noexcept {
  std::uint32_t n = 0u;
  for (; n < max; ++n) {
    CommandId *rawInUsb = _usbToSessionMail.try_get();
    if (rawInUsb == nullptr) {
      break;
    }

    MailHandle<CommandId, Config::MAIL_DEPTH> inUsb{_usbToSessionMail,
                                                    rawInUsb};

    MailHandle<message_types::UsbResponse, Config::MAIL_DEPTH> out{
        _sessionToUsbMail};
    if (!out) [[unlikely]] {
      _sessionDropCount.fetch_add(1, std::memory_order_relaxed);

      MailHandle<message_types::UsbResponse, Config::MAIL_DEPTH> busyOut{
          _sessionToUsbMail};
      if (busyOut) {
        writeUsbResponse("BUSY", busyOut.getMsg());
        _sessionToUsbMail.put(busyOut.releaseMsg());
      }

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
      char buf[sizeof(message_types::UsbResponse::msg)]{};
      const std::size_t reportN =
          _sessionManager.formatReport(buf, sizeof(buf));
      const std::string_view text = reportN > 0u
                                        ? std::string_view(buf, reportN)
                                        : std::string_view("FAIL");
      writeUsbResponse(text, out.getMsg());
      break;
    }
    case CommandId::DebugStatus: {
      char buf[sizeof(message_types::UsbResponse::msg)]{};
      _sessionManager.setStepDetectorDebugStats(_stepDetector.getDebugStats());
      const std::size_t reportN =
          _sessionManager.formatDebugReport(buf, sizeof(buf));
      const std::string_view text = reportN > 0u
                                        ? std::string_view(buf, reportN)
                                        : std::string_view("FAIL");
      writeUsbResponse(text, out.getMsg());
      break;
    }
    case CommandId::Reset:
      _imuDropCount.store(0, std::memory_order_release);
      _imuSeq.store(0, std::memory_order_release);
      _imuEventCount.store(0, std::memory_order_release);
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
  return n;
}

void Application::updateSessionStateAndLED() noexcept {
  const led::LedState newState =
      _sessionManager.isActive() ? led::LedState::Active : led::LedState::Idle;
  const led::LedState prev = _ledState.load(std::memory_order_relaxed);
  if (newState != prev) {
    _ledState.store(newState, std::memory_order_release);
    _ledVersion.fetch_add(1, std::memory_order_acq_rel);
  }
}

std::uint32_t
Application::handleStepEventsUpTo(const std::uint32_t max) noexcept {
  std::uint32_t n = 0u;
  for (; n < max; ++n) {
    message_types::StepDetectionEvent *rawInStep = _stepToSessionMail.try_get();
    if (rawInStep == nullptr) {
      break;
    }
    MailHandle<message_types::StepDetectionEvent, Config::MAIL_DEPTH> inStep{
        _stepToSessionMail, rawInStep};
    message_types::StepDetectionEvent *inMsgStep = inStep.getMsg();

    step_detection::StepEvent evt{};
    evt.timestampUs = inMsgStep->peakTimeUs;
    evt.confidence = inMsgStep->confidence;
    _sessionManager.onStep(evt);
  }
  return n;
}

void Application::sessionManagerThread() {
  auto drain_once = [&] {
    std::uint32_t usb = handleUsbCommandsUpTo(Config::SESSION_MAX_USB_PER_LOOP);
    updateSessionStateAndLED();
    std::uint32_t step =
        handleStepEventsUpTo(Config::SESSION_MAX_STEPS_PER_LOOP);
    return usb || step;
  };

  while (true) {
    _sessionHeartbeatUs.store(platform::getTimeUs(), std::memory_order_relaxed);

    if (drain_once()) {
      continue;
    }

    if (!waitOnSessionWakeup()) {
      continue;
    }

    if (drain_once()) {
      continue;
    }
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
    while ((resp = _sessionToUsbMail.try_get()) != nullptr) {
      MailHandle<message_types::UsbResponse, Config::MAIL_DEPTH> inSess{
          _sessionToUsbMail, resp};
      message_types::UsbResponse *inMsg = inSess.getMsg();
      iface.sendResponse(inMsg->msg);
      iface.printPrompt();
    }

    if (!iface.poll(lineBuf, sizeof(lineBuf))) {
      rtos::ThisThread::sleep_for(USB_IDLE_POLL_MS);
      continue;
    }

    lineBuf[usb::USB_CMD_MAX_LEN - 1u] = '\0';
    std::size_t lineLen = 0u;
    while (lineLen < usb::USB_CMD_MAX_LEN && lineBuf[lineLen] != '\0') {
      ++lineLen;
    }
    const usb::ParsedLine pl =
        usb::parseLine(std::string_view(lineBuf, lineLen));
    const std::optional<CommandId> cmdOpt = parseCommand(pl);
    if (cmdOpt.has_value()) {
      MailHandle<CommandId, Config::MAIL_DEPTH> out{_usbToSessionMail};
      if (!out) [[unlikely]] {
        _usbDropCount.fetch_add(1, std::memory_order_relaxed);
        iface.sendResponse("CMD_QUEUE_FULL");
        iface.printPrompt();
        continue;
      }
      *out.getMsg() = *cmdOpt;
      _usbToSessionMail.put(out.releaseMsg());
      _sessionSignal.set(Config::EVENT_USB_WAKE_SESSION);
      continue;
    }

    if (pl.name.empty()) {
      iface.printPrompt();
      continue;
    }

    iface.sendResponse("UNKNOWN_COMMAND");
    iface.printPrompt();
  }
}

void Application::ledManagerThread() {
  uint32_t lastApplied = 0;
  rtos::Kernel::Clock::duration_u32 idleSleep{1u};
  const std::uint32_t cap = LED_BACKOFF_MAX_MS.count();

  while (true) {
    const uint32_t v = _ledVersion.load(std::memory_order_acquire);
    if (v == lastApplied) {
      rtos::ThisThread::sleep_for(idleSleep);
      const std::uint32_t cur = idleSleep.count();
      const std::uint64_t doubled = static_cast<std::uint64_t>(cur) * 2U;
      const std::uint32_t nxt = (doubled >= static_cast<std::uint64_t>(cap))
                                    ? cap
                                    : static_cast<std::uint32_t>(doubled);
      idleSleep = rtos::Kernel::Clock::duration_u32{nxt};
      continue;
    }
    lastApplied = v;
    idleSleep = rtos::Kernel::Clock::duration_u32{1u};
    const led::LedState state = _ledState.load(std::memory_order_acquire);
    _recordingLed.apply(state);
  }
}

message_types::ImuHealthSnapshot
Application::getImuHealthSnapshot() const noexcept {
  const uint32_t seq = _imuSeq.load(std::memory_order_relaxed);
  const uint32_t drop = _imuDropCount.load(std::memory_order_relaxed);
  const uint32_t events = _imuEventCount.load(std::memory_order_acquire);

  const uint32_t buffered = (events < Config::IMU_EVENT_LOG_SIZE)
                                ? events
                                : Config::IMU_EVENT_LOG_SIZE;

  return message_types::ImuHealthSnapshot{.totalSamples = seq,
                                          .dropCount = drop,
                                          .totalEvents = events,
                                          .bufferedEvents = buffered};
}

bool Application::isImuLive(const std::uint32_t now) const noexcept {
  const std::uint32_t lastImuUs =
      _lastImuTickUs.load(std::memory_order_acquire);
  const std::uint32_t imuSilenceUs = platform::elapsed(lastImuUs, now);
  return (imuSilenceUs <= WATCHDOG_MAX_IMU_SILENCE_US);
}

bool Application::isSessionHealthy(const std::uint32_t now) const noexcept {
  const std::uint32_t sh = _sessionHeartbeatUs.load(std::memory_order_relaxed);
  return (sh != 0U) &&
         (platform::elapsed(sh, now) <= WATCHDOG_MAX_SESSION_SILENCE_US);
}

void Application::handleImuState(std::uint32_t now, ImuState s) noexcept {
  switch (s) {

  case ImuState::Healthy: {
    const std::uint32_t last = _lastImuTickUs.load(std::memory_order_acquire);
    if (platform::elapsed(last, now) > WATCHDOG_MAX_IMU_SILENCE_US) {
      _imuState.store(ImuState::Recovering, std::memory_order_release);
    }
    break;
  }

  case ImuState::Recovering: {
    (void)platform::isOk(_imu.stopSampling());

    if (!platform::isOk(_imu.reset()) || !platform::isOk(_imu.init())) {
      break;
    }

    const imu::ImuConfig cfg = _imu.getImuConfig();
    if (!platform::isOk(_imu.configure(cfg)) ||
        !platform::isOk(_imu.startSampling())) {
      break;
    }

    _imuState.store(ImuState::Healthy, std::memory_order_release);
    break;
  }
  }
}

void Application::watchdogThread() {
  MBED_ASSERT(platform::isOk(_watchdog.start(WATCHDOG_TIMEOUT_MS)));

  while (true) {
    const std::uint32_t now = platform::getTimeUs();
    const ImuState s = _imuState.load(std::memory_order_acquire);
    handleImuState(now, s);

    if (isImuLive(now) && isSessionHealthy(now)) [[likely]] {
      _watchdog.kick();
    }
    rtos::ThisThread::sleep_for(WATCHDOG_KICK_INTERVAL_MS);
  }
}

void Application::startThreads() noexcept {
  _sessionHeartbeatUs.store(platform::getTimeUs(), std::memory_order_relaxed);

  osStatus s =
      _imuThread.start(callback(this, &Application::imuDataAcquisitionThread));
  MBED_ASSERT(s == osOK);
  s = _signalThread.start(
      callback(this, &Application::imuDataProcessingThread));
  MBED_ASSERT(s == osOK);
  s = _stepThread.start(callback(this, &Application::stepDetectionThread));
  MBED_ASSERT(s == osOK);
  s = _sessionThread.start(callback(this, &Application::sessionManagerThread));
  MBED_ASSERT(s == osOK);
  s = _usbThread.start(callback(this, &Application::usbCommandThread));
  MBED_ASSERT(s == osOK);
  s = _ledThread.start(callback(this, &Application::ledManagerThread));
  MBED_ASSERT(s == osOK);
  s = _watchdogThread.start(callback(this, &Application::watchdogThread));
  MBED_ASSERT(s == osOK);
}

[[noreturn]] void Application::run() noexcept {
  bringUpHardware();
  startThreads();

  while (true) {
    rtos::ThisThread::sleep_for(rtos::Kernel::wait_for_u32_forever);
  }
}

} // namespace app
