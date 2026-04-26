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

constexpr float MILLI_G_PER_G = 1000.0f;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 2000;                   // 2s
constexpr uint32_t WATCHDOG_MAX_IMU_SILENCE_US = 10'000'000u;    // 10s
constexpr uint32_t WATCHDOG_MAX_SESSION_SILENCE_US = 5'000'000u; // 5s
constexpr uint32_t WATCHDOG_IMU_DROP_WINDOW_US = 30'000'000u;    // 30s
constexpr uint32_t IMU_THREAD_STALL_RECOVER_US = 2'000'000u;     // 2s
constexpr std::uint32_t IMU_STALL_HYSTERESIS_US =
    200'000u; // 200ms (Healthy <-> Stalled)
constexpr std::uint32_t IMU_STALL_STALE_THRESHOLD_US =
    IMU_THREAD_STALL_RECOVER_US + IMU_STALL_HYSTERESIS_US;
constexpr std::uint32_t IMU_WARMUP_MIN_US = 300'000u; // 300ms
constexpr uint32_t MAX_IMU_SNAPSHOT_ATTEMPTS = 8u;
constexpr uint32_t INVERSE_DROP_RATE_FACTOR =
    10u; // 1/10 -> drops > 10% of samples

/**
 * @brief Check if IMU I/O is allowed.
 * @param s IMU state.
 * @param inRecovery True if IMU is in recovery.
 * @return True if IMU I/O is allowed, false otherwise.
 * @details
 * - Warmup is allowed only after Recovering has completed startSampling()
 * - Flag blocks all I/O while the watchdog mutates the driver (Recovering)
 */
[[nodiscard]] inline bool isImuIoAllowed(const ImuState s,
                                         const bool inRecovery) noexcept {
  return (s == ImuState::Healthy || s == ImuState::Warmup) && !inRecovery;
}

/**
 * @brief Whether a healthy IMU should be marked as stalled.
 * @param now Current time.
 * @param lastTickUs Last tick time.
 * @param bootTimeUs Boot time.
 * @return True if IMU should stall, false otherwise.
 */
[[nodiscard]] inline bool
imuHealthyShouldStall(const std::uint32_t now, const std::uint32_t lastTickUs,
                      const std::uint32_t bootTimeUs) noexcept {
  if (bootTimeUs == 0u) {
    return false;
  }

  if (lastTickUs == 0u) {
    return platform::elapsed(bootTimeUs, now) > IMU_THREAD_STALL_RECOVER_US;
  }

  return platform::elapsed(lastTickUs, now) > IMU_STALL_STALE_THRESHOLD_US;
}

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
  const std::uint32_t n = _imuEventCount.load(std::memory_order_acquire);
  const std::uint32_t idx = n % Config::IMU_EVENT_LOG_SIZE;
  _imuEventLog[idx] = message_types::ImuEvent{
      .timestampUs = timestampUs, .type = type, .result = result};
  _imuEventCount.store(n + 1u, std::memory_order_release);
}

void Application::tryAcquireAndSendImuSample() noexcept {
  const ImuState s = _imuState.load(std::memory_order_acquire);
  const bool recovery = _imuDriverInRecovery.load(std::memory_order_acquire);
  if (!isImuIoAllowed(s, recovery)) {
    return;
  }

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

  message_types::RawImuDataFrame *msg = mailHandle.getMsg();
  msg->sequence = _imuSeq.fetch_add(1, std::memory_order_relaxed);
  msg->sample = sample;

  if (_sensorToSignalMail.put(mailHandle.releaseMsg()) == osOK) {
    const std::uint32_t t = platform::getTimeUs();
    _lastImuTickUs.store(t, std::memory_order_release);
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
    outMsg->accelMagnitudeMilliG =
        static_cast<int32_t>(processed.magnitude * MILLI_G_PER_G);

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
      _wDogImuWindowStartUs.store(0u, std::memory_order_release);
      _wDogImuWindowBaseDrops.store(0u, std::memory_order_release);
      _wDogImuWindowBaseSeq.store(0u, std::memory_order_release);
      _wDogImuSnapshotGen.fetch_add(1u, std::memory_order_release);
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

ImuSnapshot Application::readImuSnapshot() const noexcept {
  ImuSnapshot snapshot{};

  for (std::uint32_t attempt = 0u; attempt < MAX_IMU_SNAPSHOT_ATTEMPTS;
       ++attempt) {
    const std::uint32_t versionBegin =
        _wDogImuSnapshotGen.load(std::memory_order_acquire);

    snapshot.totalDrops = _imuDropCount.load(std::memory_order_relaxed);
    snapshot.totalSeq = _imuSeq.load(std::memory_order_relaxed);

    snapshot.windowStart =
        _wDogImuWindowStartUs.load(std::memory_order_acquire);
    snapshot.baseDrops =
        _wDogImuWindowBaseDrops.load(std::memory_order_acquire);
    snapshot.baseSeq = _wDogImuWindowBaseSeq.load(std::memory_order_acquire);

    const std::uint32_t versionEnd =
        _wDogImuSnapshotGen.load(std::memory_order_acquire);

    if (versionBegin == versionEnd) {
      return snapshot;
    }
  }

  return snapshot;
}

bool Application::isImuDropRateOk(const ImuSnapshot &s,
                                  const std::uint32_t now) noexcept {
  std::uint32_t totalDrops = s.totalDrops;
  std::uint32_t totalSeq = s.totalSeq;
  std::uint32_t wStart = s.windowStart;
  std::uint32_t d0 = s.baseDrops;
  std::uint32_t s0 = s.baseSeq;

  if (wStart == 0U || (now - wStart) >= WATCHDOG_IMU_DROP_WINDOW_US) {
    wStart = now;
    d0 = totalDrops;
    s0 = totalSeq;
    _wDogImuWindowStartUs.store(wStart, std::memory_order_relaxed);
    _wDogImuWindowBaseDrops.store(d0, std::memory_order_relaxed);
    _wDogImuWindowBaseSeq.store(s0, std::memory_order_relaxed);
  }

  if (totalDrops < d0 || totalSeq < s0) {
    wStart = now;
    d0 = totalDrops;
    s0 = totalSeq;
    _wDogImuWindowStartUs.store(wStart, std::memory_order_relaxed);
    _wDogImuWindowBaseDrops.store(d0, std::memory_order_relaxed);
    _wDogImuWindowBaseSeq.store(s0, std::memory_order_relaxed);
  }

  const std::uint32_t wDrops = totalDrops - d0;
  const std::uint32_t wSampleDelta = totalSeq - s0;
  if (wSampleDelta == 0U) {
    return (wDrops == 0U);
  }
  return (static_cast<std::uint64_t>(wDrops) * INVERSE_DROP_RATE_FACTOR <
          static_cast<std::uint64_t>(wSampleDelta));
}

WatchdogSnapshot
Application::evaluateWatchdog(const std::uint32_t now) noexcept {
  WatchdogSnapshot w{};
  w.imuLive = isImuLive(now);
  w.imuDropsOk = isImuDropRateOk(readImuSnapshot(), now);
  w.sessionOk = isSessionHealthy(now);
  return w;
}

void Application::handleImuState(std::uint32_t now, ImuState s) noexcept {
  switch (s) {

  case ImuState::Healthy: {
    const std::uint32_t last = _lastImuTickUs.load(std::memory_order_acquire);
    const std::uint32_t tBoot = _appBootTimeUs.load(std::memory_order_relaxed);

    if (imuHealthyShouldStall(now, last, tBoot)) {
      _imuState.store(ImuState::Stalled, std::memory_order_release);
    }

    break;
  }

  case ImuState::Stalled:
    _imuDriverInRecovery.store(true, std::memory_order_release);
    _imuState.store(ImuState::Recovering, std::memory_order_release);
    break;

  case ImuState::Recovering: {
    (void)platform::isOk(_imu.stopSampling());

    if (!platform::isOk(_imu.reset()) || !platform::isOk(_imu.init())) {
      _imuDriverInRecovery.store(false, std::memory_order_release);
      _imuState.store(ImuState::Stalled, std::memory_order_release);
      break;
    }

    const imu::ImuConfig cfg = _imu.getImuConfig();
    if (!platform::isOk(_imu.configure(cfg)) ||
        !platform::isOk(_imu.startSampling())) {
      _imuDriverInRecovery.store(false, std::memory_order_release);
      _imuState.store(ImuState::Stalled, std::memory_order_release);
      break;
    }

    _imuRecoveryStartUs.store(platform::getTimeUs(), std::memory_order_release);
    _imuState.store(ImuState::Warmup, std::memory_order_release);
    _imuDriverInRecovery.store(false, std::memory_order_release);
    break;
  }

  case ImuState::Warmup: {
    const std::uint32_t tWarm0 =
        _imuRecoveryStartUs.load(std::memory_order_acquire);

    if (platform::elapsed(tWarm0, now) >= IMU_WARMUP_MIN_US) {
      _imuState.store(ImuState::Healthy, std::memory_order_release);
      _lastImuTickUs.store(platform::getTimeUs(), std::memory_order_release);
    }
    break;
  }

  default:
    break;
  }
}

void Application::watchdogThread() {
  MBED_ASSERT(platform::isOk(_watchdog.start(WATCHDOG_TIMEOUT_MS)));

  while (true) {
    const std::uint32_t now = platform::getTimeUs();
    const ImuState s = _imuState.load(std::memory_order_acquire);
    handleImuState(now, s);

    const WatchdogSnapshot snap = evaluateWatchdog(now);
    if (snap.imuLive && snap.imuDropsOk && snap.sessionOk) [[likely]] {
      _watchdog.kick();
    }
    rtos::ThisThread::sleep_for(WATCHDOG_KICK_INTERVAL_MS);
  }
}

void Application::startThreads() noexcept {
  _appBootTimeUs.store(platform::getTimeUs(), std::memory_order_release);
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
