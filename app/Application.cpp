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

#include <algorithm>
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

[[nodiscard]] uint8_t confidenceFloatToUint8(float c) noexcept {
  const float x = std::clamp(c, 0.0f, 1.0f);
  return static_cast<uint8_t>(x * 255.0f);
}

} // anonymous namespace

Application::Application(imu::Mpu6050Driver &imu) noexcept
    : _imu(imu), _sessionToLedFlags("led_evt"),
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
    outMsg->confidence = confidenceFloatToUint8(decision.event->confidence);

    _stepToSessionMail.put(out.releaseMsg());
  }
}

void Application::enqueueUsbResponse(std::string_view text) noexcept {
  message_types::UsbResponse *slot = _sessionToUsbMail.try_alloc();
  if (!slot) {
    return;
  }

  constexpr std::size_t cap = sizeof(slot->msg) - 1u;
  const std::size_t n = text.size() < cap ? text.size() : cap;
  std::memcpy(slot->msg, text.data(), n);
  slot->msg[n] = '\0';
  _sessionToUsbMail.put(slot);
}

void Application::sessionManagerThread() {
  while (true) {
    message_types::StepDetectionEvent *in =
        _stepToSessionMail.try_get_for(SESSION_POLL_INTERVAL);

    CommandId *appCmd;
    while ((appCmd = _usbToSessionMail.try_get())) {
      // TODO: handle session commands here.
      switch (*appCmd) {
      case CommandId::Start: {
        const bool ok = _sessionManager.startSession(platform::getTimeUs());
        enqueueUsbResponse(ok ? "OK" : "FAIL");
        break;
      }
      case CommandId::Stop: {
        const bool ok = _sessionManager.stopSession(platform::getTimeUs());
        enqueueUsbResponse(ok ? "OK" : "FAIL");
        break;
      }
      case CommandId::Status: {
        char buf[sizeof(message_types::UsbResponse::msg)];
        const std::size_t n = _sessionManager.formatReport(buf, sizeof(buf));
        if (n > 0u) {
          enqueueUsbResponse(std::string_view(buf, n));
        }
        break;
      }
      case CommandId::TuneStatus: {
        const session::TuningSnapshot t = _sessionManager.getTuningSnapshot();
        const auto &st = t.detectorStats;
        if (t.acceptedStepCount == 0u && t.rejectedStepCount == 0u &&
            st.peaks == 0u && st.emitted == 0u && st.rejectSlope == 0u &&
            st.rejectInterval == 0u) {
          enqueueUsbResponse("TUNE inactive");
          break;
        }
        char buf[sizeof(message_types::UsbResponse::msg)];
        const int n =
            std::snprintf(buf, sizeof(buf),
                          "TUNE accepted=%lu rejected=%lu peaks=%lu "
                          "emitted=%lu slope=%lu interval=%lu",
                          static_cast<unsigned long>(t.acceptedStepCount),
                          static_cast<unsigned long>(t.rejectedStepCount),
                          static_cast<unsigned long>(st.peaks),
                          static_cast<unsigned long>(st.emitted),
                          static_cast<unsigned long>(st.rejectSlope),
                          static_cast<unsigned long>(st.rejectInterval));
        if (n > 0 && static_cast<std::size_t>(n) < sizeof(buf)) {
          enqueueUsbResponse(
              std::string_view(buf, static_cast<std::size_t>(n)));
        }
        break;
      }
      case CommandId::TuneStart: {
        const bool ok = _sessionManager.startTuning(platform::getTimeUs());
        enqueueUsbResponse(ok ? "OK" : "FAIL");
        break;
      }
      case CommandId::TuneStop: {
        const bool ok = _sessionManager.stopTuning(platform::getTimeUs());
        enqueueUsbResponse(ok ? "OK" : "FAIL");
        break;
      }
      case CommandId::Reset:
        _imuDropCount.store(0, std::memory_order_relaxed);
        _imuSeq.store(0, std::memory_order_relaxed);
        _imuEventCount.store(0, std::memory_order_relaxed);
        _stepDetector.resetDebugStats();
        (void)_sessionManager.stopTuning(platform::getTimeUs());
        (void)_sessionManager.stopSession(platform::getTimeUs());
        (void)_sessionManager.stopTuning(platform::getTimeUs());
        enqueueUsbResponse("OK");
        break;
      default:
        enqueueUsbResponse("INVALID_COMMAND");
        break;
      }

      _usbToSessionMail.free(appCmd);
    }

    if (!in) {
      continue;
    }

    step_detection::StepEvent evt{};
    evt.timestampUs = in->peakTimeUs;
    evt.confidence = static_cast<float>(in->confidence) / 255.0f;

    _sessionManager.onStep(evt);

    const bool active = _sessionManager.isActive();
    const uint8_t ledState = active ? 1u : 0u;
    _ledState.store(ledState, std::memory_order_relaxed);
    message_types::SessionNotification notice{};
    notice.sequence = in->sequence;
    notice.state = ledState;
    notice.stepCount = _sessionManager.getStepCount();
    notice.confidence = in->confidence;
    (void)notice;
    _sessionToLedFlags.set(Config::EVENT_LED_UPDATE);
    _stepToSessionMail.free(in);
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

void Application::usbCommandThread() {
  usb::UsbTransport transport;
  usb::UsbInterface iface(transport);

  iface.sendResponse("BOOT RTOS_THREADS");
  iface.printPrompt();

  char lineBuf[usb::USB_CMD_MAX_LEN];

  while (true) {
    message_types::UsbResponse *resp;
    while ((resp = _sessionToUsbMail.try_get())) {
      iface.sendResponse(resp->msg);
      _sessionToUsbMail.free(resp);
    }

    if (!iface.poll(lineBuf, sizeof(lineBuf))) {
      rtos::ThisThread::sleep_for(THREAD_SLEEP_INTERVAL);
      continue;
    }

    const usb::ParsedLine pl = usb::parseLine(std::string_view(lineBuf));
    if (std::optional<CommandId> cmd = parseCommand(pl)) {
      if (CommandId *mail = _usbToSessionMail.try_alloc()) {
        *mail = *cmd;
        _usbToSessionMail.put(mail);
      } else {
        _usbDropCount.fetch_add(1, std::memory_order_relaxed);
        iface.sendResponse("CMD_QUEUE_FULL");
      }
    } else {
      iface.sendResponse("UNKNOWN_COMMAND");
    }

    iface.printPrompt();
  }
}

void Application::ledManagerThread() {
  for (;;) {
    const uint32_t w = _sessionToLedFlags.wait_any_for(
        Config::EVENT_LED_UPDATE, rtos::Kernel::wait_for_u32_forever, true);
    if ((w & osFlagsError) != 0u) {
      continue;
    }
    (void)w;
    uint8_t ledState = _ledState.load(std::memory_order_relaxed);
    (void)ledState;
    // TODO: drive board LEDs via a future hal::ILed when wired.
  }
}

void Application::startThreads() noexcept {
  using mbed::callback;

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
}

[[noreturn]] void Application::run() noexcept {
  startThreads();

  while (true) {
    rtos::ThisThread::sleep_for(rtos::Kernel::wait_for_u32_forever);
  }
}

} // namespace app
