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
constexpr uint8_t STEP_CONF_THRESHOLD = 128u;

} // namespace

Application::Application(imu::Mpu6050Driver &imu) noexcept
    : _imu(imu), _sessionToLedFlags("led_evt"),
      _imuThread(osPriorityRealtime, Config::IMU_THREAD_STACK_DEPTH,
                 _stackImu.data, "imu_acq"),
      _signalThread(osPriorityHigh, Config::SIGNAL_THREAD_STACK_DEPTH,
                    _stackSignal.data, "sig_proc"),
      _stepThread(osPriorityAboveNormal, Config::STEP_THREAD_STACK_DEPTH,
                  _stackStep.data, "step_det"),
      _sessionThread(osPriorityBelowNormal, Config::SESSION_THREAD_STACK_DEPTH,
                     _stackSession.data, "session"),
      _usbThread(osPriorityLow, Config::USB_THREAD_STACK_DEPTH, _stackUsb.data,
                 "usb_cmd"),
      _ledThread(osPriorityLow, Config::LED_THREAD_STACK_DEPTH, _stackLed.data,
                 "led_mgr") {}

void Application::logImuEvent(ImuEventType type, platform::Result result,
                              uint32_t timestampUs) noexcept {
  const uint32_t count = _imuEventCount.fetch_add(1, std::memory_order_relaxed);
  const uint32_t idx = count % Config::IMU_EVENT_LOG_SIZE;
  _imuEventLog[idx] =
      ImuEvent{.timestampUs = timestampUs, .type = type, .result = result};
}

void Application::tryAcquireAndSendImuSample() noexcept {
  MailHandle<RawImuDataFrame, Config::MAIL_DEPTH> mailHandle{
      _sensorToSignalMail};
  if (!mailHandle) [[unlikely]] {
    _imuDropCount.fetch_add(1, std::memory_order_relaxed);
    logImuEvent(ImuEventType::MailAllocFail, platform::Result::Busy,
                platform::getTimeUs());
    return;
  }

  imu::ImuSample sample{};
  platform::Result sampleResult = _imu.readSample(sample);
  if (!platform::isOk(sampleResult)) [[unlikely]] {
    if (sampleResult == platform::Result::DataNotReady) {
      return;
    }

    _imuDropCount.fetch_add(1, std::memory_order_relaxed);
    logImuEvent(ImuEventType::ReadFail, sampleResult, platform::getTimeUs());
    return;
  }

  RawImuDataFrame *msg = mailHandle.getMsg();
  msg->sequence = _imuSeq.fetch_add(1, std::memory_order_relaxed);
  const auto &[ax, ay, az] = sample.accel;
  msg->accelX = ax;
  msg->accelY = ay;
  msg->accelZ = az;
  msg->timestampUs = sample.timestampUs;

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
  while (true) {
    RawImuDataFrame *rawIn = _sensorToSignalMail.try_get_for(
        rtos::Kernel::Clock::duration_u32::max());
    if (!rawIn) {
      continue;
    }

    MailHandle<RawImuDataFrame, Config::MAIL_DEPTH> in{_sensorToSignalMail,
                                                       rawIn};
    MailHandle<ProcessedImuDataFrame, Config::MAIL_DEPTH> out{
        _signalToStepMail};
    if (!out) [[unlikely]] {
      _signalDropCount.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    ProcessedImuDataFrame *outMsg = out.getMsg();
    RawImuDataFrame *inMsg = in.getMsg();
    outMsg->sequence = inMsg->sequence;
    outMsg->sourceTimestampUs = inMsg->timestampUs;

    // TODO: calculate accel magnitude milliG.
    outMsg->accelMagnitudeMilliG = static_cast<int32_t>(inMsg->sequence * 1000);

    _signalToStepMail.put(out.releaseMsg());
  }
}

void Application::stepDetectionThread() {
  while (true) {
    ProcessedImuDataFrame *rawIn =
        _signalToStepMail.try_get_for(rtos::Kernel::wait_for_u32_forever);
    if (!rawIn) {
      continue;
    }

    MailHandle<ProcessedImuDataFrame, Config::MAIL_DEPTH> in{_signalToStepMail,
                                                             rawIn};
    MailHandle<StepDetectionEvent, Config::MAIL_DEPTH> out{_stepToSessionMail};
    if (!out) [[unlikely]] {
      _stepDropCount.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    StepDetectionEvent *outMsg = out.getMsg();
    ProcessedImuDataFrame *inMsg = in.getMsg();
    outMsg->sequence = inMsg->sequence;
    outMsg->peakTimeUs = inMsg->sourceTimestampUs;

    // TODO: calculate step confidence.
    outMsg->confidence = static_cast<uint8_t>(inMsg->sequence & 0xFFu);

    _stepToSessionMail.put(out.releaseMsg());
  }
}

void Application::enqueueUsbResponse(std::string_view text) noexcept {
  UsbResponse *slot = _sessionToUsbMail.try_alloc();
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
  uint32_t steps = 0u;
  bool sessionRecording = false;

  while (true) {
    StepDetectionEvent *in =
        _stepToSessionMail.try_get_for(SESSION_POLL_INTERVAL);

    CommandId *appCmd;
    while ((appCmd = _usbToSessionMail.try_get())) {
      // TODO: handle session commands here.
      switch (*appCmd) {
      case CommandId::Start:
        sessionRecording = true;
        enqueueUsbResponse("OK");
        break;
      case CommandId::Stop:
        sessionRecording = false;
        enqueueUsbResponse("OK");
        break;
      case CommandId::Status: {
        char buf[sizeof(UsbResponse::msg)];
        const int n = std::snprintf(
            buf, sizeof(buf), "DROP sensor=%lu signal=%lu step=%lu usb=%lu",
            static_cast<unsigned long>(
                _imuDropCount.load(std::memory_order_relaxed)),
            static_cast<unsigned long>(
                _signalDropCount.load(std::memory_order_relaxed)),
            static_cast<unsigned long>(
                _stepDropCount.load(std::memory_order_relaxed)),
            static_cast<unsigned long>(
                _usbDropCount.load(std::memory_order_relaxed)));
        if (n > 0 && static_cast<std::size_t>(n) < sizeof(buf)) {
          enqueueUsbResponse(
              std::string_view(buf, static_cast<std::size_t>(n)));
        }
        break;
      }
      case CommandId::Reset:
        _imuDropCount.store(0, std::memory_order_relaxed);
        _imuSeq.store(0, std::memory_order_relaxed);
        _imuEventCount.store(0, std::memory_order_relaxed);
        steps = 0u;
        sessionRecording = false;
        enqueueUsbResponse("OK");
        break;
      }

      _usbToSessionMail.free(appCmd);
    }

    if (!in) {
      continue;
    }

    if (in->confidence >= STEP_CONF_THRESHOLD) {
      ++steps;
    }

    SessionNotification notice{};
    notice.sequence = in->sequence;
    notice.state = sessionRecording ? 1u : 0u;
    notice.stepCount = steps;
    (void)notice;
    (void)in->confidence;

    _ledState.store(1, std::memory_order_relaxed);
    _sessionToLedFlags.set(Config::EVENT_LED_UPDATE);
    _stepToSessionMail.free(in);
  }
}

ImuHealthSnapshot Application::getImuHealthSnapshot() const noexcept {
  const uint32_t seq = _imuSeq.load(std::memory_order_relaxed);
  const uint32_t drop = _imuDropCount.load(std::memory_order_relaxed);
  const uint32_t events = _imuEventCount.load(std::memory_order_relaxed);

  const uint32_t buffered = (events < Config::IMU_EVENT_LOG_SIZE)
                                ? events
                                : Config::IMU_EVENT_LOG_SIZE;

  return ImuHealthSnapshot{.totalSamples = seq,
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
    UsbResponse *resp;
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
    uint8_t state = _ledState.load(std::memory_order_relaxed);
    (void)state;
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
