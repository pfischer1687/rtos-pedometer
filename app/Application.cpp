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
#include "platform/StringUtils.hpp"
#include "rtos/EventFlags.h"
#include "rtos/Kernel.h"
#include "rtos/Mail.h"
#include "rtos/ThisThread.h"
#include "rtos/Thread.h"
#include "usb/Command.hpp"
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

constexpr std::size_t MAX_READ_RETRY_ATTEMPTS = 2u;
constexpr std::chrono::milliseconds READ_RETRY_BACKOFF = 1ms;
constexpr std::chrono::milliseconds THREAD_SLEEP_INTERVAL = 1ms;

/**
 * @brief First token of trimmed line equals cmd (ASCII case-insensitive).
 */
bool commandEquals(std::string_view line, std::string_view cmd) noexcept {
  std::string_view rest = platform::str::trim(line);
  const std::string_view token = platform::str::nextToken(rest);
  return platform::str::iequals(token, cmd);
}

} // namespace

Application::Application(imu::Mpu6050Driver &imu) noexcept
    : _imu(imu), _isrToSensorFlags("imu_isr"), _sessionToLedFlags("led_evt"),
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
      _ledThread(static_cast<osPriority>(2u), Config::LED_THREAD_STACK_DEPTH,
                 _stackLed.data, "led_mgr") {}

void Application::logImuEvent(ImuEventType type, platform::Result result,
                              uint32_t timestampUs) noexcept {
  const uint32_t writeIdx =
      _imuEventWriteIdx.fetch_add(1, std::memory_order_relaxed);
  const uint32_t idx = writeIdx % Config::IMU_EVENT_LOG_SIZE;

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
  platform::Result sampleResult{};

  for (std::size_t attempt = 0; attempt < MAX_READ_RETRY_ATTEMPTS; ++attempt) {
    sampleResult = _imu.readSample(sample);
    if (platform::isOk(sampleResult)) {
      break;
    }
    rtos::ThisThread::sleep_for(READ_RETRY_BACKOFF);
  }

  if (!platform::isOk(sampleResult)) [[unlikely]] {
    if (sampleResult == platform::Result::DataNotReady) {
      return;
    }

    _imuDropCount.fetch_add(1, std::memory_order_relaxed);
    logImuEvent(ImuEventType::ReadFail, sampleResult, sample.timestampUs);
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
    const uint32_t flags = _isrToSensorFlags.wait_any(
        Config::EVENT_IMU_DATA_READY, osWaitForever, true);

    if ((flags & osFlagsError) != 0u) {
      platform::Result res = platform::Result::Error;
      if ((flags & osFlagsErrorTimeout) != 0u) {
        res = platform::Result::Timeout;
      }

      logImuEvent(ImuEventType::Timeout, res, platform::getTimeUs());
      rtos::ThisThread::sleep_for(THREAD_SLEEP_INTERVAL);
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

void Application::sessionManagerThread() {
  uint32_t steps = 0u;

  while (true) {
    StepDetectionEvent *in =
        _stepToSessionMail.try_get_for(SESSION_POLL_INTERVAL);

    UsbCommand *usbLine;
    while ((usbLine = _usbToSessionMail.try_get())) {
      const std::string_view trimmed = platform::str::trim(usbLine->line);
      const usb::ParsedCommand parsed = usb::parseCommand(trimmed);
      if (parsed.id == usb::CommandId::Ping) {
        // TODO: parse session commands here.
      }
      _usbToSessionMail.free(usbLine);
    }

    if (!in) {
      continue;
    }

    ++steps;
    SessionNotification notice{};
    notice.sequence = in->sequence;
    notice.state = 0;
    notice.stepCount = steps;
    (void)notice;
    (void)in->confidence;

    _ledState.store(1, std::memory_order_relaxed);
    _sessionToLedFlags.set(Config::EVENT_LED_UPDATE);
    _stepToSessionMail.free(in);
  }
}

ImuHealthSnapshot Application::getImuHealthSnapshot() const noexcept {
  return ImuHealthSnapshot{
      .totalSamples = _imuSeq.load(std::memory_order_relaxed),
      .dropCount = _imuDropCount.load(std::memory_order_relaxed),
      .eventCount = _imuEventWriteIdx.load(std::memory_order_relaxed)};
}

void Application::usbCommandThread() {
  usb::UsbTransport transport;
  usb::UsbInterface iface(transport);

  iface.sendResponse("BOOT RTOS_THREADS");
  iface.printPrompt();

  char lineBuf[usb::USB_CMD_MAX_LEN];

  while (true) {
    if (!iface.poll(lineBuf, sizeof(lineBuf))) {
      rtos::ThisThread::sleep_for(THREAD_SLEEP_INTERVAL);
      continue;
    }

    const std::string_view trimmed = platform::str::trim(lineBuf);
    const usb::ParsedCommand parsed = usb::parseCommand(trimmed);

    if (commandEquals(trimmed, "TRIGGER_IMU_ISR")) {
      signalSensorAcquisitionFromIsr();
      iface.sendResponse("ISR_SIGNAL_QUEUED");
      iface.printPrompt();
      continue;
    }

    if (parsed.id == usb::CommandId::Ping) {
      iface.sendResponse("PONG");
      iface.printPrompt();
      continue;
    }

    UsbCommand *mail = _usbToSessionMail.try_alloc();
    if (mail) {
      std::memset(mail->line, 0, sizeof(mail->line));
      const size_t copyLen = (trimmed.size() < (sizeof(mail->line) - 1u))
                                 ? trimmed.size()
                                 : (sizeof(mail->line) - 1u);
      std::memcpy(mail->line, trimmed.data(), copyLen);
      mail->line[copyLen] = '\0';
      _usbToSessionMail.put(mail);
    } else {
      _usbDropCount.fetch_add(1, std::memory_order_relaxed);
    }

    iface.sendResponse("ACK");
    iface.printPrompt();

    if (parsed.id == usb::CommandId::Status) {
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "DROP sensor=%lu signal=%lu step=%lu usb=%lu",
                    static_cast<unsigned long>(_imuDropCount.load()),
                    static_cast<unsigned long>(_signalDropCount.load()),
                    static_cast<unsigned long>(_stepDropCount.load()),
                    static_cast<unsigned long>(_usbDropCount.load()));
      iface.sendResponse(buf);
      iface.printPrompt();
      continue;
    }

    if (commandEquals(trimmed, "DUMP_IMU")) {

      const auto health = getImuHealthSnapshot();

      char buf[128];
      std::snprintf(buf, sizeof(buf), "IMU total=%lu drops=%lu events=%lu",
                    (unsigned long)health.totalSamples,
                    (unsigned long)health.dropCount,
                    (unsigned long)health.eventCount);

      iface.sendResponse(buf);

      const uint32_t write = _imuEventWriteIdx.load(std::memory_order_acquire);
      const uint32_t count = (write < Config::IMU_EVENT_LOG_SIZE)
                                 ? write
                                 : Config::IMU_EVENT_LOG_SIZE;

      for (uint32_t i = 0; i < count; ++i) {
        // Calculating index to write them in chronological order based on ring
        // buffer.
        const uint32_t idx = (write - count + i) % Config::IMU_EVENT_LOG_SIZE;

        const ImuEvent e = _imuEventLog[idx];
        std::snprintf(buf, sizeof(buf), "E t=%lu type=%u res=%u",
                      (unsigned long)e.timestampUs,
                      static_cast<unsigned>(e.type),
                      static_cast<unsigned>(e.result));

        iface.sendResponse(buf);
      }

      iface.printPrompt();
      continue;
    }

    if (commandEquals(trimmed, "RESET_IMU_STATS")) {
      _imuDropCount.store(0);
      _imuSeq.store(0);
      _imuEventWriteIdx.store(0);
      iface.sendResponse("OK");
    }
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

void Application::signalSensorAcquisitionFromIsr() noexcept {
  (void)_isrToSensorFlags.set(Config::EVENT_IMU_DATA_READY);
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
