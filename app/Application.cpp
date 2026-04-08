/**
 * @file app/Application.cpp
 * @brief RTOS thread entry points.
 */

#include "app/Application.hpp"
#include "app/MailHandle.hpp"
#include "imu/Mpu6050Driver.hpp"
#include "mbed_assert.h"
#include "mbed_toolchain.h"
#include "platform/Callback.h"
#include "platform/Platform.hpp"
#include "platform/StringUtils.hpp"
#include "rtos/EventFlags.h"
#include "rtos/Kernel.h"
#include "rtos/Mail.h"
#include "rtos/ThisThread.h"
#include "rtos/Thread.h"
#include "usb/UsbInterface.hpp"
#include "usb/UsbTransport.hpp"
#include <atomic>
#include <cstring>
#include <string_view>

namespace app {

namespace {

constexpr uint32_t MAIL_DEPTH = 16u;

constexpr uint32_t EVENT_IMU_DATA_READY = 1u << 0;
constexpr uint32_t EVENT_LED_UPDATE = 1u << 1;

constexpr uint32_t STACK_SIZE_IMU_THREAD = 3072u;
constexpr uint32_t STACK_SIZE_SIGNAL_THREAD = 3072u;
constexpr uint32_t STACK_SIZE_STEP_THREAD = 2048u;
constexpr uint32_t STACK_SIZE_SESSION_THREAD = 2048u;
constexpr uint32_t STACK_SIZE_USB_THREAD = 4096u;
constexpr uint32_t STACK_SIZE_LED_THREAD = 1536u;

constexpr std::chrono::milliseconds SESSION_POLL_INTERVAL = 25ms;

constexpr std::size_t MAX_READ_RETRY_ATTEMPTS = 2u;
constexpr std::chrono::milliseconds READ_RETRY_BACKOFF_MS = 1ms;

/**
 * @struct ThreadStack
 * @brief Thread stack template.
 */
template <size_t Size> struct ThreadStack {
  alignas(std::max_align_t) unsigned char data[Size];
};

ThreadStack<STACK_SIZE_IMU_THREAD> g_stack_imu;
ThreadStack<STACK_SIZE_SIGNAL_THREAD> g_stack_signal;
ThreadStack<STACK_SIZE_STEP_THREAD> g_stack_step;
ThreadStack<STACK_SIZE_SESSION_THREAD> g_stack_session;
ThreadStack<STACK_SIZE_USB_THREAD> g_stack_usb;
ThreadStack<STACK_SIZE_LED_THREAD> g_stack_led;

rtos::EventFlags g_isrToSensorFlags{"imu_isr"};
rtos::EventFlags g_sessionToLedFlags{"led_evt"};

std::atomic<uint32_t> g_imuThreadErrorCount{0};

rtos::Mail<RawImuDataFrame, MAIL_DEPTH> g_sensorToSignalMail;
rtos::Mail<ProcessedImuDataFrame, MAIL_DEPTH> g_signalToStepMail;
rtos::Mail<StepDetectionEvent, MAIL_DEPTH> g_stepToSessionMail;
rtos::Mail<UsbCommand, MAIL_DEPTH> g_usbToSessionMail;

std::atomic<uint32_t> g_imuSeq{0};
static std::atomic<uint32_t> g_imuDropCount{0};
static std::atomic<uint32_t> g_signalDropCount{0};
static std::atomic<uint32_t> g_stepDropCount{0};
static std::atomic<uint32_t> g_usbDropCount{0};
static std::atomic<uint8_t> g_ledState{0};

rtos::Thread g_imuThread(osPriorityRealtime, STACK_SIZE_IMU_THREAD,
                         g_stack_imu.data, "imu_acq");
rtos::Thread g_signalThread(osPriorityHigh, STACK_SIZE_SIGNAL_THREAD,
                            g_stack_signal.data, "sig_proc");
rtos::Thread g_stepThread(osPriorityAboveNormal, STACK_SIZE_STEP_THREAD,
                          g_stack_step.data, "step_det");
rtos::Thread g_sessionThread(osPriorityBelowNormal, STACK_SIZE_SESSION_THREAD,
                             g_stack_session.data, "session");
rtos::Thread g_usbThread(osPriorityLow, STACK_SIZE_USB_THREAD, g_stack_usb.data,
                         "usb_cmd");
rtos::Thread g_ledThread(static_cast<osPriority>(2u), STACK_SIZE_LED_THREAD,
                         g_stack_led.data, "led_mgr");

/**
 * @brief Check if two string views are equal, ignoring case.
 * @param line The line to check.
 * @param cmd The command to check.
 * @return true if the string views are equal, ignoring case, false otherwise.
 */
bool commandEquals(std::string_view line, std::string_view cmd) noexcept {
  std::string_view rest = platform::str::trim(line);
  const std::string_view token = platform::str::nextToken(rest);
  return platform::str::iequals(token, cmd);
}

/**
 * @brief Try to send an IMU sample to the signal processing thread.
 * @param imuDriver The IMU driver.
 * @return true if the sample was sent, false otherwise.
 */
bool tryAcquireAndSendImuSample(imu::Mpu6050Driver &imuDriver) noexcept {
  MailHandle<RawImuDataFrame, MAIL_DEPTH> mailHandle{g_sensorToSignalMail};
  if (!mailHandle) [[unlikely]] {
    g_imuDropCount.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  imu::ImuSample sample{};
  platform::Result sampleResult{};

  for (std::size_t attempt = 0; attempt < MAX_READ_RETRY_ATTEMPTS; ++attempt) {
    sampleResult = imuDriver.readSample(sample);
    if (platform::isOk(sampleResult))
      break;

    rtos::ThisThread::sleep_for(READ_RETRY_BACKOFF_MS);
  }

  if (!platform::isOk(sampleResult)) [[unlikely]] {
    g_imuDropCount.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  RawImuDataFrame *msg = mailHandle.getMsg();
  msg->sequence = g_imuSeq.fetch_add(1, std::memory_order_acq_rel);
  const auto &[ax, ay, az] = sample.accel;
  msg->accelX = ax;
  msg->accelY = ay;
  msg->accelZ = az;
  msg->timestampUs = sample.timestampUs;

  g_sensorToSignalMail.put(mailHandle.releaseMsg());
  return true;
}

/**
 * @brief IMU acquisition thread.
 * @param imuDriver The IMU driver.
 */
void imuDataAcquisitionThread(imu::Mpu6050Driver &imuDriver) {
  while (true) {
    const uint32_t flags =
        g_isrToSensorFlags.wait_any(EVENT_IMU_DATA_READY, osWaitForever, true);

    if (flags & osFlagsError) {
      g_imuThreadErrorCount.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    tryAcquireAndSendImuSample(imuDriver);
  }
}

/**
 * @brief IMU data processing thread.
 */
void imuDataProcessingThread() {
  while (true) {
    RawImuDataFrame *rawIn = g_sensorToSignalMail.try_get_for(
        rtos::Kernel::Clock::duration_u32::max());
    if (!rawIn) {
      continue;
    }

    MailHandle<RawImuDataFrame, MAIL_DEPTH> in{g_sensorToSignalMail, rawIn};
    MailHandle<ProcessedImuDataFrame, MAIL_DEPTH> out{g_signalToStepMail};
    if (!out) [[unlikely]] {
      g_signalDropCount.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    ProcessedImuDataFrame *outMsg = out.getMsg();
    RawImuDataFrame *inMsg = in.getMsg();
    outMsg->sequence = inMsg->sequence;
    outMsg->sourceTimestampUs = inMsg->timestampUs;
    outMsg->accelMagnitudeMilliG = static_cast<int32_t>(inMsg->sequence * 1000);

    g_signalToStepMail.put(out.releaseMsg());
  }
}

void stepDetectionThread() {
  while (true) {
    ProcessedImuDataFrame *rawIn =
        g_signalToStepMail.try_get_for(rtos::Kernel::wait_for_u32_forever);
    if (!rawIn) {
      continue;
    }

    MailHandle<ProcessedImuDataFrame, MAIL_DEPTH> in{g_signalToStepMail, rawIn};
    MailHandle<StepDetectionEvent, MAIL_DEPTH> out{g_stepToSessionMail};
    if (!out) [[unlikely]] {
      g_stepDropCount.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    StepDetectionEvent *outMsg = out.getMsg();
    ProcessedImuDataFrame *inMsg = in.getMsg();
    outMsg->sequence = inMsg->sequence;
    outMsg->peakTimeUs = inMsg->sourceTimestampUs;
    outMsg->confidence = static_cast<uint8_t>(inMsg->sequence & 0xFFu);

    g_stepToSessionMail.put(out.releaseMsg());
  }
}

void sessionManagerThread() {
  uint32_t steps = 0u;

  while (true) {
    StepDetectionEvent *in =
        g_stepToSessionMail.try_get_for(SESSION_POLL_INTERVAL);

    UsbCommand *usbLine = nullptr;
    while ((usbLine = g_usbToSessionMail.try_get()) != nullptr) {
      const std::string_view trimmed = platform::str::trim(usbLine->line);
      if (commandEquals(trimmed, "PING")) {
        // Placeholder: future session commands parsed here.
      }
      g_usbToSessionMail.free(usbLine);
    }

    if (in == nullptr) {
      continue;
    }

    ++steps;
    SessionNotification notice{};
    notice.sequence = in->sequence;
    notice.state = 0;
    notice.stepCount = steps;
    (void)notice;
    (void)in->confidence;

    g_ledState.store(1, std::memory_order_relaxed);
    g_sessionToLedFlags.set(EVENT_LED_UPDATE);
    g_stepToSessionMail.free(in);
  }
}

void triggerImuDataAcquisitionFromIsr() noexcept {
  (void)g_isrToSensorFlags.set(EVENT_IMU_DATA_READY);
}

void usbCommandThread() {
  usb::UsbTransport transport;
  usb::UsbInterface iface(transport);

  iface.sendResponse("BOOT RTOS_THREADS");
  iface.printPrompt();

  char lineBuf[usb::USB_CMD_MAX_LEN];

  for (;;) {
    if (!iface.poll(lineBuf, sizeof(lineBuf))) {
      // Short sleep when idle: USB stack is polled, not interrupt-driven here.
      rtos::ThisThread::sleep_for(10ms);
      continue;
    }

    const std::string_view trimmed = platform::str::trim(lineBuf);

    if (commandEquals(trimmed, "TRIGGER_IMU_ISR")) {
      triggerImuDataAcquisitionFromIsr();
      iface.sendResponse("ISR_SIGNAL_QUEUED");
      iface.printPrompt();
      continue;
    }

    if (commandEquals(trimmed, "PING")) {
      iface.sendResponse("PONG");
      iface.printPrompt();
      continue;
    }

    UsbCommand *mail = g_usbToSessionMail.try_alloc();
    if (mail != nullptr) {
      std::memset(mail->line, 0, sizeof(mail->line));
      const size_t copyLen = (trimmed.size() < (sizeof(mail->line) - 1u))
                                 ? trimmed.size()
                                 : (sizeof(mail->line) - 1u);
      std::memcpy(mail->line, trimmed.data(), copyLen);
      mail->line[copyLen] = '\0';
      g_usbToSessionMail.put(mail);
    } else {
      g_usbDropCount.fetch_add(1, std::memory_order_relaxed);
    }

    iface.sendResponse("ACK");
    iface.printPrompt();

    if (commandEquals(trimmed, "STATUS")) {
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "DROP sensor=%lu signal=%lu step=%lu usb=%lu",
                    g_imuDropCount.load(), g_signalDropCount.load(),
                    g_stepDropCount.load(), g_usbDropCount.load());
      iface.sendResponse(buf);
      iface.printPrompt();
      continue;
    }
  }
}

void ledManagerThread() {
  for (;;) {
    const uint32_t w = g_sessionToLedFlags.wait_any_for(
        EVENT_LED_UPDATE, rtos::Kernel::wait_for_u32_forever, true);
    if ((w & osFlagsError) != 0u) {
      continue;
    }
    (void)w;
    uint8_t state = g_ledState.load(std::memory_order_relaxed);
    (void)state; // placeholder for future LED behavior
    // Placeholder: drive board LEDs via a future hal::ILed when wired.
  }
}

void startThreads() {
  using mbed::callback;

  MBED_ASSERT(g_imuThread.start(callback(imuDataAcquisitionThread)) == osOK);
  MBED_ASSERT(g_signalThread.start(callback(imuDataProcessingThread)) == osOK);
  MBED_ASSERT(g_stepThread.start(callback(stepDetectionThread)) == osOK);
  MBED_ASSERT(g_sessionThread.start(callback(sessionManagerThread)) == osOK);
  MBED_ASSERT(g_usbThread.start(callback(usbCommandThread)) == osOK);
  MBED_ASSERT(g_ledThread.start(callback(ledManagerThread)) == osOK);
}

} // anonymousnamespace

[[noreturn]] void run() noexcept {
  startThreads();

  for (;;) {
    rtos::ThisThread::sleep_for(rtos::Kernel::wait_for_u32_forever);
  }
}

} // namespace app
