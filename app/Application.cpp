/**
 * @file app/Application.cpp
 * @brief RTOS thread entry points.
 */

#include "app/Application.hpp"
#include "mbed_assert.h"
#include "mbed_toolchain.h"
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
#include <cctype>
#include <cstring>

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

void trimInPlace(char *s) noexcept {
  if (s == nullptr || s[0] == '\0') {
    return;
  }
  char *end = s + std::strlen(s);
  while (end > s && (end[-1] == '\r' || end[-1] == '\n' || end[-1] == ' ')) {
    *--end = '\0';
  }
  char *p = s;
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  if (p != s) {
    std::memmove(s, p, static_cast<size_t>(end - p) + 1u);
  }
}

bool commandEquals(const char *line, const char *cmd) noexcept {
  if (line == nullptr || cmd == nullptr) {
    return false;
  }
  while (*cmd != '\0') {
    char a = *line++;
    char b = *cmd++;
    if (a >= 'a' && a <= 'z') {
      a = static_cast<char>(a - ('a' - 'A'));
    }
    if (b >= 'a' && b <= 'z') {
      b = static_cast<char>(b - ('a' - 'A'));
    }
    if (a != b) {
      return false;
    }
  }
  return *line == '\0' || *line == ' ';
}

void sensorAcquisitionThread() {
  for (;;) {
    const uint32_t w =
        g_isrToSensorFlags.wait_any(kEvImuDataReady, osWaitForever, true);
    // osFlagsError in high bits: retry wait (e.g. spurious wake with error
    // code).
    if ((w & osFlagsError) != 0u) {
      continue;
    }

    // Backpressure policy: drop newest sample if mail pool is exhausted.
    // Drop counters track overload conditions for debugging/telemetry.
    SensorFrameMsg *msg = g_sensorToSignalMail.try_alloc();
    if (msg == nullptr) {
      g_sensorDropCount.fetch_add(1, std::memory_order_relaxed);
      continue;
    }

    msg->sequence = g_sensorSeq.fetch_add(1u, std::memory_order_relaxed) + 1u;
    msg->timestampUs = platform::timer().nowUs();
    msg->accelX = 0;
    msg->accelY = 0;
    msg->accelZ = 0;

    g_sensorToSignalMail.put(msg);
  }
}

void signalProcessingThread() {
  for (;;) {
    SensorFrameMsg *in = g_sensorToSignalMail.try_get_for(
        rtos::Kernel::Clock::duration_u32::max());
    if (in == nullptr) {
      continue;
    }

    ProcessedSignalMsg *out = g_signalToStepMail.try_alloc();
    if (out == nullptr) {
      g_signalDropCount.fetch_add(1, std::memory_order_relaxed);
      g_sensorToSignalMail.free(in);
      continue;
    }

    out->sequence = in->sequence;
    out->sourceTimestampUs = in->timestampUs;
    out->magnitudeMilliPlaceholder = static_cast<int32_t>(in->sequence * 1000);

    g_sensorToSignalMail.free(in);
    g_signalToStepMail.put(out);
  }
}

void stepDetectionThread() {
  for (;;) {
    ProcessedSignalMsg *in =
        g_signalToStepMail.try_get_for(rtos::Kernel::wait_for_u32_forever);
    if (in == nullptr) {
      continue;
    }

    StepEventMsg *out = g_stepToSessionMail.try_alloc();
    if (out == nullptr) {
      g_stepDropCount.fetch_add(1, std::memory_order_relaxed);
      g_signalToStepMail.free(in);
      continue;
    }

    out->sequence = in->sequence;
    out->peakTimeUs = in->sourceTimestampUs;
    out->confidencePlaceholder = static_cast<uint8_t>(in->sequence & 0xFFu);

    g_signalToStepMail.free(in);
    g_stepToSessionMail.put(out);
  }
}

void sessionManagerThread() {
  uint32_t steps = 0u;

  for (;;) {
    StepEventMsg *in = g_stepToSessionMail.try_get_for(kSessionPollInterval);

    UsbCommandMsg *usbLine = nullptr;
    while ((usbLine = g_usbToSessionMail.try_get()) != nullptr) {
      trimInPlace(usbLine->line);
      if (commandEquals(usbLine->line, "PING")) {
        // Placeholder: future session commands parsed here.
      }
      g_usbToSessionMail.free(usbLine);
    }

    if (in == nullptr) {
      continue;
    }

    ++steps;
    SessionNoticeMsg notice{};
    notice.sequence = in->sequence;
    notice.statePlaceholder = 0;
    notice.stepCountPlaceholder = steps;
    (void)notice;
    (void)in->confidencePlaceholder;

    g_ledState.store(1, std::memory_order_relaxed);
    g_sessionToLedFlags.set(kEvLedUpdate);
    g_stepToSessionMail.free(in);
  }
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

    trimInPlace(lineBuf);

    if (commandEquals(lineBuf, "TRIGGER_IMU_ISR")) {
      signalSensorAcquisitionFromIsr();
      iface.sendResponse("ISR_SIGNAL_QUEUED");
      iface.printPrompt();
      continue;
    }

    if (commandEquals(lineBuf, "PING")) {
      iface.sendResponse("PONG");
      iface.printPrompt();
      continue;
    }

    UsbCommandMsg *mail = g_usbToSessionMail.try_alloc();
    if (mail != nullptr) {
      std::memset(mail->line, 0, sizeof(mail->line));
      std::strncpy(mail->line, lineBuf, sizeof(mail->line) - 1u);
      g_usbToSessionMail.put(mail);
    } else {
      g_usbDropCount.fetch_add(1, std::memory_order_relaxed);
    }

    iface.sendResponse("ACK");
    iface.printPrompt();

    if (commandEquals(lineBuf, "STATUS")) {
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "DROP sensor=%lu signal=%lu step=%lu usb=%lu",
                    g_sensorDropCount.load(), g_signalDropCount.load(),
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
        kEvLedUpdate, rtos::Kernel::wait_for_u32_forever, true);
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

  MBED_ASSERT(g_thSensor.start(callback(sensorAcquisitionThread)) == osOK);
  MBED_ASSERT(g_thSignal.start(callback(signalProcessingThread)) == osOK);
  MBED_ASSERT(g_thStep.start(callback(stepDetectionThread)) == osOK);
  MBED_ASSERT(g_thSession.start(callback(sessionManagerThread)) == osOK);
  MBED_ASSERT(g_thUsb.start(callback(usbCommandThread)) == osOK);
  MBED_ASSERT(g_thLed.start(callback(ledManagerThread)) == osOK);
}

} // anonymousnamespace

void signalSensorAcquisitionFromIsr() noexcept {
  (void)g_isrToSensorFlags.set(kEvImuDataReady);
}

[[noreturn]] void run() noexcept {
  startThreads();

  for (;;) {
    rtos::ThisThread::sleep_for(rtos::Kernel::wait_for_u32_forever);
  }
}

} // namespace app
