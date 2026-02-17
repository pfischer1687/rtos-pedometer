/**
 * @file platform/Gpio.cpp
 * @brief GPIO implementation: InterruptIn for data-ready, DigitalOut for LED
 * (Nucleo-F767ZI).
 */

#include "platform/Gpio.hpp"
#include "mbed.h"

namespace platform {

namespace {

/**
 * @brief MPU-6050 INT (data-ready) pin must be connected to GPIO pin PA_0 on
 * the Nucleo-F767ZI board.
 */
constexpr PinName DATA_READY_PIN =
    PA_0; // EXTI0[3:0] bits in the SYSCFG_EXTICR1 register

/**
 * @brief Nucleo-F767ZI LD3 (red) LED.
 */
constexpr PinName LED_PIN = LED3;

/**
 * @class DataReadyInputImpl
 * @brief Data-ready input implementation.
 */
class DataReadyInputImpl final : public IDataReadyInput {
public:
  DataReadyInputImpl() : m_pin(DATA_READY_PIN), m_callback(nullptr) {
    m_pin.mode(PullNone);
  }

  void setCallback(DataReadyCallback cb) noexcept override {
    m_callback = cb;
    if (cb != nullptr) {
      m_pin.rise(mbed::callback(this, &DataReadyInputImpl::isrRise));
    } else {
      m_pin.rise(nullptr);
    }
  }

  void enable() noexcept override { m_pin.enable_irq(); }

  void disable() noexcept override { m_pin.disable_irq(); }

private:
  /**
   * @brief Interrupt service routine for rising edge of data-ready signal.
   */
  void isrRise() {
    if (m_callback != nullptr) {
      m_callback();
    }
  }

  mbed::InterruptIn m_pin;
  volatile DataReadyCallback m_callback;
};

class LedOutputImpl final : public ILedOutput {
public:
  void set(bool on) noexcept override { m_led.write(on ? 1 : 0); }

private:
  mbed::DigitalOut m_led{LED_PIN};
};

DataReadyInputImpl g_dataReadyInput;
LedOutputImpl g_ledOutput;

} // anonymous namespace

/**
 * @brief Get the data-ready input (MPU-6050 INT pin).
 * @return Data-ready input.
 */
IDataReadyInput &dataReadyInput() noexcept { return g_dataReadyInput; }

/**
 * @brief Get the LED output (status indicator).
 * @return LED output.
 */
ILedOutput &ledOutput() noexcept { return g_ledOutput; }

} // namespace platform
