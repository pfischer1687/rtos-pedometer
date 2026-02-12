/**
 * @file platform/Gpio.cpp
 * @brief GPIO implementation: InterruptIn for data-ready, DigitalOut for LED (Nucleo-F767ZI).
 */

#include "platform/Gpio.hpp"
#include "mbed.h"

namespace platform {

namespace {

/** MPU-6050 INT (data-ready) pin. Connect INT to this GPIO. */
constexpr PinName DATA_READY_PIN = PA_0;
/** Nucleo-F767ZI LD1 (green) status LED. */
constexpr PinName LED_PIN = PB_0;

class DataReadyInputImpl final : public IDataReadyInput {
public:
    DataReadyInputImpl() : m_pin(DATA_READY_PIN), m_callback(nullptr)
    {
        m_pin.mode(PullNone);
    }

    void setCallback(DataReadyCallback cb) noexcept override
    {
        m_callback = cb;
        if (cb != nullptr) {
            m_pin.rise(mbed::callback(this, &DataReadyInputImpl::isrRise));
        } else {
            m_pin.rise(nullptr);
        }
    }

    void enable() noexcept override
    {
        m_pin.enable_irq();
    }

    void disable() noexcept override
    {
        m_pin.disable_irq();
    }

private:
    void isrRise()
    {
        if (m_callback != nullptr) {
            m_callback();
        }
    }

    mbed::InterruptIn m_pin;
    volatile DataReadyCallback m_callback;
};

class LedOutputImpl final : public ILedOutput {
public:
    void set(bool on) noexcept override
    {
        m_led.write(on ? 1 : 0);
    }

private:
    mbed::DigitalOut m_led{ LED_PIN };
};

DataReadyInputImpl g_dataReadyInput;
LedOutputImpl g_ledOutput;

} // anonymous namespace

IDataReadyInput& dataReadyInput() noexcept
{
    return g_dataReadyInput;
}

ILedOutput& ledOutput() noexcept
{
    return g_ledOutput;
}

} // namespace platform
