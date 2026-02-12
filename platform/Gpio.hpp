/**
 * @file platform/Gpio.hpp
 * @brief GPIO abstraction: input interrupt (e.g. MPU-6050 data-ready) and LED output.
 * @details
 * - Data-ready ISR must only invoke a user callback (e.g. set RTOS flag); no blocking or allocation.
 * - LED is a simple digital output for status indication.
 */

#ifndef PLATFORM_GPIO_HPP
#define PLATFORM_GPIO_HPP

#include "platform/Platform.hpp"

namespace platform {

/**
 * @brief Callback type for data-ready interrupt. Invoked from ISR context; must be minimal.
 */
using DataReadyCallback = void (*)(void);

/**
 * @interface IDataReadyInput
 * @brief Abstract input for IMU data-ready interrupt. App sets callback; ISR only calls it.
 */
class IDataReadyInput {
public:
    virtual ~IDataReadyInput() = default;

    /** Set callback invoked on data-ready (rising edge). Call from thread context only. */
    virtual void setCallback(DataReadyCallback cb) noexcept = 0;

    /** Enable the data-ready interrupt. */
    virtual void enable() noexcept = 0;

    /** Disable the data-ready interrupt. */
    virtual void disable() noexcept = 0;
};

/**
 * @interface ILedOutput
 * @brief Abstract LED output for status indication.
 */
class ILedOutput {
public:
    virtual ~ILedOutput() = default;

    /** Set LED on (true) or off (false). */
    virtual void set(bool on) noexcept = 0;
};

/**
 * @brief Get the data-ready input (MPU-6050 INT pin).
 */
IDataReadyInput& dataReadyInput() noexcept;

/**
 * @brief Get the LED output (status indicator).
 */
ILedOutput& ledOutput() noexcept;

} // namespace platform

#endif /* PLATFORM_GPIO_HPP */
