/**
 * @file platform/Gpio.hpp
 * @brief GPIO abstraction: input interrupt (e.g. MPU-6050 data-ready) and LED
 * output.
 * @details
 * - Data-ready ISR must only invoke a user callback (e.g. set RTOS flag); no
 * blocking or allocation.
 * - LED is a simple digital output for status indication.
 */

#ifndef PLATFORM_GPIO_HPP
#define PLATFORM_GPIO_HPP

namespace platform {

/**
 * @typedef DataReadyCallback
 * @brief Callback type for data-ready interrupt.
 */
using DataReadyCallback = void (*)(void *);

/**
 * @interface IDataReadyInput
 * @brief Abstract input for IMU data-ready interrupt.
 */
class IDataReadyInput {
public:
  virtual ~IDataReadyInput() = default;

  /**
   * @brief Set callback invoked on data-ready (rising edge).
   * @param cb Callback function.
   * @param ctx Context pointer.
   */
  virtual void setCallback(DataReadyCallback cb, void *ctx) noexcept = 0;

  /**
   * @brief Enable the data-ready interrupt.
   */
  virtual void enable() noexcept = 0;

  /**
   * @brief Disable the data-ready interrupt.
   */
  virtual void disable() noexcept = 0;
};

/**
 * @interface ILedOutput
 * @brief Abstract LED output for status indication.
 */
class ILedOutput {
public:
  virtual ~ILedOutput() = default;

  /**
   * @brief Set LED on (true) or off (false).
   * @param on true = on, false = off.
   */
  virtual void set(bool on) noexcept = 0;
};

/**
 * @brief Get the data-ready input (MPU-6050 INT pin).
 */
IDataReadyInput &dataReadyInput() noexcept;

/**
 * @brief Get the LED output (status indicator).
 */
ILedOutput &ledOutput() noexcept;

} // namespace platform

#endif /* PLATFORM_GPIO_HPP */
