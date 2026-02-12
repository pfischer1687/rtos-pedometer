/**
 * @file platform/I2CProvider.hpp
 * @brief I2C bus abstraction for drivers (e.g. MPU-6050).
 */

#ifndef PLATFORM_I2CPROVIDER_HPP
#define PLATFORM_I2CPROVIDER_HPP

#include "platform/Platform.hpp"
#include <cstddef>
#include <cstdint>

namespace platform {

/**
 * @class II2CProvider
 * @brief Abstract I2C transaction interface. Implemented by platform for target
 * hardware.
 */
class II2CProvider {
public:
  virtual ~II2CProvider() = default;

  /**
   * @brief Write then read in one transaction. Returns Ok on success.
   * @param devAddr Device address.
   * @param txData Transmit data.
   * @param txLen Length of transmit data.
   * @param rxData Receive data.
   * @param rxLen Length of receive data.
   * @param timeoutMs Timeout in milliseconds.
   * @return Result.
   */
  virtual Result transfer(uint8_t devAddr, const uint8_t *txData, size_t txLen,
                          uint8_t *rxData, size_t rxLen,
                          uint32_t timeoutMs) noexcept = 0;

  /**
   * @brief Write only.
   * @param devAddr Device address.
   * @param data Data to write.
   * @param len Length of data to write.
   * @param timeoutMs Timeout in milliseconds.
   * @return Result.
   */
  virtual Result write(uint8_t devAddr, const uint8_t *data, size_t len,
                       uint32_t timeoutMs) noexcept = 0;

  /**
   * @brief Read only.
   * @param devAddr Device address.
   * @param data Data to read.
   * @param len Length of data to read.
   * @param timeoutMs Timeout in milliseconds.
   * @return Result.
   */
  virtual Result read(uint8_t devAddr, uint8_t *data, size_t len,
                      uint32_t timeoutMs) noexcept = 0;
};

/**
 * @brief Get the I2C provider.
 * @return I2C provider.
 */
II2CProvider &i2c() noexcept;

} // namespace platform

#endif /* PLATFORM_I2CPROVIDER_HPP */
