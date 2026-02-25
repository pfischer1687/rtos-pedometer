/**
 * @file platform/I2CProvider.cpp
 * @brief I2C provider implementation using Mbed I2C (Nucleo-F767ZI).
 */

#include "platform/I2CProvider.hpp"
#include "mbed.h"

namespace platform {

namespace {

/**
 * @brief MPU-6050/STM32F767ZIT6 Fast-mode frequency: 400 kHz.
 */
constexpr uint32_t I2C_FREQ_HZ = 400'000u;

/**
 * @brief Make 8-bit write address from 7-bit I2C address.
 * @param addr7bit 7-bit address.
 * @return 8-bit write address.
 */
constexpr int getWriteAddr8Bit(uint8_t addr7bit) noexcept {
  return static_cast<int>(addr7bit << 1u);
}

/**
 * @brief Make 8-bit read address from 7-bit I2C address.
 * @param addr7bit 7-bit address.
 * @return 8-bit read address.
 */
constexpr int getReadAddr8Bit(uint8_t addr7bit) noexcept {
  return static_cast<int>((addr7bit << 1u) | 0x01u);
}

/**
 * @brief Convert Mbed I2C result to platform result.
 * @param r Mbed I2C result.
 * @return Result.
 */
Result fromMbedResult(mbed::I2C::Result r) noexcept {
  if (r == mbed::I2C::ACK)
    return Result::Ok;

  if (r == mbed::I2C::TIMEOUT)
    return Result::Timeout;

  return Result::Error;
}

} // anonymous namespace

class I2CProviderImpl final : public II2CProvider {
public:
  I2CProviderImpl() : m_i2c(I2C_SDA, I2C_SCL) { m_i2c.frequency(I2C_FREQ_HZ); }

  Result write(uint8_t addr7bit, const uint8_t *data, size_t len,
               bool isRepeatedStart) noexcept override {
    if (len == 0u) {
      return Result::Ok;
    }
    if (data == nullptr) {
      return Result::InvalidArgument;
    }

    return fromMbedResult(m_i2c.write(getWriteAddr8Bit(addr7bit),
                                      reinterpret_cast<const char *>(data),
                                      static_cast<int>(len), isRepeatedStart));
  }

  Result read(uint8_t addr7bit, uint8_t *data, size_t len) noexcept override {
    if (len == 0u) {
      return Result::Ok;
    }
    if (data == nullptr) {
      return Result::InvalidArgument;
    }

    return fromMbedResult(m_i2c.read(getReadAddr8Bit(addr7bit),
                                     reinterpret_cast<char *>(data),
                                     static_cast<int>(len)));
  }

  Result transfer(uint8_t addr7bit, const uint8_t *txData, size_t txLen,
                  uint8_t *rxData, size_t rxLen) noexcept override {
    if ((txLen > 0 && txData == nullptr) || (rxLen > 0 && rxData == nullptr)) {
      return Result::InvalidArgument;
    }

    bool isRepeatedStart = (txLen > 0 && rxLen > 0);

    if (txLen > 0) {
      platform::Result writeResult =
          write(addr7bit, txData, txLen, isRepeatedStart);
      if (!platform::isOk(writeResult)) {
        return writeResult;
      }
    }

    if (rxLen > 0) {
      platform::Result readResult = read(addr7bit, rxData, rxLen);
      if (!platform::isOk(readResult)) {
        return readResult;
      }
    }

    return Result::Ok;
  }

private:
  mbed::I2C m_i2c;
};

II2CProvider &i2c() noexcept {
  static I2CProviderImpl instance;
  return instance;
}

} // namespace platform
