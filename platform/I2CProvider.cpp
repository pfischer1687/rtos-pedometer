/**
 * @file platform/I2CProvider.cpp
 * @brief I2C provider implementation (STM32 I2C via Mbed).
 */

#include "platform/I2CProvider.hpp"

namespace platform {

namespace {

class I2CProvider final : public II2CProvider {
public:
  Result transfer(uint8_t devAddr, const uint8_t *txData, size_t txLen,
                  uint8_t *rxData, size_t rxLen,
                  uint32_t timeoutMs) noexcept override {
    return Result::Ok;
  }

  Result write(uint8_t devAddr, const uint8_t *data, size_t len,
               uint32_t timeoutMs) noexcept override {
    return Result::Ok;
  }

  Result read(uint8_t devAddr, uint8_t *data, size_t len,
              uint32_t timeoutMs) noexcept override {
    return Result::Ok;
  }
};

/**
 * @brief Static instance of I2C provider.
 */
I2CProvider g_i2c;

} // anonymous namespace

II2CProvider &i2c() noexcept { return g_i2c; }

} // namespace platform
