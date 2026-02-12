/**
 * @file platform/I2CProvider.cpp
 * @brief I2C provider implementation using Mbed I2C (Nucleo-F767ZI).
 */

#include "platform/I2CProvider.hpp"
#include "mbed.h"

namespace platform {

namespace {

/** Default I2C pins for Nucleo-F767ZI (I2C1: Arduino D14/D15). */
constexpr PinName I2C_SDA = PB_9;
constexpr PinName I2C_SCL = PB_8;
constexpr uint32_t I2C_FREQ_HZ = 400000;

Result fromMbedResult(mbed::I2C::Result r) noexcept
{
    switch (r) {
    case mbed::I2C::ACK:
        return Result::Ok;
    case mbed::I2C::NACK:
    case mbed::I2C::OTHER_ERROR:
        return Result::Error;
    case mbed::I2C::TIMEOUT:
        return Result::Timeout;
    default:
        return Result::Error;
    }
}

class I2CProviderImpl final : public II2CProvider {
public:
    I2CProviderImpl() : m_i2c(I2C_SDA, I2C_SCL)
    {
        m_i2c.frequency(I2C_FREQ_HZ);
    }

    Result transfer(uint8_t devAddr7, const uint8_t* txData, size_t txLen,
                    uint8_t* rxData, size_t rxLen,
                    uint32_t timeoutMs) noexcept override
    {
        (void)timeoutMs;
        const uint8_t addr8Write = static_cast<uint8_t>(devAddr7 << 1);
        const uint8_t addr8Read = static_cast<uint8_t>(addr8Write | 1u);

        if (txLen > 0u && txData != nullptr) {
            mbed::I2C::Result r = m_i2c.write(addr8Write, reinterpret_cast<const char*>(txData),
                                              static_cast<int>(txLen), true);
            if (r != mbed::I2C::ACK) {
                return fromMbedResult(r);
            }
        }
        if (rxLen > 0u && rxData != nullptr) {
            return fromMbedResult(m_i2c.read(addr8Read, reinterpret_cast<char*>(rxData),
                                             static_cast<int>(rxLen)));
        }
        return Result::Ok;
    }

    Result write(uint8_t devAddr7, const uint8_t* data, size_t len,
                 uint32_t timeoutMs) noexcept override
    {
        (void)timeoutMs;
        const uint8_t addr8 = static_cast<uint8_t>(devAddr7 << 1);
        if (len == 0u || data == nullptr) {
            return Result::Ok;
        }
        return fromMbedResult(m_i2c.write(addr8, reinterpret_cast<const char*>(data),
                                          static_cast<int>(len)));
    }

    Result read(uint8_t devAddr7, uint8_t* data, size_t len,
               uint32_t timeoutMs) noexcept override
    {
        (void)timeoutMs;
        const uint8_t addr8 = static_cast<uint8_t>((devAddr7 << 1) | 1u);
        if (len == 0u || data == nullptr) {
            return Result::Ok;
        }
        return fromMbedResult(m_i2c.read(addr8, reinterpret_cast<char*>(data),
                                         static_cast<int>(len)));
    }

private:
    mbed::I2C m_i2c;
};

I2CProviderImpl g_i2c;

} // anonymous namespace

II2CProvider& i2c() noexcept
{
    return g_i2c;
}

} // namespace platform
