/** \file platform/I2CProvider.hpp
 *  I2C bus abstraction for drivers (e.g. MPU-6050).
 *  Ownership: platform provides the bus; drivers use it.
 */

#ifndef PLATFORM_I2CPROVIDER_HPP
#define PLATFORM_I2CPROVIDER_HPP

#include "platform/Platform.hpp"
#include <cstddef>
#include <cstdint>

namespace platform {

/** Abstract I2C transaction interface. Implemented by platform for target hardware. */
class I2CProvider {
public:
    virtual ~I2CProvider() = default;

    /** Write then read in one transaction. Returns Ok on success. */
    virtual Result transfer(uint8_t devAddr, const uint8_t* txData, size_t txLen,
                            uint8_t* rxData, size_t rxLen) = 0;

    /** Write only. */
    virtual Result write(uint8_t devAddr, const uint8_t* data, size_t len) = 0;

    /** Read only. */
    virtual Result read(uint8_t devAddr, uint8_t* data, size_t len) = 0;
};

} // namespace platform

#endif /* PLATFORM_I2CPROVIDER_HPP */
