/**
 * @file test/imu/Mpu6050DriverTests.cpp
 * @brief Host-based unit tests for Mpu6050Driver.
 */

#include "imu/Mpu6050Driver.hpp"
#include "platform/I2CProvider.hpp"
#include "platform/Platform.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

struct I2CWriteRecord {
  uint8_t addr7bit{0};
  std::vector<uint8_t> data;
};

struct I2CTransferRecord {
  uint8_t addr7bit{0};
  std::vector<uint8_t> txData;
  std::vector<uint8_t> rxData;  // data injected into rx by mock
};

class MockI2CProvider final : public platform::II2CProvider {
public:
  platform::Result transferResult{platform::Result::Ok};
  std::vector<uint8_t> transferRxInject;  // copied into rxData when rxLen > 0

  std::vector<I2CWriteRecord> writeCalls;
  std::vector<I2CTransferRecord> transferCalls;

  void reset()
  {
    writeCalls.clear();
    transferCalls.clear();
    transferResult = platform::Result::Ok;
    transferRxInject.clear();
  }

  platform::Result transfer(uint8_t addr7bit, const uint8_t* txData, size_t txLen,
                           uint8_t* rxData, size_t rxLen) noexcept override
  {
    I2CTransferRecord rec;
    rec.addr7bit = addr7bit;
    if (txData && txLen > 0) {
      rec.txData.assign(txData, txData + txLen);
    }
    if (rxData && rxLen > 0 && transferRxInject.size() >= rxLen) {
      std::memcpy(rxData, transferRxInject.data(), rxLen);
    }
    transferCalls.push_back(rec);
    return transferResult;
  }

  platform::Result write(uint8_t addr7bit, const uint8_t* data,
                         size_t len) noexcept override
  {
    I2CWriteRecord rec;
    rec.addr7bit = addr7bit;
    if (data && len > 0) {
      rec.data.assign(data, data + len);
    }
    writeCalls.push_back(rec);
    return platform::Result::Ok;
  }

  platform::Result read(uint8_t addr7bit, uint8_t* data, size_t len) noexcept override
  {
    (void)addr7bit;
    if (data && len > 0 && transferRxInject.size() >= len) {
      std::memcpy(data, transferRxInject.data(), len);
    }
    return transferResult;
  }
};

// -----------------------------------------------------------------------------
// MockTickSource: returns configurable microsecond tick
// -----------------------------------------------------------------------------

class MockTickSource final : public platform::ITickSource {
public:
  platform::TickUs tickUs{0};

  platform::TickUs nowUs() const noexcept override { return tickUs; }
};

// -----------------------------------------------------------------------------
// MPU-6050 register addresses (must match driver private enum)
// -----------------------------------------------------------------------------

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU6050_WHO_AM_I_VALUE = 0x68;
constexpr uint8_t MPU6050_DEFAULT_ADDR = 0x68;

// -----------------------------------------------------------------------------
// Test helpers
// -----------------------------------------------------------------------------

/** Put driver in Configured state (init + configure). */
static void bringToConfigured(imu::Mpu6050Driver& driver, MockI2CProvider& i2c,
                              MockTickSource& ticks)
{
  i2c.reset();
  i2c.transferRxInject.push_back(MPU6050_WHO_AM_I_VALUE);
  platform::Result r = driver.init();
  ASSERT_EQ(r, platform::Result::Ok) << "init for Configured";
  ASSERT_EQ(driver.getState(), imu::DriverState::Initialized);

  imu::ImuConfig config{imu::AccelRange::PlusMinus2G, imu::DlpfConfig::Hz44,
                       imu::OutputDataRate::Hz100};
  r = driver.configure(config);
  ASSERT_EQ(r, platform::Result::Ok) << "configure";
  ASSERT_EQ(driver.getState(), imu::DriverState::Configured);
}

/** Put driver in Sampling state. */
static void bringToSampling(imu::Mpu6050Driver& driver, MockI2CProvider& i2c,
                            MockTickSource& ticks)
{
  bringToConfigured(driver, i2c, ticks);
  platform::Result r = driver.startSampling();
  ASSERT_EQ(r, platform::Result::Ok) << "startSampling";
  ASSERT_EQ(driver.getState(), imu::DriverState::Sampling);
}

// -----------------------------------------------------------------------------
// A. init() tests
// -----------------------------------------------------------------------------

TEST(Mpu6050Driver, Init_WhoAmIMismatch_ReturnsHardwareFault)
{
  MockI2CProvider i2c;
  MockTickSource ticks;
  imu::Mpu6050Driver driver(i2c, ticks);

  i2c.transferRxInject.push_back(0x00);  // wrong WHO_AM_I
  platform::Result r = driver.init();

  EXPECT_EQ(r, platform::Result::HardwareFault);
  EXPECT_EQ(driver.getState(), imu::DriverState::Uninitialized);
}

TEST(Mpu6050Driver, Init_WhoAmICorrect_TransitionsToInitialized)
{
  MockI2CProvider i2c;
  MockTickSource ticks;
  imu::Mpu6050Driver driver(i2c, ticks);

  i2c.transferRxInject.push_back(MPU6050_WHO_AM_I_VALUE);
  platform::Result r = driver.init();

  EXPECT_EQ(r, platform::Result::Ok);
  EXPECT_EQ(driver.getState(), imu::DriverState::Initialized);
  EXPECT_TRUE(driver.isInitialized());
  bool readWhoAmI = false;
  for (const auto& t : i2c.transferCalls) {
    if (!t.txData.empty() && t.txData[0] == REG_WHO_AM_I) {
      readWhoAmI = true;
      break;
    }
  }
  EXPECT_TRUE(readWhoAmI);
}

// -----------------------------------------------------------------------------
// B. configure() tests – correct register writes
// -----------------------------------------------------------------------------

TEST(Mpu6050Driver, Configure_WritesAccelConfigConfigSmplrtDiv)
{
  MockI2CProvider i2c;
  MockTickSource ticks;
  imu::Mpu6050Driver driver(i2c, ticks);

  i2c.transferRxInject.push_back(MPU6050_WHO_AM_I_VALUE);
  driver.init();
  i2c.reset();
  i2c.writeCalls.clear();

  imu::ImuConfig config{imu::AccelRange::PlusMinus2G, imu::DlpfConfig::Hz44,
                       imu::OutputDataRate::Hz100};
  platform::Result r = driver.configure(config);

  EXPECT_EQ(r, platform::Result::Ok);
  EXPECT_EQ(driver.getState(), imu::DriverState::Configured);

  // Expect writes: [reg, value]. ACCEL_CONFIG=0x1C AFS_SEL=0 -> 0x00; CONFIG=0x1A DLPF=3 -> 0x03; SMPLRT_DIV=0x19 100Hz -> 9
  EXPECT_GE(i2c.writeCalls.size(), 3u);

  bool hasAccelConfig = false;
  bool hasConfig = false;
  bool hasSmplrtDiv = false;
  for (const auto& w : i2c.writeCalls) {
    if (w.addr7bit != MPU6050_DEFAULT_ADDR || w.data.size() < 2u) continue;
    uint8_t reg = w.data[0];
    uint8_t val = w.data[1];
    if (reg == REG_ACCEL_CONFIG) {
      hasAccelConfig = true;
      EXPECT_EQ((val & 0x18u), 0u) << "AFS_SEL = 0 for PlusMinus2G";
    } else if (reg == REG_CONFIG) {
      hasConfig = true;
      EXPECT_EQ((val & 0x07u), static_cast<uint8_t>(imu::DlpfConfig::Hz44));
    } else if (reg == REG_SMPLRT_DIV) {
      hasSmplrtDiv = true;
      EXPECT_EQ(val, 9u) << "1000/100 - 1 = 9 for Hz100";
    }
  }
  EXPECT_TRUE(hasAccelConfig);
  EXPECT_TRUE(hasConfig);
  EXPECT_TRUE(hasSmplrtDiv);
}

// -----------------------------------------------------------------------------
// C. State enforcement tests
// -----------------------------------------------------------------------------

TEST(Mpu6050Driver, ReadSample_BeforeStartSampling_ReturnsInvalidState)
{
  MockI2CProvider i2c;
  MockTickSource ticks;
  imu::Mpu6050Driver driver(i2c, ticks);

  bringToConfigured(driver, i2c, ticks);
  EXPECT_EQ(driver.getState(), imu::DriverState::Configured);

  imu::ImuSample sample{};
  platform::Result r = driver.readSample(sample);

  EXPECT_EQ(r, platform::Result::InvalidState);
}

TEST(Mpu6050Driver, StartSampling_BeforeConfigure_ReturnsInvalidState)
{
  MockI2CProvider i2c;
  MockTickSource ticks;
  imu::Mpu6050Driver driver(i2c, ticks);

  i2c.transferRxInject.push_back(MPU6050_WHO_AM_I_VALUE);
  driver.init();
  EXPECT_EQ(driver.getState(), imu::DriverState::Initialized);

  platform::Result r = driver.startSampling();

  EXPECT_EQ(r, platform::Result::InvalidState);
  EXPECT_EQ(driver.getState(), imu::DriverState::Initialized);
}

// -----------------------------------------------------------------------------
// D. notifyDataReadyFromIsr – no I2C, sets data-ready flag
// -----------------------------------------------------------------------------

TEST(Mpu6050Driver, NotifyDataReadyFromIsr_DoesNotCallI2C_SetsDataReadyFlag)
{
  MockI2CProvider i2c;
  MockTickSource ticks;
  imu::Mpu6050Driver driver(i2c, ticks);

  bringToSampling(driver, i2c, ticks);
  size_t transferCountBefore = i2c.transferCalls.size();
  size_t writeCountBefore = i2c.writeCalls.size();

  driver.notifyDataReadyFromIsr();

  EXPECT_EQ(i2c.transferCalls.size(), transferCountBefore);
  EXPECT_EQ(i2c.writeCalls.size(), writeCountBefore);

  bool consumed = driver.consumeDataReady();
  EXPECT_TRUE(consumed);

  bool consumedAgain = driver.consumeDataReady();
  EXPECT_FALSE(consumedAgain);
}

// -----------------------------------------------------------------------------
// E. readSample – burst read, 16-bit assembly, timestamp from ITickSource
// -----------------------------------------------------------------------------

TEST(Mpu6050Driver, ReadSample_BurstReadFromAccelXoutH_CorrectAssembly_TimestampFromTickSource)
{
  MockI2CProvider i2c;
  MockTickSource ticks;
  imu::Mpu6050Driver driver(i2c, ticks);

  bringToSampling(driver, i2c, ticks);
  i2c.reset();

  // MPU-6050 accel registers are big-endian: ACCEL_XOUT_H, ACCEL_XOUT_L, ...
  // Inject 6 bytes: X=0x1234, Y=0x5678, Z=0x9ABC
  i2c.transferRxInject.push_back(0x12);
  i2c.transferRxInject.push_back(0x34);
  i2c.transferRxInject.push_back(0x56);
  i2c.transferRxInject.push_back(0x78);
  i2c.transferRxInject.push_back(0x9A);
  i2c.transferRxInject.push_back(0xBC);
  ticks.tickUs = 12345u;

  imu::ImuSample sample{};
  platform::Result r = driver.readSample(sample);

  EXPECT_EQ(r, platform::Result::Ok);
  EXPECT_EQ(sample.accel[0], 0x1234);
  EXPECT_EQ(sample.accel[1], 0x5678);
  EXPECT_EQ(sample.accel[2], static_cast<int16_t>(0x9ABC));
  EXPECT_EQ(sample.timestampUs, 12345u);

  // Driver must have performed a read/transfer starting at ACCEL_XOUT_H (0x3B), 6 bytes
  bool foundAccelBurst = false;
  for (const auto& t : i2c.transferCalls) {
    if (t.txData.size() >= 1u && t.txData[0] == REG_ACCEL_XOUT_H && t.addr7bit == MPU6050_DEFAULT_ADDR) {
      foundAccelBurst = true;
      break;
    }
  }
  for (const auto& w : i2c.writeCalls) {
    if (w.data.size() >= 1u && w.data[0] == REG_ACCEL_XOUT_H) {
      foundAccelBurst = true;
      break;
    }
  }
  EXPECT_TRUE(foundAccelBurst);
}

} // anonymous namespace
