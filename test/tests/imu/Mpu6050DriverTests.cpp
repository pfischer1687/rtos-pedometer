/**
 * @file test/tests/imu/Mpu6050DriverTests.cpp
 * @brief GoogleTest suite for imu::Mpu6050Driver. Uses MockI2CProvider and
 * MockTimer.
 */

#include "imu/Mpu6050Driver.hpp"
#include "platform/Platform.hpp"
#include "support/mocks/MockI2CProvider.hpp"
#include "support/mocks/MockTimer.hpp"
#include <cstdint>
#include <gtest/gtest.h>

namespace {

constexpr uint8_t MPU6050_I2C_ADDR_7BIT = 0x68u;
constexpr uint8_t WHO_AM_I_REG = 0x75u;
constexpr uint8_t WHO_AM_I_EXPECTED = 0x68u;
constexpr uint8_t PWR_MGMT_1_REG = 0x6Bu;
constexpr uint8_t INT_ENABLE_REG = 0x38u;
constexpr uint8_t INT_STATUS_REG = 0x3Au;
constexpr uint8_t ACCEL_XOUT_H_REG = 0x3Bu;
constexpr uint8_t DATA_RDY_EN_BIT = 0x01u;
constexpr uint8_t DATA_RDY_INT_BIT = 0x01u;
constexpr uint8_t CONFIG_REG = 0x1Au;
constexpr uint8_t ACCEL_CONFIG_REG = 0x1Cu;
constexpr uint8_t SMPLRT_DIV_REG = 0x19u;
constexpr uint8_t PWR_MGMT_2_REG = 0x6Cu;
constexpr uint8_t GYRO_DISABLE_BIT = 0x07u;
constexpr size_t SAMPLE_BUF_LEN = 6u;

class Mpu6050DriverTest : public ::testing::Test {
protected:
  void SetUp() override {
    mockI2c_.clearRegisterResponses();
    mockI2c_.clearWriteLog();
    mockI2c_.resetCounters();
    mockI2c_.clearSimulatedFailures();
    mockTimer_.reset();
    driver_ = std::make_unique<imu::Mpu6050Driver>(mockI2c_, mockTimer_,
                                                   MPU6050_I2C_ADDR_7BIT);
  }

  void setupSuccessfulInit() {
    mockI2c_.setRegisterResponseByte(WHO_AM_I_REG, WHO_AM_I_EXPECTED);
  }

  void setupSuccessfulConfigure() {
    setupSuccessfulInit();
    ASSERT_TRUE(platform::isOk(driver_->init()));
    imu::ImuConfig config{};
    config.range = imu::AccelRange::PlusMinus2G;
    config.dlpf = imu::DlpfConfig::Hz44;
    config.odr = imu::OutputDataRate::Hz100;
    ASSERT_TRUE(platform::isOk(driver_->configure(config)));
  }

  void setupSampling() {
    setupSuccessfulConfigure();
    ASSERT_TRUE(platform::isOk(driver_->startSampling()));
  }

  test_support::MockI2CProvider mockI2c_;
  test_support::MockTimer mockTimer_;
  std::unique_ptr<imu::Mpu6050Driver> driver_;
};

/**
 * STATE MACHINE TRANSITION TESTS:
 * - Invalid state transitions
 */

TEST_F(Mpu6050DriverTest, InitWhenNotUninitializedReturnsInvalidState) {
  setupSuccessfulInit();
  ASSERT_TRUE(platform::isOk(driver_->init()));
  platform::Result r = driver_->init();
  EXPECT_EQ(r, platform::Result::InvalidState);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Initialized);
}

TEST_F(Mpu6050DriverTest, ConfigureWhenUninitializedReturnsInvalidState) {
  imu::ImuConfig config{};
  platform::Result r = driver_->configure(config);
  EXPECT_EQ(r, platform::Result::InvalidState);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Uninitialized);
}

TEST_F(Mpu6050DriverTest, ConfigureWhenConfiguredReturnsInvalidState) {
  setupSuccessfulConfigure();
  imu::ImuConfig config{};
  platform::Result r = driver_->configure(config);
  EXPECT_EQ(r, platform::Result::InvalidState);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Configured);
}

TEST_F(Mpu6050DriverTest, StartSamplingWhenInitializedReturnsInvalidState) {
  setupSuccessfulInit();
  ASSERT_TRUE(platform::isOk(driver_->init()));
  platform::Result r = driver_->startSampling();
  EXPECT_EQ(r, platform::Result::InvalidState);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Initialized);
}

TEST_F(Mpu6050DriverTest, StartSamplingWhenUninitializedReturnsInvalidState) {
  platform::Result r = driver_->startSampling();
  EXPECT_EQ(r, platform::Result::InvalidState);
}

TEST_F(Mpu6050DriverTest, StopSamplingWhenConfiguredReturnsInvalidState) {
  setupSuccessfulConfigure();
  platform::Result r = driver_->stopSampling();
  EXPECT_EQ(r, platform::Result::InvalidState);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Configured);
}

TEST_F(Mpu6050DriverTest, StopSamplingWhenUninitializedReturnsInvalidState) {
  platform::Result r = driver_->stopSampling();
  EXPECT_EQ(r, platform::Result::InvalidState);
}

TEST_F(Mpu6050DriverTest, ReadSampleWhenNotSamplingReturnsInvalidState) {
  setupSuccessfulConfigure();
  imu::ImuSample sample{};
  platform::Result r = driver_->readSample(sample);
  EXPECT_EQ(r, platform::Result::InvalidState);
}

TEST_F(Mpu6050DriverTest, ReadSampleWhenUninitializedReturnsInvalidState) {
  imu::ImuSample sample{};
  platform::Result r = driver_->readSample(sample);
  EXPECT_EQ(r, platform::Result::InvalidState);
}

/**
 * STATE MACHINE TRANSITION TESTS:
 * - Valid state transitions
 */

TEST_F(Mpu6050DriverTest, InitTransitionsUninitializedToInitialized) {
  setupSuccessfulInit();
  platform::Result r = driver_->init();
  EXPECT_TRUE(platform::isOk(r));
  EXPECT_EQ(driver_->getState(), imu::DriverState::Initialized);
  EXPECT_TRUE(driver_->isReadyForConfig());
}

TEST_F(Mpu6050DriverTest, ConfigureTransitionsInitializedToConfigured) {
  setupSuccessfulConfigure();
  EXPECT_EQ(driver_->getState(), imu::DriverState::Configured);
  EXPECT_TRUE(driver_->isReadyForSampling());
}

TEST_F(Mpu6050DriverTest, StartSamplingTransitionsConfiguredToSampling) {
  setupSampling();
  EXPECT_EQ(driver_->getState(), imu::DriverState::Sampling);
  EXPECT_TRUE(driver_->isSampling());
}

TEST_F(Mpu6050DriverTest, StopSamplingTransitionsSamplingToConfigured) {
  setupSampling();
  platform::Result r = driver_->stopSampling();
  EXPECT_TRUE(platform::isOk(r));
  EXPECT_EQ(driver_->getState(), imu::DriverState::Configured);
  EXPECT_TRUE(driver_->isReadyForSampling());
}

TEST_F(Mpu6050DriverTest, ResetFromConfiguredTransitionsToUninitialized) {
  setupSuccessfulConfigure();
  platform::Result r = driver_->reset();
  EXPECT_TRUE(platform::isOk(r));
  EXPECT_EQ(driver_->getState(), imu::DriverState::Uninitialized);
  EXPECT_TRUE(driver_->isReadyForInit());
}

TEST_F(Mpu6050DriverTest, ResetFromSamplingTransitionsToUninitialized) {
  setupSampling();
  platform::Result r = driver_->reset();
  EXPECT_TRUE(platform::isOk(r));
  EXPECT_EQ(driver_->getState(), imu::DriverState::Uninitialized);
}

TEST_F(Mpu6050DriverTest, EnterFaultStateTransitionsToFault) {
  setupSampling();
  driver_->enterFaultState();
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
  EXPECT_TRUE(driver_->isFault());
}

TEST_F(Mpu6050DriverTest, EnterFaultStateWhenAlreadyFaultIsIdempotent) {
  setupSampling();
  driver_->enterFaultState();
  driver_->enterFaultState();
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
}

/**
 * STATE MACHINE TESTS:
 * - init
 */

TEST_F(Mpu6050DriverTest, InitFailsWhenWhoAmIMismatch) {
  mockI2c_.setRegisterResponseByte(WHO_AM_I_REG, 0x00u);
  platform::Result r = driver_->init();
  EXPECT_EQ(r, platform::Result::HardwareFault);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
}

TEST_F(Mpu6050DriverTest, InitSucceedsWhenWhoAmICorrect) {
  setupSuccessfulInit();
  platform::Result r = driver_->init();
  EXPECT_EQ(r, platform::Result::Ok);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Initialized);
}

TEST_F(Mpu6050DriverTest, InitFailsWhenFirstWriteFails) {
  setupSuccessfulInit();
  mockI2c_.setNextWriteResult(platform::Result::Error);
  platform::Result r = driver_->init();
  EXPECT_EQ(r, platform::Result::Error);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
}

TEST_F(Mpu6050DriverTest, InitFailsWhenWhoAmITransferFails) {
  setupSuccessfulInit();
  mockI2c_.setRegisterResponseByte(WHO_AM_I_REG, WHO_AM_I_EXPECTED);
  mockI2c_.setNextTransferResult(platform::Result::HardwareFault);
  platform::Result r = driver_->init();
  EXPECT_EQ(r, platform::Result::HardwareFault);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
}

TEST_F(Mpu6050DriverTest, InitWritesPwrMgmt1WithClkselPllXGyro) {
  setupSuccessfulInit();
  driver_->init();
  const test_support::RecordedWrite *w = mockI2c_.lastWriteToRegister(
      MPU6050_I2C_ADDR_7BIT, PWR_MGMT_1_REG, 0x01u);
  ASSERT_NE(w, nullptr);
  EXPECT_FALSE(w->payload.empty());
  EXPECT_EQ(w->payload[0], 0x01u);
}

TEST_F(Mpu6050DriverTest, IsReadyForInitOnlyWhenUninitialized) {
  EXPECT_TRUE(driver_->isReadyForInit());
  setupSuccessfulInit();
  driver_->init();
  EXPECT_FALSE(driver_->isReadyForInit());
}

/**
 * STATE MACHINE TESTS:
 * - configure
 */

TEST_F(Mpu6050DriverTest, ConfigureWritesConfigAccelConfigSmplrtDivPwrMgmt2) {
  setupSuccessfulConfigure();
  EXPECT_NE(mockI2c_.lastWriteToRegister(MPU6050_I2C_ADDR_7BIT, CONFIG_REG, 3u),
            nullptr);
  EXPECT_NE(
      mockI2c_.lastWriteToRegister(MPU6050_I2C_ADDR_7BIT, ACCEL_CONFIG_REG, 0u),
      nullptr);
  EXPECT_NE(
      mockI2c_.lastWriteToRegister(MPU6050_I2C_ADDR_7BIT, SMPLRT_DIV_REG, 9u),
      nullptr);
  EXPECT_NE(mockI2c_.lastWriteToRegister(MPU6050_I2C_ADDR_7BIT, PWR_MGMT_2_REG,
                                         GYRO_DISABLE_BIT),
            nullptr);
}

TEST_F(Mpu6050DriverTest, ConfigureFaultsOnFirstWriteRegVerifiedFailure) {
  setupSuccessfulInit();
  ASSERT_TRUE(platform::isOk(driver_->init()));
  mockI2c_.setRegisterResponseByte(CONFIG_REG, 3u);
  mockI2c_.setRegisterResponseByte(ACCEL_CONFIG_REG, 0u);
  mockI2c_.setRegisterResponseByte(SMPLRT_DIV_REG, 9u);
  mockI2c_.setRegisterResponseByte(PWR_MGMT_2_REG, GYRO_DISABLE_BIT);
  mockI2c_.setNextWriteResult(platform::Result::Error);
  imu::ImuConfig config{};
  config.dlpf = imu::DlpfConfig::Hz44;
  config.odr = imu::OutputDataRate::Hz100;
  platform::Result r = driver_->configure(config);
  EXPECT_EQ(r, platform::Result::Error);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
}

TEST_F(Mpu6050DriverTest, ConfigureFaultsOnTransferFailureDuringVerify) {
  setupSuccessfulInit();
  ASSERT_TRUE(platform::isOk(driver_->init()));
  mockI2c_.setRegisterResponseByte(CONFIG_REG, 3u);
  mockI2c_.setRegisterResponseByte(ACCEL_CONFIG_REG, 0u);
  mockI2c_.setRegisterResponseByte(SMPLRT_DIV_REG, 9u);
  mockI2c_.setRegisterResponseByte(PWR_MGMT_2_REG, GYRO_DISABLE_BIT);
  mockI2c_.setNextTransferResults({platform::Result::HardwareFault,
                                   platform::Result::HardwareFault,
                                   platform::Result::HardwareFault});
  imu::ImuConfig config{};
  config.dlpf = imu::DlpfConfig::Hz44;
  config.odr = imu::OutputDataRate::Hz100;
  platform::Result r = driver_->configure(config);
  EXPECT_EQ(r, platform::Result::HardwareFault);
  EXPECT_TRUE(driver_->isFault());
}

TEST_F(Mpu6050DriverTest, IsReadyForConfigOnlyWhenInitialized) {
  EXPECT_FALSE(driver_->isReadyForConfig());
  setupSuccessfulInit();
  ASSERT_TRUE(platform::isOk(driver_->init()));
  EXPECT_TRUE(driver_->isReadyForConfig());
  imu::ImuConfig config{};
  config.range = imu::AccelRange::PlusMinus2G;
  config.dlpf = imu::DlpfConfig::Hz44;
  config.odr = imu::OutputDataRate::Hz100;
  ASSERT_TRUE(platform::isOk(driver_->configure(config)));
  EXPECT_FALSE(driver_->isReadyForConfig());
  EXPECT_TRUE(driver_->isReadyForSampling());
}

/**
 * STATE MACHINE TESTS:
 * - startSampling
 */

TEST_F(Mpu6050DriverTest, NotifyDataReadyFromIsrDoesNotCallI2C) {
  setupSampling();
  size_t transfersBefore = mockI2c_.transferCount();
  size_t writesBefore = mockI2c_.writeCount();
  driver_->notifyDataReadyFromIsr();
  EXPECT_EQ(mockI2c_.transferCount(), transfersBefore);
  EXPECT_EQ(mockI2c_.writeCount(), writesBefore);
}

TEST_F(Mpu6050DriverTest, NotifyDataReadyFromIsrWhenNotSamplingDoesNothing) {
  setupSuccessfulConfigure();
  driver_->notifyDataReadyFromIsr();
  EXPECT_FALSE(driver_->consumeDataReady());
}

TEST_F(Mpu6050DriverTest, ConsumeDataReadyReturnsTrueOnceAfterSingleNotify) {
  setupSampling();
  driver_->notifyDataReadyFromIsr();
  EXPECT_TRUE(driver_->consumeDataReady());
  EXPECT_FALSE(driver_->consumeDataReady());
}

TEST_F(Mpu6050DriverTest, ConsumeDataReadyReturnsFalseWhenNotNotified) {
  setupSampling();
  EXPECT_FALSE(driver_->consumeDataReady());
}

TEST_F(Mpu6050DriverTest, ConsumeDataReadyReturnsTruePerNotify) {
  setupSampling();
  driver_->notifyDataReadyFromIsr();
  EXPECT_TRUE(driver_->consumeDataReady());
  EXPECT_FALSE(driver_->consumeDataReady());
  driver_->notifyDataReadyFromIsr();
  EXPECT_TRUE(driver_->consumeDataReady());
  EXPECT_FALSE(driver_->consumeDataReady());
}

/**
 * FUNCTIONAL TESTS:
 * - readSample
 */

TEST_F(Mpu6050DriverTest, ReadSampleReadsFromAccelXoutHRegister) {
  setupSampling();
  driver_->notifyDataReadyFromIsr();
  mockI2c_.setRegisterResponseByte(INT_STATUS_REG, DATA_RDY_INT_BIT);
  uint8_t accelData[SAMPLE_BUF_LEN] = {0x00u, 0x01u, 0x00u,
                                       0x02u, 0xFFu, 0xFFu};
  mockI2c_.setRegisterResponseBytes(ACCEL_XOUT_H_REG, accelData,
                                    SAMPLE_BUF_LEN);
  mockI2c_.resetCounters();

  imu::ImuSample sample{};
  ASSERT_TRUE(platform::isOk(driver_->readSample(sample)));
  EXPECT_EQ(sample.accel[0], 1);
  EXPECT_EQ(sample.accel[1], 2);
  EXPECT_EQ(sample.accel[2], -1);
  EXPECT_GE(mockI2c_.transferCount(), 1u);
}

TEST_F(Mpu6050DriverTest, ReadSampleDecodesBigEndianAccelCorrectly) {
  setupSampling();
  driver_->notifyDataReadyFromIsr();
  mockI2c_.setRegisterResponseByte(INT_STATUS_REG, DATA_RDY_INT_BIT);
  uint8_t buf[6] = {0x12u, 0x34u, 0xABu, 0xCDu, 0x00u, 0x01u};
  mockI2c_.setRegisterResponseBytes(ACCEL_XOUT_H_REG, buf, SAMPLE_BUF_LEN);

  imu::ImuSample sample{};
  ASSERT_TRUE(platform::isOk(driver_->readSample(sample)));
  EXPECT_EQ(sample.accel[0], static_cast<int16_t>(0x1234u));
  EXPECT_EQ(sample.accel[1], static_cast<int16_t>(0xABCDu));
  EXPECT_EQ(sample.accel[2], static_cast<int16_t>(0x0001u));
}

TEST_F(Mpu6050DriverTest, ReadSampleUsesTimestampFromNotifyDataReadyFromIsr) {
  setupSampling();
  mockTimer_.setNowUs(12345u);
  driver_->notifyDataReadyFromIsr();
  mockI2c_.setRegisterResponseByte(INT_STATUS_REG, DATA_RDY_INT_BIT);
  uint8_t buf[SAMPLE_BUF_LEN] = {0};
  mockI2c_.setRegisterResponseBytes(ACCEL_XOUT_H_REG, buf, SAMPLE_BUF_LEN);

  imu::ImuSample sample{};
  ASSERT_TRUE(platform::isOk(driver_->readSample(sample)));
  EXPECT_EQ(sample.timestampUs, 12345u);
}

TEST_F(Mpu6050DriverTest, ReadSampleFaultsOnIntStatusTransferFailure) {
  setupSampling();
  driver_->notifyDataReadyFromIsr();
  mockI2c_.setRegisterResponseByte(INT_STATUS_REG, DATA_RDY_INT_BIT);
  mockI2c_.setNextTransferResult(platform::Result::Error);

  imu::ImuSample sample{};
  platform::Result r = driver_->readSample(sample);
  EXPECT_EQ(r, platform::Result::Error);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
}

TEST_F(Mpu6050DriverTest, ReadSampleFaultsOnAccelBurstReadFailure) {
  setupSampling();
  driver_->notifyDataReadyFromIsr();
  mockI2c_.setRegisterResponseByte(INT_STATUS_REG, DATA_RDY_INT_BIT);
  uint8_t buf[SAMPLE_BUF_LEN] = {0};
  mockI2c_.setRegisterResponseBytes(ACCEL_XOUT_H_REG, buf, SAMPLE_BUF_LEN);
  mockI2c_.setNextTransferResult(platform::Result::HardwareFault);

  imu::ImuSample sample{};
  platform::Result r = driver_->readSample(sample);
  EXPECT_EQ(r, platform::Result::HardwareFault);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
}

TEST_F(Mpu6050DriverTest, ReadSampleReturnsDataNotReadyWhenIntStatusClear) {
  setupSampling();
  mockI2c_.setRegisterResponseByte(INT_STATUS_REG, 0u);

  imu::ImuSample sample{};
  platform::Result r = driver_->readSample(sample);
  EXPECT_EQ(r, platform::Result::DataNotReady);
}

TEST_F(Mpu6050DriverTest, ReadRegsWithRetrySucceedsAfterTransientFailure) {
  setupSampling();
  driver_->notifyDataReadyFromIsr();
  mockI2c_.setRegisterResponseByte(INT_STATUS_REG, DATA_RDY_INT_BIT);
  uint8_t buf[SAMPLE_BUF_LEN] = {0};
  mockI2c_.setRegisterResponseBytes(ACCEL_XOUT_H_REG, buf, SAMPLE_BUF_LEN);
  mockI2c_.setNextTransferResults({platform::Result::Ok, platform::Result::Busy,
                                   platform::Result::Busy,
                                   platform::Result::Ok});

  imu::ImuSample sample{};
  platform::Result r = driver_->readSample(sample);
  EXPECT_TRUE(platform::isOk(r));
}

TEST_F(Mpu6050DriverTest, ReadRegsWithRetryFailsAfterAllAttemptsFail) {
  setupSampling();
  driver_->notifyDataReadyFromIsr();
  mockI2c_.setRegisterResponseByte(INT_STATUS_REG, DATA_RDY_INT_BIT);
  uint8_t buf[SAMPLE_BUF_LEN] = {0};
  mockI2c_.setRegisterResponseBytes(ACCEL_XOUT_H_REG, buf, SAMPLE_BUF_LEN);
  mockI2c_.setNextTransferResult(platform::Result::Error);
  mockI2c_.setNextTransferResult(platform::Result::Error);
  mockI2c_.setNextTransferResult(platform::Result::Error);
  mockI2c_.setNextTransferResult(platform::Result::Error);

  imu::ImuSample sample{};
  platform::Result r = driver_->readSample(sample);
  EXPECT_EQ(r, platform::Result::Error);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
}

/**
 * STATE MACHINE TESTS:
 * - reset
 */

TEST_F(Mpu6050DriverTest, ResetFaultsOnWriteIntEnableFailure) {
  setupSampling();
  mockI2c_.setNextWriteResult(platform::Result::Error);
  platform::Result r = driver_->reset();
  EXPECT_EQ(r, platform::Result::Error);
  EXPECT_EQ(driver_->getState(), imu::DriverState::Fault);
}

TEST_F(Mpu6050DriverTest,
       HealthCheckReturnsErrorWhenUninitializedAndNoMockResponse) {
  platform::Result r = driver_->healthCheck();
  EXPECT_EQ(r, platform::Result::Error);
}

} // namespace
