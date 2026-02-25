/**
 * @file test/support/mocks/MockI2CProvider.hpp
 * @brief Mock for platform::II2CProvider.
 */

#ifndef TEST_SUPPORT_MOCKS_MOCKI2CPROVIDER_HPP
#define TEST_SUPPORT_MOCKS_MOCKI2CPROVIDER_HPP

#include "platform/I2CProvider.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace test_support {

/**
 * @struct RecordedWrite
 * @brief Recorded I2C write for verification.
 */
struct RecordedWrite {
  uint8_t addr7bit{0};
  uint8_t reg{0};
  std::vector<uint8_t> payload;
};

/**
 * @class MockI2CProvider
 * @brief Implements II2CProvider with configurable responses and failure
 * injection.
 */
class MockI2CProvider : public platform::II2CProvider {
public:
  MockI2CProvider() = default;
  ~MockI2CProvider() override = default;

  platform::Result transfer(uint8_t addr7bit, const uint8_t *txData,
                            size_t txLen, uint8_t *rxData,
                            size_t rxLen) noexcept override;

  platform::Result write(uint8_t addr7bit, const uint8_t *data, size_t len,
                         bool isRepeatedStart = false) noexcept override;

  platform::Result read(uint8_t addr7bit, uint8_t *data,
                        size_t len) noexcept override;

  /**
   * @brief Set response bytes for a register.
   * @param registerAddr Register address.
   * @param bytes Response bytes.
   * @param len Length of response bytes.
   */
  void setRegisterResponseBytes(uint8_t registerAddr, const uint8_t *bytes,
                           size_t len);

  /**
   * @brief Set single-byte response for a register.
   * @param registerAddr Register address.
   * @param byte Response byte.
   */
  void setRegisterResponseByte(uint8_t registerAddr, uint8_t byte);

  /**
   * @brief Clear all register responses.
   */
  void clearRegisterResponses();

  /**
   * @brief Clear response for one register.
   * @param registerAddr Register address.
   */
  void clearRegisterResponse(uint8_t registerAddr);

  /**
   * @brief Set next write result.
   * @param r Result.
   */
  void setNextWriteResult(platform::Result r) noexcept { nextWriteResult_ = r; }

  /**
   * @brief Set next read result.
   * @param r Result.
   */
  void setNextReadResult(platform::Result r) noexcept { nextReadResult_ = r; }

  /**
   * @brief Set next transfer result.
   * @param r Result.
   */
  void setNextTransferResult(platform::Result r) noexcept {
    nextTransferResult_ = r;
  }

  /**
   * @brief Clear simulated failures.
   */
  void clearSimulatedFailures() noexcept;

  /**
   * @brief Get transfer count.
   * @return Transfer count.
   */
  size_t transferCount() const noexcept { return transferCount_; }

  /**
   * @brief Get write count.
   * @return Write count.
   */
  size_t writeCount() const noexcept { return writeCount_; }

  /**
   * @brief Get read count.
   * @return Read count.
   */
  size_t readCount() const noexcept { return readCount_; }

  /**
   * @brief Reset counters.
   */
  void resetCounters() noexcept;

  /**
   * @brief Get write log.
   * @return Write log.
   */
  const std::vector<RecordedWrite> &writeLog() const noexcept {
    return writeLog_;
  }

  /**
   * @brief Clear write log.
   */
  void clearWriteLog() { writeLog_.clear(); }

  /**
   * @brief Get write count to a given 7-bit address.
   * @param addr7bit 7-bit address.
   * @return Write count.
   */
  size_t writeCountTo(uint8_t addr7bit) const noexcept;

  /**
   * @brief Find last write that wrote the given register (first data byte) and
   * optional value.
   * @param addr7bit 7-bit address.
   * @param reg Register address.
   * @param value Optional value.
   * @return Last write.
   */
  const RecordedWrite *
  lastWriteToRegister(uint8_t addr7bit, uint8_t reg,
                      std::optional<uint8_t> value = {}) const noexcept;

private:
  std::unordered_map<uint8_t, std::vector<uint8_t>> registerResponses_;
  platform::Result nextWriteResult_{platform::Result::Ok};
  platform::Result nextReadResult_{platform::Result::Ok};
  platform::Result nextTransferResult_{platform::Result::Ok};
  size_t transferCount_{0};
  size_t writeCount_{0};
  size_t readCount_{0};
  std::vector<RecordedWrite> writeLog_;
};

} // namespace test_support

#endif /* TEST_SUPPORT_MOCKS_MOCKI2CPROVIDER_HPP */
