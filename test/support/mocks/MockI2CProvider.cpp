/**
 * @file test/support/mocks/MockI2CProvider.cpp
 * @brief Implementation of MockI2CProvider.
 */

#include "MockI2CProvider.hpp"
#include <algorithm>
#include <cstring>

namespace test_support {

void MockI2CProvider::clearSimulatedFailures() noexcept {
  nextWriteResult_ = platform::Result::Ok;
  nextReadResult_ = platform::Result::Ok;
  nextTransferResult_ = platform::Result::Ok;
}

void MockI2CProvider::resetCounters() noexcept {
  transferCount_ = 0;
  writeCount_ = 0;
  readCount_ = 0;
}

platform::Result MockI2CProvider::write(uint8_t addr7bit, const uint8_t *data,
                                        size_t len,
                                        bool isRepeatedStart) noexcept {
  (void)isRepeatedStart;
  ++writeCount_;

  if (nextWriteResult_ != platform::Result::Ok)
    return nextWriteResult_;
  if (len == 0u) {
    return platform::Result::Ok;
  }
  if (data == nullptr) {
    return platform::Result::InvalidArgument;
  }
  if (len == 1u) {
    return platform::Result::Ok;
  }

  uint8_t reg = data[0];
  std::vector<uint8_t> payload(data + 1, data + len);
  registerResponses_[reg] = payload;
  writeLog_.push_back({addr7bit, reg, payload});

  return platform::Result::Ok;
}

platform::Result MockI2CProvider::read(uint8_t addr7bit, uint8_t *data,
                                       size_t len) noexcept {
  (void)addr7bit;
  (void)data;
  (void)len;
  return platform::Result::NotSupported;
}

platform::Result MockI2CProvider::transfer(uint8_t addr7bit,
                                           const uint8_t *txData, size_t txLen,
                                           uint8_t *rxData,
                                           size_t rxLen) noexcept {
  (void)addr7bit;
  ++transferCount_;

  if (nextTransferResult_ != platform::Result::Ok) {
    platform::Result r = nextTransferResult_;
    nextTransferResult_ = platform::Result::Ok;
    return r;
  }

  if ((txLen > 0 && txData == nullptr) || (rxLen > 0 && rxData == nullptr)) {
    return platform::Result::InvalidArgument;
  }

  if (!(txLen == 1 && rxLen > 0))
    return platform::Result::NotSupported;

  const uint8_t reg = txData[0];
  if (!registerResponses_.contains(reg)) {
    return platform::Result::Error;
  }

  const std::vector<uint8_t> &payload = registerResponses_.at(reg);
  if (rxLen > payload.size()) {
    return platform::Result::Error;
  }

  std::copy_n(payload.begin(), rxLen, rxData);
  return platform::Result::Ok;
}

void MockI2CProvider::setRegisterResponseBytes(uint8_t registerAddr,
                                               const uint8_t *bytes,
                                               size_t len) {
  if (len > 0 && bytes == nullptr) {
    return;
  }

  registerResponses_[registerAddr] = std::vector<uint8_t>(bytes, bytes + len);
}

void MockI2CProvider::setRegisterResponseByte(uint8_t registerAddr,
                                              uint8_t byte) {
  registerResponses_[registerAddr] = std::vector<uint8_t>{byte};
}

void MockI2CProvider::clearRegisterResponses() { registerResponses_.clear(); }

void MockI2CProvider::clearRegisterResponse(uint8_t registerAddr) {
  registerResponses_.erase(registerAddr);
}

size_t MockI2CProvider::writeCountTo(uint8_t addr7bit) const noexcept {
  size_t n = 0;
  for (const RecordedWrite &w : writeLog_) {
    if (w.addr7bit == addr7bit) {
      ++n;
    }
  }
  return n;
}

const RecordedWrite *MockI2CProvider::lastWriteToRegister(
    uint8_t addr7bit, uint8_t reg,
    std::optional<uint8_t> value) const noexcept {
  for (auto it = writeLog_.crbegin(); it != writeLog_.crend(); ++it) {
    if (it->addr7bit != addr7bit || it->reg != reg)
      continue;
    if (value.has_value()) {
      if (it->payload.empty() || it->payload[0] != *value)
        continue;
    }
    return &(*it);
  }
  return nullptr;
}

} // namespace test_support
