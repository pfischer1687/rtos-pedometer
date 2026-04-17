/**
 * @file signal_processing/SignalProcessing.cpp
 * @brief Implementation of the SignalProcessor class.
 */

#include "signal_processing/SignalProcessing.hpp"

namespace signal_processing {

namespace {

constexpr float TWO_PI = 6.2831853f;

} // anonymous namespace

void SignalProcessor::applyConfig() noexcept {
  _accelScale = _config.accelScale;

  if (_config.sampleRateHz < 2u) {
    _hpBypass = true;
    _alphaHp = 0.0f;
    _maWindow = 1u;
    return;
  }

  const float fs = static_cast<float>(_config.sampleRateHz);
  const float dt = 1.0f / fs;

  if (_config.highPassCutoffHz > 0.0f) {
    const float rc = 1.0f / (TWO_PI * _config.highPassCutoffHz);
    _alphaHp = rc / (rc + dt);
    _hpBypass = false;
  } else {
    _alphaHp = 0.0f;
    _hpBypass = true;
  }

  uint32_t w = _config.movingAverageWindow;
  if (w < 1u) {
    w = 1u;
  }
  if (w > MAX_MOVING_AVERAGE_WINDOW) {
    w = MAX_MOVING_AVERAGE_WINDOW;
  }
  _maWindow = w;
}

void SignalProcessor::resetFilters() noexcept {
  for (auto &axis : _axis) {
    axis.hpOut = 0.0f;
    axis.hpPrevIn = 0.0f;
  }

  std::fill(std::begin(_maBuf), std::end(_maBuf), 0.0f);

  _maSum = 0.0f;
  _maIndex = 0u;
  _maCount = 0u;
}

void SignalProcessor::setConfig(const FilterConfig &config) noexcept {
  _config = config;
  applyConfig();
  resetFilters();
}

void SignalProcessor::reset() noexcept { resetFilters(); }

float SignalProcessor::processAxis(const imu::ImuSample &in,
                                   size_t axis) noexcept {
  if (axis >= imu::NUM_ACC_AXES) {
    return 0.0f;
  }

  float x = static_cast<float>(in.accel[axis]) * _accelScale;

  if (!_hpBypass) {
    const float y = _alphaHp * (_axis[axis].hpOut + x - _axis[axis].hpPrevIn);
    _axis[axis].hpOut = (y == y) ? y : 0.0f; // NaN check
    _axis[axis].hpPrevIn = x;
    x = y;
  } else {
    _axis[axis].hpPrevIn = x;
  }

  return x;
}

float SignalProcessor::updateMagnitudeMovingAverage(float mag) noexcept {
  if (_maWindow <= 1u) {
    return mag;
  }

  _maSum -= _maBuf[_maIndex];
  _maBuf[_maIndex] = mag;
  _maSum += mag;

  ++_maIndex;
  if (_maIndex >= _maWindow) {
    _maIndex = 0u;
  }

  if (_maCount < _maWindow) {
    ++_maCount;
  }

  return _maSum / static_cast<float>(_maCount);
}

void SignalProcessor::processOne(const imu::ImuSample &in,
                                 ProcessedSample &out) noexcept {
  out.accelX = processAxis(in, 0);
  out.accelY = processAxis(in, 1);
  out.accelZ = processAxis(in, 2);

  out.timestampUs = in.timestampUs;
  const float mag = accelerationMagnitude(out.accelX, out.accelY, out.accelZ);
  out.magnitude = updateMagnitudeMovingAverage(mag);
}

void SignalProcessor::processBatch(const imu::ImuSample *in,
                                   ProcessedSample *out,
                                   const size_t count) noexcept {
  if (!in || !out || count == 0u) {
    return;
  }

  for (size_t i = 0; i < count; ++i) {
    processOne(in[i], out[i]);
  }
}

} // namespace signal_processing
