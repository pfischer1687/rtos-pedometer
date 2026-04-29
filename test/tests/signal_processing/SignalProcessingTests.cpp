/**
 * @file test/tests/signal_processing/SignalProcessingTests.cpp
 * @brief Unit tests for signal_processing::SignalProcessor and helpers.
 */

#include "signal_processing/SignalProcessing.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>

namespace {

constexpr float kPi = 3.14159265358979323846f;

float referenceAlphaHp(float sampleRateHz, float cutoffHz) {
  const float dt = 1.0f / sampleRateHz;
  const float rc = 1.0f / (2.0f * kPi * cutoffHz);
  return rc / (rc + dt);
}

float referenceAlphaLp(float sampleRateHz, float cutoffHz) {
  const float dt = 1.0f / sampleRateHz;
  const float rc = 1.0f / (2.0f * kPi * cutoffHz);
  return dt / (rc + dt);
}

float rmsAccelX(const std::vector<signal_processing::ProcessedSample> &out,
                size_t beginIdx, size_t endIdx) {
  double s2 = 0.0;
  const size_t n = endIdx - beginIdx;
  EXPECT_GT(n, 0u);
  for (size_t i = beginIdx; i < endIdx; ++i) {
    const float x = out[i].accelX;
    s2 += static_cast<double>(x) * static_cast<double>(x);
  }
  return static_cast<float>(std::sqrt(s2 / static_cast<double>(n)));
}

imu::ImuSample makeSample(int16_t x, int16_t y, int16_t z,
                          platform::TickUs ts = 0) {
  imu::ImuSample s{};
  s.accel[0] = x;
  s.accel[1] = y;
  s.accel[2] = z;
  s.timestampUs = ts;
  return s;
}

} // namespace

// ---------------------------------------------------------------------------
// Magnitude (stateless helper + consistency with processed axes)
// ---------------------------------------------------------------------------

TEST(SignalProcessingMagnitude, L2NormAxisAligned) {
  EXPECT_FLOAT_EQ(signal_processing::accelerationMagnitude(3.0f, 4.0f, 0.0f),
                  5.0f);
  EXPECT_FLOAT_EQ(signal_processing::accelerationMagnitude(0.0f, 0.0f, 0.0f),
                  0.0f);
}

TEST(SignalProcessingMagnitude, L2NormNonAxisAligned) {
  EXPECT_FLOAT_EQ(signal_processing::accelerationMagnitude(3.0f, 4.0f, 12.0f),
                  13.0f);
}

TEST(SignalProcessingMagnitude, MatchesFilteredAxesWhenMovingAverageDisabled) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample in = makeSample(3, 4, 12);
  signal_processing::ProcessedSample out{};
  proc.processOne(in, out);

  const float expected = signal_processing::accelerationMagnitude(
      out.accelX, out.accelY, out.accelZ);
  EXPECT_NEAR(out.magnitude, expected, 1.0e-5f);
}

// ---------------------------------------------------------------------------
// Configuration (observed via outputs; no private access)
// ---------------------------------------------------------------------------

TEST(SignalProcessingConfig, HighPassBypassWhenCutoffNonPositive) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 0.5f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample in = makeSample(100, -200, 300);
  signal_processing::ProcessedSample out{};
  proc.processOne(in, out);

  EXPECT_NEAR(out.accelX, 50.0f, 1.0e-4f);
  EXPECT_NEAR(out.accelY, -100.0f, 1.0e-4f);
  EXPECT_NEAR(out.accelZ, 150.0f, 1.0e-4f);
}

TEST(SignalProcessingConfig, SampleRateBelow2DisablesHighPass) {
  signal_processing::SignalProcessor lowFs;
  signal_processing::FilterConfig cfgLow{};
  cfgLow.sampleRateHz = 1;
  cfgLow.highPassCutoffHz = 10.0f;
  cfgLow.accelScale = 1.0f;
  cfgLow.lowPassCutoffHz = 0.0f;
  lowFs.setConfig(cfgLow);

  signal_processing::SignalProcessor highFs;
  signal_processing::FilterConfig cfgHigh{};
  cfgHigh.sampleRateHz = 100;
  cfgHigh.highPassCutoffHz = 0.0f;
  cfgHigh.accelScale = 1.0f;
  cfgHigh.lowPassCutoffHz = 0.0f;
  highFs.setConfig(cfgHigh);

  imu::ImuSample step = makeSample(1000, 0, 0);
  signal_processing::ProcessedSample oLow{};
  signal_processing::ProcessedSample oHigh{};
  lowFs.processOne(step, oLow);
  highFs.processOne(step, oHigh);

  EXPECT_NEAR(oLow.accelX, 1000.0f, 1.0e-4f);
  EXPECT_NEAR(oHigh.accelX, 1000.0f, 1.0e-4f);
}

TEST(SignalProcessingConfig, LowPassZeroAndNegativeBothBypass) {
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;

  signal_processing::SignalProcessor lpZero;
  lpZero.setConfig(cfg);

  signal_processing::SignalProcessor lpNegative;
  cfg.lowPassCutoffHz = -2.0f;
  lpNegative.setConfig(cfg);

  constexpr int kN = 120;
  std::vector<imu::ImuSample> in(static_cast<size_t>(kN));
  for (int i = 0; i < kN; ++i) {
    in[static_cast<size_t>(i)] = makeSample(
        static_cast<int16_t>(i % 17 - 8), static_cast<int16_t>(i % 5),
        static_cast<int16_t>((i * 3) % 11 - 5));
  }

  std::vector<signal_processing::ProcessedSample> oZero(
      static_cast<size_t>(kN));
  std::vector<signal_processing::ProcessedSample> oNegative(
      static_cast<size_t>(kN));

  lpZero.processBatch(in.data(), oZero.data(), in.size());
  lpNegative.processBatch(in.data(), oNegative.data(), in.size());

  for (int i = 0; i < kN; ++i) {
    EXPECT_NEAR(oZero[static_cast<size_t>(i)].magnitude,
                oNegative[static_cast<size_t>(i)].magnitude, 1.0e-5f);
  }
}

// ---------------------------------------------------------------------------
// Scale
// ---------------------------------------------------------------------------

TEST(SignalProcessingScale, LsbToGWithCustomScale) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 1.0f / 1000.0f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample in = makeSample(1000, 0, 0);
  signal_processing::ProcessedSample out{};
  proc.processOne(in, out);

  EXPECT_NEAR(out.accelX, 1.0f, 1.0e-5f);
  EXPECT_NEAR(out.magnitude, 1.0f, 1.0e-5f);
}

TEST(SignalProcessingScale, PlusMinus2GScaleMatchesGetAccelScale) {
  const float scale =
      signal_processing::getAccelScale(imu::AccelRange::PlusMinus2G);
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = scale;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample in = makeSample(16384, 0, 0);
  signal_processing::ProcessedSample out{};
  proc.processOne(in, out);

  EXPECT_NEAR(out.accelX, 1.0f, 1.0e-4f);
}

// ---------------------------------------------------------------------------
// High-pass
// ---------------------------------------------------------------------------

TEST(SignalProcessingHighPass, StepThenHoldMatchesReferenceRecurrence) {
  const float fs = 100.0f;
  const float fc = 10.0f;
  const float alpha = referenceAlphaHp(fs, fc);

  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = fc;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample s0 = makeSample(0, 0, 0);
  imu::ImuSample s1 = makeSample(1, 0, 0);
  signal_processing::ProcessedSample o0{};
  signal_processing::ProcessedSample o1{};
  signal_processing::ProcessedSample o2{};

  proc.processOne(s0, o0);
  proc.processOne(s1, o1);
  proc.processOne(s1, o2);

  EXPECT_NEAR(o1.accelX, alpha, 1.0e-5f);
  EXPECT_NEAR(o2.accelX, alpha * alpha, 1.0e-5f);
}

TEST(SignalProcessingHighPass, ImpulseDecaysOnFollowingZeros) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 5.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample z = makeSample(0, 0, 0);
  imu::ImuSample spike = makeSample(2000, 0, 0);
  signal_processing::ProcessedSample out{};

  proc.processOne(z, out);
  proc.processOne(spike, out);
  float peak = std::fabs(out.accelX);
  EXPECT_GT(peak, 1.0f);

  float lastAbs = peak;
  for (int i = 0; i < 50; ++i) {
    proc.processOne(z, out);
    const float a = std::fabs(out.accelX);
    EXPECT_LE(a, lastAbs + 1.0e-4f);
    lastAbs = a;
  }
  EXPECT_LT(std::fabs(out.accelX), 0.5f * peak);
}

TEST(SignalProcessingHighPass, ZeroInputSequenceStaysNearZero) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 2.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample z = makeSample(0, 0, 0);
  signal_processing::ProcessedSample out{};
  for (int i = 0; i < 200; ++i) {
    proc.processOne(z, out);
    EXPECT_NEAR(out.accelX, 0.0f, 1.0e-5f);
    EXPECT_NEAR(out.accelY, 0.0f, 1.0e-5f);
    EXPECT_NEAR(out.accelZ, 0.0f, 1.0e-5f);
  }
}

TEST(SignalProcessingHighPass, LargeConstantOffsetDecaysTowardZero) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.8f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample s = makeSample(1000, 0, 0);
  signal_processing::ProcessedSample out{};
  for (int i = 0; i < 600; ++i) {
    proc.processOne(s, out);
  }
  EXPECT_LT(std::fabs(out.magnitude), 15.0f);
  EXPECT_LT(std::fabs(out.accelX), 15.0f);
}

TEST(SignalProcessingHighPass, FasterSampleRateChangesAlphaForStep) {
  const float fc = 5.0f;
  const float alpha50 = referenceAlphaHp(50.0f, fc);
  const float alpha200 = referenceAlphaHp(200.0f, fc);

  auto runFirstStep = [&](uint16_t fs) {
    signal_processing::SignalProcessor proc;
    signal_processing::FilterConfig cfg{};
    cfg.sampleRateHz = fs;
    cfg.highPassCutoffHz = fc;
    cfg.accelScale = 1.0f;
    cfg.lowPassCutoffHz = 0.0f;
    proc.setConfig(cfg);
    imu::ImuSample z = makeSample(0, 0, 0);
    imu::ImuSample one = makeSample(1, 0, 0);
    signal_processing::ProcessedSample o0{};
    signal_processing::ProcessedSample o1{};
    proc.processOne(z, o0);
    proc.processOne(one, o1);
    return o1.accelX;
  };

  const float y50 = runFirstStep(50);
  const float y200 = runFirstStep(200);

  EXPECT_NEAR(y50, alpha50, 1.0e-5f);
  EXPECT_NEAR(y200, alpha200, 1.0e-5f);
  EXPECT_GT(std::fabs(y50 - y200), 0.01f);
}

// ---------------------------------------------------------------------------
// Magnitude low-pass
// ---------------------------------------------------------------------------

TEST(SignalProcessingMagnitudeLpf, FirstOrderStepFromZeroMatchesReference) {
  const float fs = 100.0f;
  const float fc = 4.5f;
  const float alpha = referenceAlphaLp(fs, fc);

  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = fc;
  proc.setConfig(cfg);

  imu::ImuSample in = makeSample(5, 0, 0);
  signal_processing::ProcessedSample out{};
  proc.processOne(in, out);

  const float expectedMag = 5.0f * alpha;
  EXPECT_NEAR(out.magnitude, expectedMag, 1.0e-4f);
}

TEST(SignalProcessingMagnitudeLpf, ConstantInputSettlesToL2Magnitude) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 4.5f;
  proc.setConfig(cfg);

  imu::ImuSample in = makeSample(3, 4, 0);
  signal_processing::ProcessedSample out{};
  for (int i = 0; i < 500; ++i) {
    proc.processOne(in, out);
  }
  EXPECT_NEAR(out.magnitude, 5.0f, 0.02f);
}

// ---------------------------------------------------------------------------
// Low-pass smoothing behavior
// ---------------------------------------------------------------------------

TEST(SignalProcessingLpfSmoothing, LowPassEnabledSmoothsRamp) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 5.0f;
  proc.setConfig(cfg);

  std::array<imu::ImuSample, 4> in{};
  for (int i = 0; i < 4; ++i) {
    in[static_cast<size_t>(i)] = makeSample(static_cast<int16_t>(i + 1), 0, 0);
  }

  std::array<signal_processing::ProcessedSample, 4> out{};
  proc.processBatch(in.data(), out.data(), in.size());

  EXPECT_GT(out[1].magnitude, out[0].magnitude);
  EXPECT_GT(out[2].magnitude, out[1].magnitude);
  EXPECT_GT(out[3].magnitude, out[2].magnitude);
  EXPECT_LT(out[3].magnitude, 4.0f);
}

TEST(SignalProcessingLpfSmoothing, LowPassDisabledMatchesRawMagnitudeSequence) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  constexpr int kTotal = 30;
  std::vector<float> expectedMag(static_cast<size_t>(kTotal));
  std::vector<float> actualMag(static_cast<size_t>(kTotal));

  for (int i = 0; i < kTotal; ++i) {
    imu::ImuSample s = makeSample(static_cast<int16_t>((i % 5) + 1),
                                  static_cast<int16_t>(i % 3),
                                  static_cast<int16_t>((i * 7) % 11));
    signal_processing::ProcessedSample out{};
    proc.processOne(s, out);

    const float mag = signal_processing::accelerationMagnitude(
        static_cast<float>(s.accel[0]) * cfg.accelScale,
        static_cast<float>(s.accel[1]) * cfg.accelScale,
        static_cast<float>(s.accel[2]) * cfg.accelScale);

    expectedMag[static_cast<size_t>(i)] = mag;
    actualMag[static_cast<size_t>(i)] = out.magnitude;
  }

  for (int i = 0; i < kTotal; ++i) {
    EXPECT_NEAR(actualMag[static_cast<size_t>(i)],
                expectedMag[static_cast<size_t>(i)], 1.0e-4f);
  }
}

// ---------------------------------------------------------------------------
// Timestamp
// ---------------------------------------------------------------------------

TEST(SignalProcessingTimestamp, PropagatesPerSample) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample in = makeSample(1, 2, 3, 999888u);
  signal_processing::ProcessedSample out{};
  proc.processOne(in, out);
  EXPECT_EQ(out.timestampUs, 999888u);
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

TEST(SignalProcessingReset, ClearsStateForSubsequentSamples) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 2.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  imu::ImuSample warm = makeSample(500, 0, 0);
  signal_processing::ProcessedSample tmp{};
  for (int i = 0; i < 40; ++i) {
    proc.processOne(warm, tmp);
  }

  proc.reset();

  imu::ImuSample z = makeSample(0, 0, 0);
  imu::ImuSample one = makeSample(1, 0, 0);
  signal_processing::ProcessedSample o0{};
  signal_processing::ProcessedSample o1{};
  proc.processOne(z, o0);
  proc.processOne(one, o1);

  signal_processing::SignalProcessor fresh;
  fresh.setConfig(cfg);
  signal_processing::ProcessedSample f0{};
  signal_processing::ProcessedSample f1{};
  fresh.processOne(z, f0);
  fresh.processOne(one, f1);

  EXPECT_NEAR(o0.accelX, f0.accelX, 1.0e-5f);
  EXPECT_NEAR(o1.accelX, f1.accelX, 1.0e-5f);
  EXPECT_NEAR(o1.magnitude, f1.magnitude, 1.0e-5f);
}

TEST(SignalProcessingReset, ClearsMagnitudeLowPassState) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 5.0f;
  proc.setConfig(cfg);

  signal_processing::ProcessedSample out{};
  const imu::ImuSample step = makeSample(10, 0, 0);
  for (int i = 0; i < 40; ++i) {
    proc.processOne(step, out);
  }

  proc.reset();
  proc.processOne(step, out);

  const float alpha = referenceAlphaLp(100.0f, 5.0f);
  EXPECT_NEAR(out.magnitude, 10.0f * alpha, 1.0e-4f);
}

// ---------------------------------------------------------------------------
// Batch vs single
// ---------------------------------------------------------------------------

TEST(SignalProcessingBatch, MatchesPerSampleMultiAxisAndTimestamps) {
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 1.0f;
  cfg.accelScale = 0.002f;
  cfg.lowPassCutoffHz = 0.0f;

  std::array<imu::ImuSample, 7> in{};
  for (size_t i = 0; i < in.size(); ++i) {
    in[i].accel[0] = static_cast<int16_t>(50 + static_cast<int>(i) * 3);
    in[i].accel[1] = static_cast<int16_t>(-20 + static_cast<int>(i) * 2);
    in[i].accel[2] = static_cast<int16_t>(10 + static_cast<int>(i));
    in[i].timestampUs =
        static_cast<platform::TickUs>(1000u + static_cast<uint32_t>(i) * 777u);
  }

  signal_processing::SignalProcessor procBatch;
  procBatch.setConfig(cfg);
  std::array<signal_processing::ProcessedSample, 7> batchOut{};
  procBatch.processBatch(in.data(), batchOut.data(), in.size());

  signal_processing::SignalProcessor procStep;
  procStep.setConfig(cfg);
  std::array<signal_processing::ProcessedSample, 7> stepOut{};
  for (size_t i = 0; i < in.size(); ++i) {
    procStep.processOne(in[i], stepOut[i]);
  }

  for (size_t i = 0; i < in.size(); ++i) {
    EXPECT_NEAR(batchOut[i].accelX, stepOut[i].accelX, 1.0e-5f);
    EXPECT_NEAR(batchOut[i].accelY, stepOut[i].accelY, 1.0e-5f);
    EXPECT_NEAR(batchOut[i].accelZ, stepOut[i].accelZ, 1.0e-5f);
    EXPECT_NEAR(batchOut[i].magnitude, stepOut[i].magnitude, 1.0e-5f);
    EXPECT_EQ(batchOut[i].timestampUs, stepOut[i].timestampUs);
    EXPECT_EQ(batchOut[i].timestampUs, in[i].timestampUs);
  }
}

// ---------------------------------------------------------------------------
// Numerical stability
// ---------------------------------------------------------------------------

TEST(SignalProcessingNumerical, ExtremeInt16FiniteWithHighPassOff) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.accelScale = 0.0001f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  std::array<int16_t, 4> xs = {32767, -32768, 0, 1234};
  for (int16_t x : xs) {
    imu::ImuSample in = makeSample(x, x, static_cast<int16_t>(-x));
    signal_processing::ProcessedSample out{};
    proc.processOne(in, out);
    EXPECT_TRUE(std::isfinite(out.accelX));
    EXPECT_TRUE(std::isfinite(out.accelY));
    EXPECT_TRUE(std::isfinite(out.accelZ));
    EXPECT_TRUE(std::isfinite(out.magnitude));
    EXPECT_FALSE(std::isnan(out.magnitude));
  }
}

TEST(SignalProcessingNumerical, ExtremeInt16FiniteWithHighPassOn) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 200;
  cfg.highPassCutoffHz = 5.0f;
  cfg.accelScale = 0.0002f;
  cfg.lowPassCutoffHz = 0.0f;
  proc.setConfig(cfg);

  for (int i = 0; i < 400; ++i) {
    const int16_t v = static_cast<int16_t>((i * 7919) % 65535 - 32767);
    imu::ImuSample in = makeSample(v, static_cast<int16_t>(-v / 2),
                                   static_cast<int16_t>(v / 3));
    signal_processing::ProcessedSample out{};
    proc.processOne(in, out);
    EXPECT_TRUE(std::isfinite(out.magnitude));
    EXPECT_TRUE(std::isfinite(out.accelX));
  }
}

// ---------------------------------------------------------------------------
// Frequency-domain
// ---------------------------------------------------------------------------

TEST(SignalProcessingFrequency, LowFrequencyMoreAttenuatedThanHigh) {
  const uint16_t fs = 100;
  const float fc = 1.0f;
  const int n = 6000;
  const int discard = 4000;

  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = fs;
  cfg.highPassCutoffHz = fc;
  cfg.accelScale = 1.0f;
  cfg.lowPassCutoffHz = 0.0f;

  auto runSine = [&](float toneHz) {
    signal_processing::SignalProcessor proc;
    proc.setConfig(cfg);
    std::vector<signal_processing::ProcessedSample> out(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(fs);
      const float s = 2500.0f * std::sin(2.0f * kPi * toneHz * t);
      const float clamped = std::max(-32768.0f, std::min(32767.0f, s));
      const int16_t ax = static_cast<int16_t>(clamped);
      imu::ImuSample in = makeSample(ax, 0, 0);
      proc.processOne(in, out[static_cast<size_t>(i)]);
    }
    return rmsAccelX(out, static_cast<size_t>(discard), static_cast<size_t>(n));
  };

  const float rmsLow = runSine(0.08f);
  const float rmsMid = runSine(1.0f);
  const float rmsHigh = runSine(12.0f);

  EXPECT_GT(rmsHigh, 50.0f);
  EXPECT_LT(rmsLow, 0.45f * rmsHigh);
  EXPECT_GT(rmsMid, rmsLow);
}
