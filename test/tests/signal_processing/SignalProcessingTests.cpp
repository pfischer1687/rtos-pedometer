/**
 * @file test/tests/signal_processing/SignalProcessingTests.cpp
 * @brief Unit tests for signal_processing::SignalProcessor and magnitude hook.
 */

#include "signal_processing/SignalProcessing.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <thread>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846f;

float referenceAlphaHp(float sampleRateHz, float cutoffHz) {
  const float dt = 1.0f / sampleRateHz;
  const float rc = 1.0f / (2.0f * kPi * cutoffHz);
  return rc / (rc + dt);
}

} // namespace

TEST(SignalProcessingMagnitude, L2Norm) {
  EXPECT_FLOAT_EQ(signal_processing::accelerationMagnitude(3.0f, 4.0f, 0.0f),
                  5.0f);
  EXPECT_FLOAT_EQ(signal_processing::accelerationMagnitude(0.0f, 0.0f, 0.0f),
                  0.0f);
}

TEST(SignalProcessingMovingAverage, ThreeTapOnMagnitude) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.movingAverageWindow = 3;
  proc.setConfig(cfg);

  std::vector<signal_processing::AccelSampleF> in(4);
  for (int i = 0; i < 4; ++i) {
    in[static_cast<size_t>(i)].x = static_cast<float>(i + 1);
    in[static_cast<size_t>(i)].y = 0.0f;
    in[static_cast<size_t>(i)].z = 0.0f;
  }

  std::vector<signal_processing::ProcessedSample> out(4);
  proc.processBatchAccelFloat(in.data(), out.data(), 4);

  EXPECT_FLOAT_EQ(out[0].magnitude, 1.0f);
  EXPECT_FLOAT_EQ(out[1].magnitude, 1.5f);
  EXPECT_FLOAT_EQ(out[2].magnitude, 2.0f);
  EXPECT_FLOAT_EQ(out[3].magnitude, 3.0f);
}

TEST(SignalProcessingHighPass, StepThenHoldMatchesReferenceRecurrence) {
  const float fs = 100.0f;
  const float fc = 10.0f;
  const float alpha = referenceAlphaHp(fs, fc);

  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = fc;
  cfg.lowPassCutoffHz = 0.0f;
  cfg.movingAverageWindow = 1;
  proc.setConfig(cfg);

  signal_processing::AccelSampleF s0{0.0f, 0.0f, 0.0f, 0};
  signal_processing::AccelSampleF s1{1.0f, 0.0f, 0.0f, 0};
  signal_processing::ProcessedSample o0{};
  signal_processing::ProcessedSample o1{};
  signal_processing::ProcessedSample o2{};

  proc.processAccelFloat(s0, o0);
  proc.processAccelFloat(s1, o1);
  proc.processAccelFloat(s1, o2);

  EXPECT_NEAR(o1.accelX, alpha, 1.0e-5f);
  EXPECT_NEAR(o2.accelX, alpha * alpha, 1.0e-5f);
}

TEST(SignalProcessingHighPass, ConstantVectorDecaysTowardZero) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.8f;
  cfg.lowPassCutoffHz = 0.0f;
  cfg.movingAverageWindow = 1;
  proc.setConfig(cfg);

  signal_processing::AccelSampleF s{1000.0f, 0.0f, 0.0f, 0};
  signal_processing::ProcessedSample out{};
  for (int i = 0; i < 600; ++i) {
    proc.processAccelFloat(s, out);
  }
  EXPECT_LT(std::fabs(out.magnitude), 15.0f);
  EXPECT_LT(std::fabs(out.accelX), 15.0f);
}

TEST(SignalProcessingLowPass, AttenuatesHighFrequencySine) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 200;
  cfg.highPassCutoffHz = 0.0f;
  cfg.lowPassCutoffHz = 5.0f;
  cfg.movingAverageWindow = 1;
  proc.setConfig(cfg);

  const float fs = 200.0f;
  const float toneHz = 40.0f;
  const int n = 800;
  float peakIn = 0.0f;
  float peakOut = 0.0f;
  signal_processing::ProcessedSample out{};

  for (int i = 0; i < n; ++i) {
    const float t = static_cast<float>(i) / fs;
    const float x = 1.0f * std::sin(2.0f * kPi * toneHz * t);
    signal_processing::AccelSampleF s{x, 0.0f, 0.0f, 0};
    proc.processAccelFloat(s, out);
    peakIn = std::max(peakIn, std::fabs(x));
    peakOut = std::max(peakOut, std::fabs(out.accelX));
  }

  EXPECT_GT(peakIn, 0.9f);
  EXPECT_LT(peakOut, 0.35f * peakIn);
}

TEST(SignalProcessingImuSample, BatchMatchesPerSample) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 0.0f;
  cfg.lowPassCutoffHz = 10.0f;
  cfg.movingAverageWindow = 1;
  cfg.accelScale = 0.001f;
  proc.setConfig(cfg);

  std::array<imu::ImuSample, 5> in{};
  for (size_t i = 0; i < in.size(); ++i) {
    in[i].accel[0] = static_cast<int16_t>(100 * (static_cast<int>(i) + 1));
    in[i].accel[1] = 0;
    in[i].accel[2] = 0;
    in[i].timestampUs = static_cast<uint32_t>(i * 10000u);
  }

  std::array<signal_processing::ProcessedSample, 5> batchOut{};
  std::array<signal_processing::ProcessedSample, 5> stepOut{};

  proc.processBatch(in.data(), batchOut.data(), in.size());

  signal_processing::SignalProcessor proc2;
  proc2.setConfig(cfg);
  for (size_t i = 0; i < in.size(); ++i) {
    proc2.process(in[i], stepOut[i]);
  }

  for (size_t i = 0; i < in.size(); ++i) {
    EXPECT_NEAR(batchOut[i].accelX, stepOut[i].accelX, 1.0e-4f);
    EXPECT_EQ(batchOut[i].timestampUs, stepOut[i].timestampUs);
  }
}

TEST(SignalProcessingThreading, SharedProcessorConcurrentCalls) {
  signal_processing::SignalProcessor proc;
  signal_processing::FilterConfig cfg{};
  cfg.sampleRateHz = 100;
  cfg.highPassCutoffHz = 1.0f;
  cfg.lowPassCutoffHz = 20.0f;
  cfg.movingAverageWindow = 4;
  proc.setConfig(cfg);

  auto worker = [&proc](int base) {
    signal_processing::ProcessedSample out{};
    for (int i = 0; i < 500; ++i) {
      signal_processing::AccelSampleF s{static_cast<float>(base + i % 7),
                                        static_cast<float>(i % 5),
                                        static_cast<float>((base * i) % 11), 0};
      proc.processAccelFloat(s, out);
      EXPECT_TRUE(std::isfinite(out.magnitude));
      EXPECT_TRUE(std::isfinite(out.accelX));
    }
  };

  std::thread a(worker, 0);
  std::thread b(worker, 100);
  a.join();
  b.join();
}
