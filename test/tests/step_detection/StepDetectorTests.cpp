/**
 * @file test/tests/step_detection/StepDetectorTests.cpp
 * @brief Unit tests for step_detection::StepDetector (synthetic streams).
 */

#include "step_detection/StepDetector.hpp"
#include "signal_processing/SignalProcessing.hpp"
#include <gtest/gtest.h>
#include <optional>
#include <vector>

namespace {

signal_processing::ProcessedSample magSample(float m, platform::TickUs t) {
  signal_processing::ProcessedSample s{};
  s.magnitude = m;
  s.timestampUs = t;
  return s;
}

} // namespace

TEST(StepDetectorSynthetic, NoPeakOnFlatLine) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 10u;
  cfg.minPeakProminenceG = 0.5f;
  det.setConfig(cfg);
  for (int i = 0; i < 20; ++i) {
    const auto ev =
        det.processSample(magSample(1.0f, static_cast<platform::TickUs>(i * 10000u)));
    EXPECT_FALSE(ev.has_value());
  }
}

TEST(StepDetectorSynthetic, DetectsSinglePeak) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 100u;
  cfg.minValleyProminenceG = 0.05f;
  cfg.minPeakProminenceG = 0.05f;
  cfg.slopeMin = 0.03f;
  cfg.baselineEmaAlpha = 0.2f;
  det.setConfig(cfg);

  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.2f, 1.5f, 1.2f, 1.0f, 1.0f};
  std::optional<step_detection::StepEvent> ev;
  for (size_t i = 0; i < mag.size(); ++i) {
    if (const auto e =
            det.processSample(magSample(mag[i], static_cast<platform::TickUs>(i * 10000u)))) {
      ev = e;
    }
  }
  ASSERT_TRUE(ev.has_value());
  EXPECT_EQ(ev->stepIndex, 0u);
  EXPECT_EQ(ev->timestampUs, 40000u);
  EXPECT_GE(ev->confidence, 0.0f);
  EXPECT_LE(ev->confidence, 1.0f);
}

TEST(StepDetectorSynthetic, MinIntervalSuppressesSecondPeak) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 50u;
  cfg.minValleyProminenceG = 0.05f;
  cfg.minPeakProminenceG = 0.05f;
  cfg.slopeMin = 0.03f;
  cfg.baselineEmaAlpha = 0.2f;
  det.setConfig(cfg);

  constexpr platform::TickUs kDt = 10000u;
  size_t steps = 0u;
  auto push = [&](float m, platform::TickUs t) {
    if (det.processSample(magSample(m, t))) {
      ++steps;
    }
  };

  auto bump = [&](platform::TickUs base) {
    push(1.0f, base);
    push(1.0f, static_cast<platform::TickUs>(base + kDt));
    push(1.0f, static_cast<platform::TickUs>(base + 2u * kDt));
    push(1.5f, static_cast<platform::TickUs>(base + 3u * kDt));
    push(1.0f, static_cast<platform::TickUs>(base + 4u * kDt));
  };
  bump(0u);
  bump(40000u);
  EXPECT_EQ(steps, 1u);
}

TEST(StepDetectorSynthetic, ConstantMagnitudeHasNoPeaks) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 10u;
  cfg.minValleyProminenceG = 0.05f;
  cfg.minPeakProminenceG = 0.05f;
  det.setConfig(cfg);

  size_t steps = 0u;
  for (int i = 0; i < 30; ++i) {
    if (det.processSample(magSample(1.4f, static_cast<platform::TickUs>(i * 5000u)))) {
      ++steps;
    }
  }
  EXPECT_EQ(steps, 0u);
}

TEST(StepDetectorSynthetic, ResetClearsStepIndex) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 100u;
  cfg.minValleyProminenceG = 0.05f;
  cfg.minPeakProminenceG = 0.05f;
  cfg.slopeMin = 0.03f;
  cfg.baselineEmaAlpha = 0.2f;
  det.setConfig(cfg);

  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.2f, 1.5f, 1.2f, 1.0f, 1.0f};
  std::optional<step_detection::StepEvent> first;
  for (size_t i = 0; i < mag.size(); ++i) {
    if (const auto e =
            det.processSample(magSample(mag[i], static_cast<platform::TickUs>(i * 10000u)))) {
      first = e;
    }
  }
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->stepIndex, 0u);

  det.reset();

  std::optional<step_detection::StepEvent> afterReset;
  for (size_t i = 0; i < mag.size(); ++i) {
    if (const auto e = det.processSample(
            magSample(mag[i], static_cast<platform::TickUs>(1000000u + i * 10000u)))) {
      afterReset = e;
    }
  }
  ASSERT_TRUE(afterReset.has_value());
  EXPECT_EQ(afterReset->stepIndex, 0u);
}

TEST(StepDetectorSynthetic, SlopeRejectsSoftBump) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 50u;
  cfg.minValleyProminenceG = 0.02f;
  cfg.minPeakProminenceG = 0.02f;
  cfg.slopeMin = 0.08f;
  cfg.baselineEmaAlpha = 0.15f;
  det.setConfig(cfg);

  // Would satisfy m0 < m1 >= m2 with tiny slopes (rise/fall ≈ 0.04 g).
  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.04f, 1.08f, 1.04f, 1.0f};
  size_t steps = 0u;
  for (size_t i = 0; i < mag.size(); ++i) {
    if (det.processSample(magSample(mag[i], static_cast<platform::TickUs>(i * 10000u)))) {
      ++steps;
    }
  }
  EXPECT_EQ(steps, 0u);
}

TEST(StepDetectorSynthetic, StableWalkingTrainMaintainsCount) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 100u;
  cfg.minValleyProminenceG = 0.08f;
  cfg.minPeakProminenceG = 0.08f;
  cfg.slopeMin = 0.04f;
  cfg.baselineEmaAlpha = 0.08f;
  det.setConfig(cfg);

  constexpr platform::TickUs kDt = 10000u;
  constexpr uint32_t kStrideUs = 120000u;
  size_t steps = 0u;
  auto bumpAt = [&](platform::TickUs base) {
    (void)det.processSample(magSample(1.0f, base));
    (void)det.processSample(magSample(1.0f, static_cast<platform::TickUs>(base + kDt)));
    (void)det.processSample(magSample(1.0f, static_cast<platform::TickUs>(base + 2u * kDt)));
    (void)det.processSample(
        magSample(1.45f, static_cast<platform::TickUs>(base + 3u * kDt)));
    // Step is emitted when the trailing valley arrives (5th sample).
    if (det.processSample(magSample(1.0f, static_cast<platform::TickUs>(base + 4u * kDt)))) {
      ++steps;
    }
  };

  for (unsigned n = 0u; n < 18u; ++n) {
    bumpAt(static_cast<platform::TickUs>(n * kStrideUs));
  }
  EXPECT_EQ(steps, 18u);
}

TEST(StepDetectorSynthetic, AsymmetricPeakFailsSlope) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 50u;
  cfg.minValleyProminenceG = 0.05f;
  cfg.minPeakProminenceG = 0.05f;
  cfg.slopeMin = 0.04f;
  det.setConfig(cfg);

  // Peak sample 1.5 g with a very shallow trailing edge → fails `slopeMin`.
  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.4f, 1.5f, 1.49f, 1.0f};
  size_t steps = 0u;
  for (size_t i = 0; i < mag.size(); ++i) {
    if (det.processSample(magSample(mag[i], static_cast<platform::TickUs>(i * 10000u)))) {
      ++steps;
    }
  }
  EXPECT_EQ(steps, 0u);
}

TEST(StepDetectorSynthetic, LongGapStillRegistersStep) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 200u;
  cfg.minValleyProminenceG = 0.08f;
  cfg.minPeakProminenceG = 0.08f;
  cfg.slopeMin = 0.04f;
  det.setConfig(cfg);

  constexpr platform::TickUs kDt = 10000u;
  auto bump = [&](platform::TickUs base) {
    (void)det.processSample(magSample(1.0f, base));
    (void)det.processSample(magSample(1.0f, static_cast<platform::TickUs>(base + kDt)));
    (void)det.processSample(magSample(1.0f, static_cast<platform::TickUs>(base + 2u * kDt)));
    (void)det.processSample(
        magSample(1.45f, static_cast<platform::TickUs>(base + 3u * kDt)));
    return det.processSample(magSample(1.0f, static_cast<platform::TickUs>(base + 4u * kDt)));
  };

  ASSERT_TRUE(bump(0u).has_value());

  const auto second =
      bump(static_cast<platform::TickUs>(5000000u)); // +5 s — rhythm reset
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->stepIndex, 1u);
}

TEST(StepDetectorSynthetic, ManyStepsWithSlowDriftStillDetected) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 90u;
  cfg.minValleyProminenceG = 0.10f;
  cfg.minPeakProminenceG = 0.08f;
  cfg.slopeMin = 0.04f;
  cfg.baselineEmaAlpha = 0.06f;
  det.setConfig(cfg);

  constexpr platform::TickUs kDt = 5000u;
  constexpr uint32_t kStrideUs = 100000u;
  float drift = 0.0f;
  size_t steps = 0u;

  for (unsigned n = 0u; n < 25u; ++n) {
    const float o = drift;
    const platform::TickUs b = static_cast<platform::TickUs>(n * kStrideUs);
    (void)det.processSample(magSample(1.0f + o, b));
    (void)det.processSample(magSample(1.0f + o, static_cast<platform::TickUs>(b + kDt)));
    (void)det.processSample(magSample(1.0f + o, static_cast<platform::TickUs>(b + 2u * kDt)));
    (void)det.processSample(magSample(1.4f + o, static_cast<platform::TickUs>(b + 3u * kDt)));
    if (det.processSample(magSample(1.0f + o, static_cast<platform::TickUs>(b + 4u * kDt)))) {
      ++steps;
    }
    drift += 0.012f;
  }
  EXPECT_EQ(steps, 25u);
}

TEST(StepDetectorSynthetic, WarmupSuppressesStepsUntilDone) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 30u;
  cfg.minStepIntervalMs = 100u;
  cfg.minValleyProminenceG = 0.05f;
  cfg.minPeakProminenceG = 0.05f;
  cfg.slopeMin = 0.03f;
  cfg.baselineEmaAlpha = 0.2f;
  det.setConfig(cfg);

  platform::TickUs t = 0u;
  for (int i = 0; i < 30; ++i) {
    EXPECT_FALSE(det.processSample(magSample(1.0f, t)).has_value());
    t += 10000u;
  }

  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.2f, 1.5f, 1.2f, 1.0f, 1.0f};
  std::optional<step_detection::StepEvent> ev;
  for (size_t i = 0; i < mag.size(); ++i) {
    if (const auto e = det.processSample(magSample(mag[i], t))) {
      ev = e;
    }
    t += 10000u;
  }
  ASSERT_TRUE(ev.has_value());
  EXPECT_EQ(ev->stepIndex, 0u);
  EXPECT_GE(ev->confidence, 0.0f);
  EXPECT_LE(ev->confidence, 1.0f);
}
