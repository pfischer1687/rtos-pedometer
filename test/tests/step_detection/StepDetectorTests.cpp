/**
 * @file test/tests/step_detection/StepDetectorTests.cpp
 * @brief Behavior-focused unit tests for step_detection::StepDetector.
 */

#include "signal_processing/SignalProcessing.hpp"
#include "step_detection/StepDetector.hpp"
#include <gtest/gtest.h>
#include <vector>

namespace {

constexpr platform::TickUs kSamplePeriodUs = 10000u;
constexpr platform::TickUs kPeakTimeToleranceUs = 25000u;

signal_processing::ProcessedSample magSample(float m, platform::TickUs t) {
  signal_processing::ProcessedSample s{};
  s.magnitude = m;
  s.timestampUs = t;
  return s;
}

std::vector<step_detection::StepEvent>
collectEventsFromMagnitudes(step_detection::StepDetector &det,
                            const std::vector<float> &mag, platform::TickUs t0,
                            platform::TickUs periodUs) {
  std::vector<step_detection::StepEvent> out;
  platform::TickUs t = t0;
  for (float m : mag) {
    if (const auto e = det.processSample(magSample(m, t)).event) {
      out.push_back(*e);
    }
    t += periodUs;
  }
  return out;
}

void assertStepCount(const std::vector<step_detection::StepEvent> &events,
                     size_t expected) {
  EXPECT_EQ(events.size(), expected);
}

void assertMonotonicTimestamps(
    const std::vector<step_detection::StepEvent> &events) {
  for (size_t i = 1; i < events.size(); ++i) {
    EXPECT_GE(events[i].timestampUs, events[i - 1].timestampUs)
        << "event timestamps must be non-decreasing";
  }
}

void assertStrictlyIncreasingStepIndex(
    const std::vector<step_detection::StepEvent> &events) {
  for (size_t i = 1; i < events.size(); ++i) {
    EXPECT_GT(events[i].stepIndex, events[i - 1].stepIndex)
        << "stepIndex must increase across a sequence";
  }
}

void expectConfidenceBounded(
    const std::vector<step_detection::StepEvent> &events) {
  for (const auto &e : events) {
    EXPECT_GE(e.confidence, 0.0f);
    EXPECT_LE(e.confidence, 1.0f);
  }
}

void expectTimestampNear(platform::TickUs actual, platform::TickUs nominal,
                         platform::TickUs tol = kPeakTimeToleranceUs) {
  const uint64_t a = actual;
  const uint64_t n = nominal;
  const uint64_t d = (a > n) ? (a - n) : (n - a);
  EXPECT_LE(d, static_cast<uint64_t>(tol))
      << "timestamp " << actual << " too far from nominal " << nominal;
}

} // namespace

TEST(StepDetectorSynthetic, NoPeakOnFlatLine) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 10u;
  cfg.minPeakProminenceG = 0.5f;
  det.setConfig(cfg);

  std::vector<float> mag(20u, 1.0f);
  const auto events =
      collectEventsFromMagnitudes(det, mag, 0u, kSamplePeriodUs);
  assertStepCount(events, 0u);
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

  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.2f,
                                  1.5f, 1.2f, 1.0f, 1.0f};
  const auto events =
      collectEventsFromMagnitudes(det, mag, 0u, kSamplePeriodUs);
  assertStepCount(events, 1u);
  expectTimestampNear(events[0].timestampUs, 4u * kSamplePeriodUs);
  expectConfidenceBounded(events);
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

  constexpr platform::TickUs kDt = kSamplePeriodUs;
  std::vector<step_detection::StepEvent> events;

  auto push = [&](float m, platform::TickUs t) {
    if (const auto e = det.processSample(magSample(m, t)).event) {
      events.push_back(*e);
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
  bump(static_cast<platform::TickUs>(4u * kDt));
  assertStepCount(events, 1u);
}

TEST(StepDetectorSynthetic, ConstantMagnitudeHasNoPeaks) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 10u;
  cfg.minValleyProminenceG = 0.05f;
  cfg.minPeakProminenceG = 0.05f;
  det.setConfig(cfg);

  constexpr platform::TickUs kShortPeriod = 5000u;
  std::vector<float> mag(30u, 1.4f);
  const auto events = collectEventsFromMagnitudes(det, mag, 0u, kShortPeriod);
  assertStepCount(events, 0u);
}

TEST(StepDetectorSynthetic, ResetRestartsStepIndex) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 100u;
  cfg.minValleyProminenceG = 0.05f;
  cfg.minPeakProminenceG = 0.05f;
  cfg.slopeMin = 0.03f;
  cfg.baselineEmaAlpha = 0.2f;
  det.setConfig(cfg);

  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.2f,
                                  1.5f, 1.2f, 1.0f, 1.0f};
  const auto firstRun =
      collectEventsFromMagnitudes(det, mag, 0u, kSamplePeriodUs);
  ASSERT_EQ(firstRun.size(), 1u);

  det.reset();

  constexpr platform::TickUs kSecondRunT0 = 1000000u;
  const auto secondRun =
      collectEventsFromMagnitudes(det, mag, kSecondRunT0, kSamplePeriodUs);
  ASSERT_EQ(secondRun.size(), 1u);
  EXPECT_EQ(secondRun[0].stepIndex, 0u)
      << "after reset(), stepIndex must restart from zero";
  expectConfidenceBounded(secondRun);
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

  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.04f, 1.08f, 1.04f, 1.0f};
  const auto events =
      collectEventsFromMagnitudes(det, mag, 0u, kSamplePeriodUs);
  assertStepCount(events, 0u);
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

  constexpr platform::TickUs kDt = kSamplePeriodUs;
  constexpr uint32_t kStrideUs = 120000u;
  std::vector<step_detection::StepEvent> events;

  auto bumpAt = [&](platform::TickUs base) {
    (void)det.processSample(magSample(1.0f, base));
    (void)det.processSample(
        magSample(1.0f, static_cast<platform::TickUs>(base + kDt)));
    (void)det.processSample(
        magSample(1.0f, static_cast<platform::TickUs>(base + 2u * kDt)));
    (void)det.processSample(
        magSample(1.45f, static_cast<platform::TickUs>(base + 3u * kDt)));
    if (const auto e =
            det.processSample(magSample(1.0f, static_cast<platform::TickUs>(
                                                  base + 4u * kDt)))
                .event) {
      events.push_back(*e);
    }
  };

  for (unsigned n = 0u; n < 18u; ++n) {
    bumpAt(static_cast<platform::TickUs>(n * kStrideUs));
  }
  assertStepCount(events, 18u);
  assertMonotonicTimestamps(events);
  assertStrictlyIncreasingStepIndex(events);
  expectConfidenceBounded(events);
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

  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.4f, 1.5f, 1.49f, 1.0f};
  const auto events =
      collectEventsFromMagnitudes(det, mag, 0u, kSamplePeriodUs);
  assertStepCount(events, 0u);
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

  constexpr platform::TickUs kDt = kSamplePeriodUs;
  std::vector<step_detection::StepEvent> events;

  auto bump = [&](platform::TickUs base) {
    (void)det.processSample(magSample(1.0f, base));
    (void)det.processSample(
        magSample(1.0f, static_cast<platform::TickUs>(base + kDt)));
    (void)det.processSample(
        magSample(1.0f, static_cast<platform::TickUs>(base + 2u * kDt)));
    (void)det.processSample(
        magSample(1.45f, static_cast<platform::TickUs>(base + 3u * kDt)));
    if (const auto e =
            det.processSample(magSample(1.0f, static_cast<platform::TickUs>(
                                                  base + 4u * kDt)))
                .event) {
      events.push_back(*e);
    }
  };

  bump(0u);
  bump(static_cast<platform::TickUs>(5000000u));

  assertStepCount(events, 2u);
  assertMonotonicTimestamps(events);
  assertStrictlyIncreasingStepIndex(events);
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
  std::vector<step_detection::StepEvent> events;

  for (unsigned n = 0u; n < 25u; ++n) {
    const float o = drift;
    const platform::TickUs b = static_cast<platform::TickUs>(n * kStrideUs);
    (void)det.processSample(magSample(1.0f + o, b));
    (void)det.processSample(
        magSample(1.0f + o, static_cast<platform::TickUs>(b + kDt)));
    (void)det.processSample(
        magSample(1.0f + o, static_cast<platform::TickUs>(b + 2u * kDt)));
    (void)det.processSample(
        magSample(1.4f + o, static_cast<platform::TickUs>(b + 3u * kDt)));
    if (const auto e =
            det.processSample(magSample(1.0f + o, static_cast<platform::TickUs>(
                                                      b + 4u * kDt)))
                .event) {
      events.push_back(*e);
    }
    drift += 0.012f;
  }
  assertStepCount(events, 25u);
  assertMonotonicTimestamps(events);
  assertStrictlyIncreasingStepIndex(events);
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

  std::vector<step_detection::StepEvent> duringWarmup;
  platform::TickUs t = 0u;
  for (int i = 0; i < 30; ++i) {
    if (const auto e = det.processSample(magSample(1.0f, t)).event) {
      duringWarmup.push_back(*e);
    }
    t += kSamplePeriodUs;
  }
  EXPECT_TRUE(duringWarmup.empty()) << "no steps must be emitted during warmup";

  const std::vector<float> mag = {1.0f, 1.0f, 1.0f, 1.2f,
                                  1.5f, 1.2f, 1.0f, 1.0f};
  const auto afterWarmup =
      collectEventsFromMagnitudes(det, mag, t, kSamplePeriodUs);
  EXPECT_FALSE(afterWarmup.empty()) << "steps allowed after warmup completes";
  expectConfidenceBounded(afterWarmup);
}

TEST(StepDetectorSynthetic, SinglePeakToleratesDeterministicJitter) {
  step_detection::StepDetector det;
  step_detection::StepDetectorConfig cfg{};
  cfg.warmupSamples = 0u;
  cfg.minStepIntervalMs = 100u;
  cfg.minValleyProminenceG = 0.04f;
  cfg.minPeakProminenceG = 0.04f;
  cfg.slopeMin = 0.025f;
  cfg.baselineEmaAlpha = 0.18f;
  det.setConfig(cfg);

  const std::vector<float> mag = {1.0f,  1.0f,  1.0f,  1.18f,
                                  1.52f, 1.21f, 0.98f, 1.02f};
  std::vector<step_detection::StepEvent> events;
  platform::TickUs ts = 0u;
  for (size_t i = 0; i < mag.size(); ++i) {
    if (const auto e = det.processSample(magSample(mag[i], ts)).event) {
      events.push_back(*e);
    }
    const unsigned m = static_cast<unsigned>(i % 5u);
    if (m == 0u) {
      ts += 9000u;
    } else if (m == 1u) {
      ts += 11000u;
    } else {
      ts += 10000u;
    }
  }
  assertStepCount(events, 1u);
  expectConfidenceBounded(events);
}
