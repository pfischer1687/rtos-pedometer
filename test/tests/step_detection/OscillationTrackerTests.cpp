/**
 * @file test/tests/step_detection/OscillationTrackerTests.cpp
 * @brief Unit tests for step_detection::OscillationTracker.
 */

#include "step_detection/OscillationTracker.hpp"
#include "gtest/gtest.h"
#include <cmath>
#include <random>

namespace {

constexpr platform::TickUs kTs = 10'000u;
constexpr platform::TickUs kMinStepIntervalUs = 300'000u;

/**
 * @brief Create a processed sample with the given magnitude and timestamp.
 * @param mag The magnitude of the sample.
 * @param ts The timestamp of the sample.
 * @return The processed sample.
 */
signal_processing::ProcessedSample magSample(float mag, platform::TickUs ts) {
  signal_processing::ProcessedSample s{};
  s.magnitude = mag;
  s.timestampUs = ts;
  s.accelX = 0.0f;
  s.accelY = 0.0f;
  s.accelZ = mag;
  return s;
}

/**
 * @brief Feed a triangle cycle to the oscillation tracker.
 * @param tr The oscillation tracker.
 * @param tUs The current timestamp.
 * @param samplesPerHalfCycle The number of samples per half cycle.
 * @param baseMag The base magnitude.
 * @param amplitude The amplitude.
 * @param eventTimes The vector to store the event times.
 */
void feedTriangleCycle(step_detection::OscillationTracker &tr,
                       platform::TickUs &tUs, int samplesPerHalfCycle,
                       float baseMag, float amplitude,
                       std::vector<platform::TickUs> *eventTimes = nullptr) {
  for (int i = 0; i < samplesPerHalfCycle; ++i) {
    const float mag =
        baseMag + (amplitude / static_cast<float>(samplesPerHalfCycle)) *
                      static_cast<float>(i);
    const auto ev = tr.processSample(magSample(mag, tUs));
    if (ev && eventTimes != nullptr) {
      eventTimes->push_back(ev->timestampUs);
    }
    tUs += kTs;
  }

  for (int i = 0; i < samplesPerHalfCycle; ++i) {
    const float mag = (baseMag + amplitude) -
                      (amplitude / static_cast<float>(samplesPerHalfCycle)) *
                          static_cast<float>(i);
    const auto ev = tr.processSample(magSample(mag, tUs));
    if (ev && eventTimes != nullptr) {
      eventTimes->push_back(ev->timestampUs);
    }
    tUs += kTs;
  }
}

/**
 * @brief Create an oscillation tracker with a stricter refractory period.
 * @return The oscillation tracker.
 */
step_detection::OscillationTracker make_tracker() {
  step_detection::OscillationTrackerConfig cfg{};
  cfg.min_step_dt = 0.30f;
  return step_detection::OscillationTracker{cfg};
}

} // namespace

TEST(OscillationTracker, FlatSignalProducesNoSteps) {
  step_detection::OscillationTracker tr = make_tracker();
  platform::TickUs tUs = 0u;
  for (int i = 0; i < 200; ++i) {
    (void)tr.processSample(magSample(1.0f, tUs));
    tUs += kTs;
  }

  const auto &st = tr.getDebugStats();
  EXPECT_EQ(st.peaks, 0u);
  EXPECT_EQ(st.emitted, 0u);
}

TEST(OscillationTracker, NoiseSignalProducesNoSteps) {
  step_detection::OscillationTracker tr = make_tracker();
  std::mt19937 rng(123456u);
  std::uniform_real_distribution<float> deltaStep(-0.001f, 0.001f);

  platform::TickUs tUs = 0u;
  float offset = 0.0f;
  for (int i = 0; i < 240; ++i) {
    offset += deltaStep(rng);
    if (offset > 0.01f) {
      offset = 0.01f;
    } else if (offset < -0.01f) {
      offset = -0.01f;
    }
    const float mag = 1.0f + offset;
    (void)tr.processSample(magSample(mag, tUs));
    tUs += kTs;
  }

  const auto &st = tr.getDebugStats();
  EXPECT_EQ(st.peaks, 0u);
  EXPECT_EQ(st.emitted, 0u);
}

TEST(OscillationTracker, TriangleWaveEmitsExpectedSteps) {
  step_detection::OscillationTracker tr = make_tracker();
  platform::TickUs tUs = 0u;
  constexpr int kCycles = 3;
  constexpr int kSamplesPerHalfCycle = 40;
  constexpr float kBase = 1.0f;
  constexpr float kAmplitude = 0.40f;

  std::vector<platform::TickUs> events;
  for (int c = 0; c < kCycles; ++c) {
    feedTriangleCycle(tr, tUs, kSamplesPerHalfCycle, kBase, kAmplitude,
                      &events);
  }

  EXPECT_GE(static_cast<int>(events.size()), kCycles - 1);
  EXPECT_LE(static_cast<int>(events.size()), kCycles + 1);

  const auto &st = tr.getDebugStats();
  EXPECT_EQ(st.emitted, events.size());
  EXPECT_GE(st.peaks, st.emitted);
}

TEST(OscillationTracker, StepIntervalEnforcement) {
  step_detection::OscillationTracker tr = make_tracker();
  platform::TickUs tUs = 0u;
  std::vector<platform::TickUs> eventTimes;

  constexpr int kFastCycles = 12;
  constexpr int kSamplesPerHalfCycle =
      10; // 0.2s full cycle => faster than gate
  for (int c = 0; c < kFastCycles; ++c) {
    feedTriangleCycle(tr, tUs, kSamplesPerHalfCycle, 1.0f, 0.40f, &eventTimes);
  }

  ASSERT_GE(eventTimes.size(), 2u);
  for (std::size_t i = 1; i < eventTimes.size(); ++i) {
    const platform::TickUs dt =
        platform::elapsed(eventTimes[i - 1], eventTimes[i]);
    EXPECT_GE(dt, kMinStepIntervalUs);
  }
}

TEST(OscillationTracker, StatsAccumulationAndReset) {
  step_detection::OscillationTracker tr = make_tracker();
  platform::TickUs tUs = 0u;
  for (int c = 0; c < 4; ++c) {
    feedTriangleCycle(tr, tUs, 40, 1.0f, 0.40f);
  }

  const auto &beforeReset = tr.getDebugStats();
  EXPECT_GT(beforeReset.peaks, 0u);
  EXPECT_GT(beforeReset.emitted, 0u);

  tr.resetDebugStats();

  const auto &afterReset = tr.getDebugStats();
  EXPECT_EQ(afterReset.peaks, 0u);
  EXPECT_EQ(afterReset.emitted, 0u);
  EXPECT_EQ(afterReset.rejected, 0u);
}
