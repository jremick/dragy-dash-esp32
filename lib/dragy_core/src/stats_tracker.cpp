#include "dragy_core/stats_tracker.h"

#include <algorithm>
#include <cmath>

namespace dragy {

void StatsTracker::resetStats() {
  stats_ = RunStats{};
  ringBaseSeq_ = 0;
  ringCount_ = 0;
  activeBucket_ = DistanceBucket{};
  win1Km_.reset();
  win5Km_.reset();
  win10Km_.reset();
  win1Mi_.reset();
  win3Mi_.reset();
  win5Mi_.reset();
  hasLastUsable_ = false;
  lastFilteredSpeedKph_ = 0.0f;
}

float StatsTracker::elapsedSecondsBetween(
  const TelemetrySample &previous,
  const TelemetrySample &current
) {
  if (previous.hasGpsTimeOfWeek && current.hasGpsTimeOfWeek &&
      current.gpsTimeOfWeekMs != previous.gpsTimeOfWeekMs) {
    constexpr uint32_t kGpsWeekMs = 604800000u;
    const uint32_t deltaMs = current.gpsTimeOfWeekMs >= previous.gpsTimeOfWeekMs
                               ? (current.gpsTimeOfWeekMs - previous.gpsTimeOfWeekMs)
                               : ((kGpsWeekMs - previous.gpsTimeOfWeekMs) + current.gpsTimeOfWeekMs);
    const float seconds = static_cast<float>(deltaMs) / 1000.0f;
    if (seconds > 0.0f && seconds < kMaxSampleGapSeconds) {
      return seconds;
    }
  }

  return static_cast<float>(current.timestampMs - previous.timestampMs) / 1000.0f;
}

void StatsTracker::record(const TelemetrySample &sample, float filteredSpeedKph) {
  current_ = sample;
  currentValid_ = true;

  if (!sample.usable()) {
    hasLastUsable_ = false;
    lastFilteredSpeedKph_ = 0.0f;
    return;
  }

  const float speedKph = std::max(0.0f, filteredSpeedKph);
  stats_.peakSpeedKph = std::max(stats_.peakSpeedKph, speedKph);

  if (hasLastUsable_) {
    const float elapsedSeconds = elapsedSecondsBetween(lastUsable_, sample);
    if (elapsedSeconds > 0.0f && elapsedSeconds < kMaxSampleGapSeconds) {
      const float averageMetersPerSecond = ((lastFilteredSpeedKph_ + speedKph) * 0.5f) / 3.6f;
      const float distanceMeters = std::max(0.0f, averageMetersPerSecond * elapsedSeconds);
      if (distanceMeters > 0.0f) {
        stats_.distanceMeters += distanceMeters;
        appendSegment(distanceMeters, elapsedSeconds);
        updateAverages();
      }
    }
  }

  lastUsable_ = sample;
  lastFilteredSpeedKph_ = speedKph;
  hasLastUsable_ = true;
}

void StatsTracker::appendSegment(float distanceMeters, float elapsedSeconds) {
  activeBucket_.distanceMeters += distanceMeters;
  activeBucket_.elapsedSeconds += elapsedSeconds;

  win1Km_.accumulatedDistance += distanceMeters;
  win1Km_.accumulatedElapsed += elapsedSeconds;
  win5Km_.accumulatedDistance += distanceMeters;
  win5Km_.accumulatedElapsed += elapsedSeconds;
  win10Km_.accumulatedDistance += distanceMeters;
  win10Km_.accumulatedElapsed += elapsedSeconds;
  win1Mi_.accumulatedDistance += distanceMeters;
  win1Mi_.accumulatedElapsed += elapsedSeconds;
  win3Mi_.accumulatedDistance += distanceMeters;
  win3Mi_.accumulatedElapsed += elapsedSeconds;
  win5Mi_.accumulatedDistance += distanceMeters;
  win5Mi_.accumulatedElapsed += elapsedSeconds;

  if (activeBucket_.distanceMeters >= kBucketDistanceMeters) {
    commitActiveBucket();
  }

  advanceWindowTail(win1Km_);
  advanceWindowTail(win5Km_);
  advanceWindowTail(win10Km_);
  advanceWindowTail(win1Mi_);
  advanceWindowTail(win3Mi_);
  advanceWindowTail(win5Mi_);
}

void StatsTracker::commitActiveBucket() {
  const size_t writeSeq = ringBaseSeq_ + ringCount_;
  ring_[writeSeq % kRingCapacity] = activeBucket_;
  activeBucket_ = DistanceBucket{};

  if (ringCount_ == kRingCapacity) {
    ringBaseSeq_ += 1;
    clampWindowToRing(win1Km_);
    clampWindowToRing(win5Km_);
    clampWindowToRing(win10Km_);
    clampWindowToRing(win1Mi_);
    clampWindowToRing(win3Mi_);
    clampWindowToRing(win5Mi_);
  } else {
    ringCount_ += 1;
  }
}

void StatsTracker::advanceWindowTail(SlidingWindow &window) {
  const size_t ringEndSeq = ringBaseSeq_ + ringCount_;
  while (window.tailSeq < ringEndSeq) {
    const DistanceBucket &oldest = ring_[window.tailSeq % kRingCapacity];
    if (window.accumulatedDistance - oldest.distanceMeters >= window.targetMeters) {
      window.accumulatedDistance -= oldest.distanceMeters;
      window.accumulatedElapsed -= oldest.elapsedSeconds;
      window.tailSeq += 1;
    } else {
      break;
    }
  }
}

void StatsTracker::clampWindowToRing(SlidingWindow &window) {
  if (window.tailSeq >= ringBaseSeq_) {
    return;
  }

  window.tailSeq = ringBaseSeq_;
  window.accumulatedDistance = activeBucket_.distanceMeters;
  window.accumulatedElapsed = activeBucket_.elapsedSeconds;
  const size_t ringEndSeq = ringBaseSeq_ + ringCount_;
  for (size_t seq = window.tailSeq; seq < ringEndSeq; ++seq) {
    const DistanceBucket &bucket = ring_[seq % kRingCapacity];
    window.accumulatedDistance += bucket.distanceMeters;
    window.accumulatedElapsed += bucket.elapsedSeconds;
  }
}

float StatsTracker::evaluateWindow(SlidingWindow &window) const {
  if (window.accumulatedDistance + 0.01 < window.targetMeters) {
    return NAN;
  }

  const size_t ringEndSeq = ringBaseSeq_ + ringCount_;
  const DistanceBucket &oldest =
    window.tailSeq < ringEndSeq ? ring_[window.tailSeq % kRingCapacity] : activeBucket_;

  const double excessDistance = std::max(0.0, window.accumulatedDistance - window.targetMeters);
  const double excessRatio = oldest.distanceMeters > 0.0f
                               ? std::min(1.0, excessDistance / static_cast<double>(oldest.distanceMeters))
                               : 0.0;
  const double effectiveElapsed =
    window.accumulatedElapsed - static_cast<double>(oldest.elapsedSeconds) * excessRatio;
  if (effectiveElapsed <= 0.0) {
    return NAN;
  }

  return static_cast<float>((static_cast<double>(window.targetMeters) / effectiveElapsed) * 3.6);
}

void StatsTracker::updateAverages() {
  stats_.avg1KmKph = evaluateWindow(win1Km_);
  stats_.avg5KmKph = evaluateWindow(win5Km_);
  stats_.avg10KmKph = evaluateWindow(win10Km_);
  stats_.avg1MiKph = evaluateWindow(win1Mi_);
  stats_.avg3MiKph = evaluateWindow(win3Mi_);
  stats_.avg5MiKph = evaluateWindow(win5Mi_);
}

}  // namespace dragy
