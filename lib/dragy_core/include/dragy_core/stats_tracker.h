#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "dragy_core/types.h"

namespace dragy {

class StatsTracker {
 public:
  // 1,600 x 10 m distance buckets = 16 km fixed-RAM circular ring (~12.8 KB static, zero heap).
  static constexpr float kBucketDistanceMeters = 10.0f;
  static constexpr size_t kRingCapacity = 1600;
  static constexpr float kMaxSampleGapSeconds = 10.0f;

  void resetStats();

  // Record a decoded sample alongside its stationary-filtered speed (in km/h) so parked
  // GPS jitter does not pollute peak speed, total distance, or rolling averages.
  void record(const TelemetrySample &sample, float filteredSpeedKph);

  const TelemetrySample &current() const { return current_; }
  bool hasCurrent() const { return currentValid_; }
  const RunStats &stats() const { return stats_; }

 private:
  struct DistanceBucket {
    float distanceMeters = 0.0f;
    float elapsedSeconds = 0.0f;
  };

  struct SlidingWindow {
    float targetMeters = 0.0f;
    size_t tailSeq = 0;
    double accumulatedDistance = 0.0;
    double accumulatedElapsed = 0.0;

    void reset() {
      tailSeq = 0;
      accumulatedDistance = 0.0;
      accumulatedElapsed = 0.0;
    }
  };

  static float elapsedSecondsBetween(const TelemetrySample &previous, const TelemetrySample &current);

  void appendSegment(float distanceMeters, float elapsedSeconds);
  void commitActiveBucket();
  float evaluateWindow(SlidingWindow &window) const;
  void advanceWindowTail(SlidingWindow &window);
  void clampWindowToRing(SlidingWindow &window);
  void updateAverages();

  TelemetrySample current_;
  TelemetrySample lastUsable_;
  float lastFilteredSpeedKph_ = 0.0f;
  RunStats stats_;

  std::array<DistanceBucket, kRingCapacity> ring_{};
  size_t ringBaseSeq_ = 0;
  size_t ringCount_ = 0;
  DistanceBucket activeBucket_{};

  SlidingWindow win1Km_{1000.0f};
  SlidingWindow win5Km_{5000.0f};
  SlidingWindow win10Km_{10000.0f};
  SlidingWindow win1Mi_{1609.344f};
  SlidingWindow win3Mi_{4828.032f};
  SlidingWindow win5Mi_{8046.72f};

  bool currentValid_ = false;
  bool hasLastUsable_ = false;
};

}  // namespace dragy
