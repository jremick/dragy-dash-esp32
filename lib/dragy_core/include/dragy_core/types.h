#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace dragy {

enum class FixQuality : uint8_t {
  None,
  TwoD,
  ThreeD,
  Estimated,
};

enum class GpsQuality : uint8_t {
  None,
  Poor,
  Fair,
  Good,
};

struct TelemetrySample {
  uint32_t timestampMs = 0;
  uint32_t gpsTimeOfWeekMs = 0;
  bool hasGpsTimeOfWeek = false;
  float speedKph = 0.0f;
  float headingDegrees = NAN;
  float altitudeMeters = NAN;
  float horizontalAccuracyMeters = NAN;
  uint8_t satelliteCount = 0;
  FixQuality fixQuality = FixQuality::None;

  bool usable() const {
    return fixQuality != FixQuality::None && satelliteCount > 0;
  }
};

struct RunStats {
  float peakSpeedKph = 0.0f;
  float distanceMeters = 0.0f;
  float avg1KmKph = NAN;
  float avg5KmKph = NAN;
  float avg10KmKph = NAN;
  float avg1MiKph = NAN;
  float avg3MiKph = NAN;
  float avg5MiKph = NAN;
};

constexpr float kKphToMph = 0.621371f;
constexpr float kMetersToFeet = 3.28084f;
constexpr float kMetersPerMile = 1609.344f;

const char *fixLabel(FixQuality quality);
GpsQuality gpsQualityFor(const TelemetrySample &sample);
FixQuality fixQualityForUbx(uint8_t fixType);
std::string redactDeviceName(const std::string &name);

}  // namespace dragy
