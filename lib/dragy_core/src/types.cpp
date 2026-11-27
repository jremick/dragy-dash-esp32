#include "dragy_core/types.h"

#include <cctype>

namespace dragy {

const char *fixLabel(FixQuality quality) {
  switch (quality) {
    case FixQuality::TwoD:
      return "2D";
    case FixQuality::ThreeD:
      return "3D";
    case FixQuality::Estimated:
      return "EST";
    case FixQuality::None:
    default:
      return "NO FIX";
  }
}

GpsQuality gpsQualityFor(const TelemetrySample &sample) {
  if (!sample.usable() || !std::isfinite(sample.horizontalAccuracyMeters)) {
    return GpsQuality::None;
  }

  if (sample.fixQuality == FixQuality::ThreeD &&
      sample.satelliteCount >= 8 &&
      sample.horizontalAccuracyMeters <= 2.5f) {
    return GpsQuality::Good;
  }

  if ((sample.fixQuality == FixQuality::ThreeD || sample.fixQuality == FixQuality::Estimated) &&
      sample.satelliteCount >= 6 &&
      sample.horizontalAccuracyMeters <= 5.0f) {
    return GpsQuality::Fair;
  }

  return GpsQuality::Poor;
}

FixQuality fixQualityForUbx(uint8_t fixType) {
  switch (fixType) {
    case 0:
    case 1:
      return FixQuality::None;
    case 2:
      return FixQuality::TwoD;
    case 3:
    case 4:
      return FixQuality::ThreeD;
    default:
      return FixQuality::Estimated;
  }
}

std::string redactDeviceName(const std::string &name) {
  if (name.empty()) {
    return "<unnamed>";
  }

  const size_t dash = name.find('-');
  if (dash != std::string::npos && dash >= 3 && dash + 1 < name.size()) {
    std::string prefix = name.substr(0, dash);
    std::string upperPrefix = prefix;
    for (char &ch : upperPrefix) {
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    if (upperPrefix.rfind("DRG", 0) == 0) {
      return prefix + "-******";
    }
  }

  return name;
}

}  // namespace dragy
