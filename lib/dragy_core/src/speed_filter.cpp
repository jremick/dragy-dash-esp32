#include "dragy_core/speed_filter.h"

namespace dragy {

void SpeedDisplayFilter::reset() {
  moving_ = false;
}

float SpeedDisplayFilter::apply(const TelemetrySample &sample) {
  if (!sample.usable()) {
    moving_ = false;
    return 0.0f;
  }

  if (moving_) {
    if (sample.speedKph <= kStopMovingKph) {
      moving_ = false;
    }
  } else if (sample.speedKph >= kStartMovingKph) {
    moving_ = true;
  }

  return moving_ ? sample.speedKph : 0.0f;
}

}  // namespace dragy
