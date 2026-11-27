#pragma once

#include "dragy_core/types.h"

namespace dragy {

class SpeedDisplayFilter {
 public:
  static constexpr float kStartMovingKph = 5.0f;
  static constexpr float kStopMovingKph = 2.0f;

  void reset();
  float apply(const TelemetrySample &sample);
  bool isMoving() const { return moving_; }

 private:
  bool moving_ = false;
};

}  // namespace dragy
