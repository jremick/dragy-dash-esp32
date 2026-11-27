#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "dragy_core/types.h"

namespace dragy {

class UbxReassembler {
 public:
  static constexpr size_t kMaxBufferBytes = 4096;
  static constexpr size_t kMaxFrameBytes = 1024;

  std::vector<TelemetrySample> append(
    const uint8_t *data,
    size_t length,
    uint32_t rxTimestampMs
  );

  void reset();
  size_t bufferedBytes() const { return buffer_.size(); }

 private:
  void drain(uint32_t rxTimestampMs, std::vector<TelemetrySample> &samples);
  int firstSyncIndex() const;

  std::vector<uint8_t> buffer_;
};

}  // namespace dragy
