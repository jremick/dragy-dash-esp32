#include "dragy_core/ubx_reassembler.h"

#include "dragy_core/ubx_parser.h"

namespace dragy {

std::vector<TelemetrySample> UbxReassembler::append(
  const uint8_t *data,
  size_t length,
  uint32_t rxTimestampMs
) {
  std::vector<TelemetrySample> samples;
  if (data == nullptr || length == 0) {
    return samples;
  }

  if (length >= kMaxBufferBytes) {
    buffer_.assign(data + (length - kMaxBufferBytes), data + length);
  } else {
    if (buffer_.size() + length > kMaxBufferBytes) {
      const size_t drop = buffer_.size() + length - kMaxBufferBytes;
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(drop));
    }
    buffer_.insert(buffer_.end(), data, data + length);
  }

  drain(rxTimestampMs, samples);
  return samples;
}

void UbxReassembler::reset() {
  buffer_.clear();
}

void UbxReassembler::drain(uint32_t rxTimestampMs, std::vector<TelemetrySample> &samples) {
  while (true) {
    const int sync = firstSyncIndex();
    if (sync < 0) {
      if (buffer_.size() > 1) {
        const uint8_t last = buffer_.back();
        buffer_.clear();
        buffer_.push_back(last);
      }
      return;
    }

    if (sync > 0) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + sync);
    }

    if (buffer_.size() < 6) {
      return;
    }

    const uint16_t payloadLength = u16LE(buffer_.data() + 4);
    const size_t frameLength = 6u + static_cast<size_t>(payloadLength) + 2u;
    if (frameLength > kMaxFrameBytes) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
      continue;
    }

    if (buffer_.size() < frameLength) {
      return;
    }

    if (!validateUbxChecksum(buffer_.data(), frameLength)) {
      // False sync or corrupt frame: advance past the 2-byte sync marker so any
      // valid UBX frame starting inside those bytes is not swallowed.
      buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
      continue;
    }

    TelemetrySample sample;
    if (parseUbxNavPvt(buffer_.data(), frameLength, rxTimestampMs, sample)) {
      samples.push_back(sample);
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frameLength));
  }
}

int UbxReassembler::firstSyncIndex() const {
  if (buffer_.size() < 2) {
    return -1;
  }

  for (size_t index = 0; index < buffer_.size() - 1; ++index) {
    if (buffer_[index] == 0xB5 && buffer_[index + 1] == 0x62) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

}  // namespace dragy
