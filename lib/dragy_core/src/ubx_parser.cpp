#include "dragy_core/ubx_parser.h"

#include <algorithm>
#include <cctype>

namespace dragy {

uint16_t u16LE(const uint8_t *bytes) {
  return static_cast<uint16_t>(bytes[0]) |
         (static_cast<uint16_t>(bytes[1]) << 8);
}

uint32_t u32LE(const uint8_t *bytes) {
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) |
         (static_cast<uint32_t>(bytes[3]) << 24);
}

int32_t i32LE(const uint8_t *bytes) {
  return static_cast<int32_t>(u32LE(bytes));
}

bool validateUbxChecksum(const uint8_t *frame, size_t frameLength) {
  if (frame == nullptr || frameLength < 8) {
    return false;
  }

  uint8_t checksumA = 0;
  uint8_t checksumB = 0;
  for (size_t index = 2; index < frameLength - 2; ++index) {
    checksumA = static_cast<uint8_t>(checksumA + frame[index]);
    checksumB = static_cast<uint8_t>(checksumB + checksumA);
  }

  return checksumA == frame[frameLength - 2] &&
         checksumB == frame[frameLength - 1];
}

bool parseUbxNavPvt(
  const uint8_t *frame,
  size_t frameLength,
  uint32_t rxTimestampMs,
  TelemetrySample &sample
) {
  if (frame == nullptr || frameLength != 100 ||
      frame[0] != 0xB5 || frame[1] != 0x62 ||
      frame[2] != 0x01 || frame[3] != 0x07) {
    return false;
  }

  const uint16_t payloadLength = u16LE(frame + 4);
  if (payloadLength != 92 || !validateUbxChecksum(frame, frameLength)) {
    return false;
  }

  const uint8_t *payload = frame + 6;
  const uint32_t iTOW = u32LE(payload);
  const FixQuality quality = fixQualityForUbx(payload[20]);
  const uint8_t satelliteCount = payload[23];
  const int32_t gSpeedMmPerSecond = i32LE(payload + 60);

  sample.timestampMs = rxTimestampMs;
  sample.gpsTimeOfWeekMs = iTOW;
  sample.hasGpsTimeOfWeek = true;
  sample.fixQuality = quality;
  sample.satelliteCount = satelliteCount;
  sample.altitudeMeters = static_cast<float>(i32LE(payload + 36)) / 1000.0f;
  sample.horizontalAccuracyMeters = static_cast<float>(u32LE(payload + 40)) / 1000.0f;
  sample.speedKph = sample.usable()
                      ? std::max(0.0f, static_cast<float>(gSpeedMmPerSecond) * 3.6f / 1000.0f)
                      : 0.0f;
  sample.headingDegrees = static_cast<float>(i32LE(payload + 64)) * 0.00001f;
  return true;
}

std::array<uint8_t, 4> buildFd03HandshakeResponse(uint8_t challengeA, uint8_t challengeB) {
  return {
    challengeA,
    challengeB,
    static_cast<uint8_t>(challengeA ^ challengeB),
    static_cast<uint8_t>(challengeA & challengeB)
  };
}

bool parseFd04BatteryPercent(const uint8_t *data, size_t length, int &batteryPercent) {
  if (data == nullptr || length < 2 || data[1] > 100) {
    return false;
  }

  batteryPercent = static_cast<int>(data[1]);
  return true;
}

bool looksLikeDragyDevice(const std::string &name, bool advertisesFd00Service) {
  if (advertisesFd00Service) {
    return true;
  }

  std::string lower = name;
  for (char &ch : lower) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }

  return lower.rfind("drg", 0) == 0 || lower.find("dragy") != std::string::npos;
}

}  // namespace dragy
