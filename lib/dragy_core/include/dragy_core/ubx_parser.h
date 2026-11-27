#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "dragy_core/types.h"

namespace dragy {

uint16_t u16LE(const uint8_t *bytes);
uint32_t u32LE(const uint8_t *bytes);
int32_t i32LE(const uint8_t *bytes);

bool validateUbxChecksum(const uint8_t *frame, size_t frameLength);

bool parseUbxNavPvt(
  const uint8_t *frame,
  size_t frameLength,
  uint32_t rxTimestampMs,
  TelemetrySample &sample
);

std::array<uint8_t, 4> buildFd03HandshakeResponse(uint8_t challengeA, uint8_t challengeB);

bool parseFd04BatteryPercent(const uint8_t *data, size_t length, int &batteryPercent);

bool looksLikeDragyDevice(const std::string &name, bool advertisesFd00Service);

}  // namespace dragy
