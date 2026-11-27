#include <unity.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "dragy_core/speed_filter.h"
#include "dragy_core/stats_tracker.h"
#include "dragy_core/types.h"
#include "dragy_core/ubx_parser.h"
#include "dragy_core/ubx_reassembler.h"

namespace {

void writeU16LE(std::vector<uint8_t> &buf, size_t offset, uint16_t value) {
  buf[offset] = static_cast<uint8_t>(value & 0xFFu);
  buf[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

void writeU32LE(std::vector<uint8_t> &buf, size_t offset, uint32_t value) {
  buf[offset] = static_cast<uint8_t>(value & 0xFFu);
  buf[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
  buf[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
  buf[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
}

void writeI32LE(std::vector<uint8_t> &buf, size_t offset, int32_t value) {
  writeU32LE(buf, offset, static_cast<uint32_t>(value));
}

std::vector<uint8_t> makeNavPvtFrame(
  uint32_t iTOW,
  uint8_t fixType,
  uint8_t sats,
  int32_t altitudeMm,
  uint32_t hAccMm,
  int32_t gSpeedMmPerSec,
  int32_t headingScaled
) {
  std::vector<uint8_t> frame(100, 0);
  frame[0] = 0xB5;
  frame[1] = 0x62;
  frame[2] = 0x01;
  frame[3] = 0x07;
  writeU16LE(frame, 4, 92);

  constexpr size_t p = 6;
  writeU32LE(frame, p + 0, iTOW);
  frame[p + 20] = fixType;
  frame[p + 23] = sats;
  writeI32LE(frame, p + 36, altitudeMm);
  writeU32LE(frame, p + 40, hAccMm);
  writeI32LE(frame, p + 60, gSpeedMmPerSec);
  writeI32LE(frame, p + 64, headingScaled);

  uint8_t ckA = 0;
  uint8_t ckB = 0;
  for (size_t i = 2; i < 98; ++i) {
    ckA = static_cast<uint8_t>(ckA + frame[i]);
    ckB = static_cast<uint8_t>(ckB + ckA);
  }
  frame[98] = ckA;
  frame[99] = ckB;
  return frame;
}

void test_parse_valid_ubx_nav_pvt() {
  auto frame = makeNavPvtFrame(123456, 3, 18, 51200, 900, 10000, 9000000);
  dragy::TelemetrySample sample;

  TEST_ASSERT_TRUE(dragy::parseUbxNavPvt(frame.data(), frame.size(), 5000, sample));
  TEST_ASSERT_EQUAL_UINT32(5000, sample.timestampMs);
  TEST_ASSERT_TRUE(sample.hasGpsTimeOfWeek);
  TEST_ASSERT_EQUAL_UINT32(123456, sample.gpsTimeOfWeekMs);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(dragy::FixQuality::ThreeD), static_cast<int>(sample.fixQuality));
  TEST_ASSERT_EQUAL_UINT8(18, sample.satelliteCount);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 36.0f, sample.speedKph);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 51.2f, sample.altitudeMeters);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.9f, sample.horizontalAccuracyMeters);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f, sample.headingDegrees);
  TEST_ASSERT_EQUAL_INT(
    static_cast<int>(dragy::GpsQuality::Good),
    static_cast<int>(dragy::gpsQualityFor(sample))
  );
}

void test_rejects_bad_ubx_checksum() {
  auto frame = makeNavPvtFrame(1000, 3, 12, 10000, 1000, 10000, 0);
  frame[99] ^= 0xFF;
  dragy::TelemetrySample sample;
  TEST_ASSERT_FALSE(dragy::parseUbxNavPvt(frame.data(), frame.size(), 1000, sample));
}

void test_zeros_speed_without_usable_fix_or_satellites() {
  auto noFix = makeNavPvtFrame(1000, 0, 10, 10000, 1000, 20000, 0);
  dragy::TelemetrySample s1;
  TEST_ASSERT_TRUE(dragy::parseUbxNavPvt(noFix.data(), noFix.size(), 1000, s1));
  TEST_ASSERT_FALSE(s1.usable());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s1.speedKph);

  auto zeroSats = makeNavPvtFrame(2000, 3, 0, 10000, 1000, 20000, 0);
  dragy::TelemetrySample s2;
  TEST_ASSERT_TRUE(dragy::parseUbxNavPvt(zeroSats.data(), zeroSats.size(), 2000, s2));
  TEST_ASSERT_FALSE(s2.usable());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s2.speedKph);
}

void test_reassembler_handles_fragments_and_false_sync() {
  auto valid = makeNavPvtFrame(50000, 3, 14, 42000, 1200, 25000, 18000000);
  // False B5 62 sync claiming a 92-byte frame immediately followed by the real frame.
  std::vector<uint8_t> stream = {0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00, 0xDE, 0xAD};
  stream.insert(stream.end(), valid.begin(), valid.end());

  dragy::UbxReassembler reassembler;
  std::vector<dragy::TelemetrySample> decoded;
  for (size_t offset = 0; offset < stream.size(); offset += 13) {
    const size_t chunk = std::min<size_t>(13, stream.size() - offset);
    auto batch = reassembler.append(stream.data() + offset, chunk, 1234);
    decoded.insert(decoded.end(), batch.begin(), batch.end());
  }

  TEST_ASSERT_EQUAL_UINT32(1, decoded.size());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 90.0f, decoded[0].speedKph);
}

void test_fd03_handshake_and_fd04_battery() {
  const auto resp = dragy::buildFd03HandshakeResponse(0xA5, 0x3C);
  TEST_ASSERT_EQUAL_HEX8(0xA5, resp[0]);
  TEST_ASSERT_EQUAL_HEX8(0x3C, resp[1]);
  TEST_ASSERT_EQUAL_HEX8(0x99, resp[2]);
  TEST_ASSERT_EQUAL_HEX8(0x24, resp[3]);

  int percent = -1;
  const uint8_t validBat[2] = {0x01, 0x54};
  TEST_ASSERT_TRUE(dragy::parseFd04BatteryPercent(validBat, 2, percent));
  TEST_ASSERT_EQUAL_INT(84, percent);

  const uint8_t invalidBat[2] = {0x01, 0xFE};
  TEST_ASSERT_FALSE(dragy::parseFd04BatteryPercent(invalidBat, 2, percent));
  TEST_ASSERT_FALSE(dragy::parseFd04BatteryPercent(validBat, 1, percent));
}

void test_device_matching_and_redaction() {
  TEST_ASSERT_TRUE(dragy::looksLikeDragyDevice("DRG70-123456", false));
  TEST_ASSERT_TRUE(dragy::looksLikeDragyDevice("drg70-abcdef", false));
  TEST_ASSERT_TRUE(dragy::looksLikeDragyDevice("My dragy pro", false));
  TEST_ASSERT_TRUE(dragy::looksLikeDragyDevice("Unnamed", true));
  TEST_ASSERT_FALSE(dragy::looksLikeDragyDevice("Random Speaker", false));

  TEST_ASSERT_EQUAL_STRING("DRG70-******", dragy::redactDeviceName("DRG70-9A1B2C").c_str());
  TEST_ASSERT_EQUAL_STRING("<unnamed>", dragy::redactDeviceName("").c_str());
  TEST_ASSERT_EQUAL_STRING("Dragy Pro", dragy::redactDeviceName("Dragy Pro").c_str());
}

void test_speed_filter_hysteresis_and_no_fix() {
  dragy::SpeedDisplayFilter filter;
  dragy::TelemetrySample sample;
  sample.fixQuality = dragy::FixQuality::ThreeD;
  sample.satelliteCount = 12;

  sample.speedKph = 4.2f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, filter.apply(sample));

  sample.speedKph = 5.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, filter.apply(sample));

  sample.speedKph = 2.5f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.5f, filter.apply(sample));

  sample.speedKph = 2.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, filter.apply(sample));

  sample.speedKph = 30.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, filter.apply(sample));

  sample.fixQuality = dragy::FixQuality::None;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, filter.apply(sample));
  TEST_ASSERT_FALSE(filter.isMoving());
}

void test_stats_tracker_rolling_averages_and_ring_wraparound() {
  dragy::StatsTracker tracker;
  dragy::TelemetrySample sample;
  sample.fixQuality = dragy::FixQuality::ThreeD;
  sample.satelliteCount = 14;
  sample.horizontalAccuracyMeters = 1.2f;
  sample.hasGpsTimeOfWeek = true;

  // Drive 20 km at 100 km/h (27.7778 m/s) in 1-second ticks (720 ticks -> exceeds 16 km ring!).
  for (uint32_t sec = 0; sec <= 720; ++sec) {
    sample.timestampMs = sec * 1000u;
    sample.gpsTimeOfWeekMs = 100000u + sec * 1000u;
    sample.speedKph = 100.0f;
    tracker.record(sample, 100.0f);
  }

  TEST_ASSERT_FLOAT_WITHIN(1.0f, 20000.0f, tracker.stats().distanceMeters);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 100.0f, tracker.stats().peakSpeedKph);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.0f, tracker.stats().avg1KmKph);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.0f, tracker.stats().avg5KmKph);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.0f, tracker.stats().avg10KmKph);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.0f, tracker.stats().avg1MiKph);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.0f, tracker.stats().avg3MiKph);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.0f, tracker.stats().avg5MiKph);

  // Now accelerate to 144 km/h (40 m/s) for 2 km (50 seconds) -> 1 km avg should climb to 144 km/h.
  for (uint32_t sec = 721; sec <= 771; ++sec) {
    sample.timestampMs = sec * 1000u;
    sample.gpsTimeOfWeekMs = 100000u + sec * 1000u;
    sample.speedKph = 144.0f;
    tracker.record(sample, 144.0f);
  }

  TEST_ASSERT_FLOAT_WITHIN(0.5f, 144.0f, tracker.stats().avg1KmKph);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 144.0f, tracker.stats().peakSpeedKph);
}

void test_stats_tracker_ignores_stationary_filtered_jitter() {
  dragy::StatsTracker tracker;
  dragy::SpeedDisplayFilter filter;
  dragy::TelemetrySample sample;
  sample.fixQuality = dragy::FixQuality::ThreeD;
  sample.satelliteCount = 12;

  for (uint32_t sec = 0; sec < 10; ++sec) {
    sample.timestampMs = sec * 1000u;
    sample.speedKph = 3.5f;
    const float filtered = filter.apply(sample);
    tracker.record(sample, filtered);
  }

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, tracker.stats().peakSpeedKph);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, tracker.stats().distanceMeters);
}

void test_stats_tracker_includes_stationary_time_in_rolling_averages() {
  dragy::StatsTracker tracker;
  dragy::TelemetrySample sample;
  sample.fixQuality = dragy::FixQuality::ThreeD;
  sample.satelliteCount = 12;
  sample.hasGpsTimeOfWeek = true;

  // Drive 1,000 m in 100 s at 36 km/h (10 m/s), then sit stationary for 100 s.
  for (uint32_t sec = 0; sec <= 100; ++sec) {
    sample.timestampMs = sec * 1000u;
    sample.gpsTimeOfWeekMs = 10000u + sec * 1000u;
    sample.speedKph = 36.0f;
    tracker.record(sample, 36.0f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 36.0f, tracker.stats().avg1KmKph);

  for (uint32_t sec = 101; sec <= 200; ++sec) {
    sample.timestampMs = sec * 1000u;
    sample.gpsTimeOfWeekMs = 10000u + sec * 1000u;
    sample.speedKph = 0.0f;
    tracker.record(sample, 0.0f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 18.0f, tracker.stats().avg1KmKph);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_parse_valid_ubx_nav_pvt);
  RUN_TEST(test_rejects_bad_ubx_checksum);
  RUN_TEST(test_zeros_speed_without_usable_fix_or_satellites);
  RUN_TEST(test_reassembler_handles_fragments_and_false_sync);
  RUN_TEST(test_fd03_handshake_and_fd04_battery);
  RUN_TEST(test_device_matching_and_redaction);
  RUN_TEST(test_speed_filter_hysteresis_and_no_fix);
  RUN_TEST(test_stats_tracker_rolling_averages_and_ring_wraparound);
  RUN_TEST(test_stats_tracker_ignores_stationary_filtered_jitter);
  RUN_TEST(test_stats_tracker_includes_stationary_time_in_rolling_averages);
  return UNITY_END();
}
