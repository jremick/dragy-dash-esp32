#include <Arduino.h>
#include <NimBLEDevice.h>
#include <TFT_eSPI.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace {

constexpr uint8_t kButtonPrevPin = 0;
constexpr uint8_t kButtonNextPin = 14;
constexpr uint8_t kDisplayPowerPin = 15;
constexpr uint8_t kBacklightPin = 38;

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kBleScanMs = 5000;
constexpr uint32_t kBleRetryMs = 6000;
constexpr uint32_t kRenderIntervalMs = 100;
constexpr uint32_t kButtonDebounceMs = 35;
constexpr uint32_t kButtonLongPressMs = 900;
constexpr uint32_t kBothButtonHoldMs = 1200;
constexpr uint32_t kActionMessageMs = 1600;
constexpr uint32_t kStreamStartTimeoutMs = 8000;
constexpr uint32_t kPacketStallMs = 4000;
constexpr uint8_t kPageCount = 3;
constexpr float kAverageWindowMeters = 15000.0f;
constexpr size_t kMaxAverageSegments = 12000;
constexpr uint8_t kBrightnessLevels[] = {4, 10, 16};
constexpr const char *kBrightnessLabels[] = {"DIM", "MID", "HIGH"};
constexpr uint8_t kBrightnessModeCount = 3;

const NimBLEUUID kDragyServiceUuid("fd00");
const NimBLEUUID kFd02Uuid("fd02");
const NimBLEUUID kFd03Uuid("fd03");
const NimBLEUUID kFd04Uuid("fd04");

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

struct DistanceSegment {
  float distanceMeters = 0.0f;
  float elapsedSeconds = 0.0f;
};

struct RunStats {
  float peakSpeedKph = 0.0f;
  float distanceMeters = 0.0f;
  float avg1KmKph = NAN;
  float avg5KmKph = NAN;
  float avg10KmKph = NAN;
};

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

bool validateUbxChecksum(const uint8_t *frame, size_t frameLength) {
  if (frameLength < 8) {
    return false;
  }

  uint8_t checksumA = 0;
  uint8_t checksumB = 0;
  for (size_t index = 2; index < frameLength - 2; ++index) {
    checksumA += frame[index];
    checksumB += checksumA;
  }

  return checksumA == frame[frameLength - 2] &&
         checksumB == frame[frameLength - 1];
}

bool parseUbxNavPvt(const uint8_t *frame, size_t frameLength, TelemetrySample &sample) {
  if (frameLength != 100 || frame[0] != 0xB5 || frame[1] != 0x62 ||
      frame[2] != 0x01 || frame[3] != 0x07) {
    return false;
  }

  const uint16_t payloadLength = u16LE(frame + 4);
  if (payloadLength != 92 || !validateUbxChecksum(frame, frameLength)) {
    return false;
  }

  const uint8_t *payload = frame + 6;
  const FixQuality quality = fixQualityForUbx(payload[20]);
  const int32_t gSpeedMmPerSecond = i32LE(payload + 60);

  sample.timestampMs = millis();
  sample.fixQuality = quality;
  sample.satelliteCount = payload[23];
  sample.altitudeMeters = static_cast<float>(i32LE(payload + 36)) / 1000.0f;
  sample.horizontalAccuracyMeters = static_cast<float>(u32LE(payload + 40)) / 1000.0f;
  sample.speedKph = quality == FixQuality::None
                      ? 0.0f
                      : std::max(0.0f, static_cast<float>(gSpeedMmPerSecond) * 3.6f / 1000.0f);
  sample.headingDegrees = static_cast<float>(i32LE(payload + 64)) * 0.00001f;
  return true;
}

class UbxReassembler {
 public:
  std::vector<TelemetrySample> append(const uint8_t *data, size_t length) {
    buffer_.insert(buffer_.end(), data, data + length);

    std::vector<TelemetrySample> samples;
    drain(samples);
    return samples;
  }

  void reset() {
    buffer_.clear();
  }

 private:
  void drain(std::vector<TelemetrySample> &samples) {
    while (true) {
      const auto sync = firstSyncIndex();
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
      const size_t frameLength = 6 + payloadLength + 2;
      if (frameLength > 1024) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
        continue;
      }

      if (buffer_.size() < frameLength) {
        return;
      }

      TelemetrySample sample;
      if (parseUbxNavPvt(buffer_.data(), frameLength, sample)) {
        samples.push_back(sample);
      }
      buffer_.erase(buffer_.begin(), buffer_.begin() + frameLength);
    }
  }

  int firstSyncIndex() const {
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

  std::vector<uint8_t> buffer_;
};

class StatsTracker {
 public:
  void resetStats() {
    stats_ = RunStats{};
    segments_.clear();
    averageWindowDistanceMeters_ = 0.0f;
    hasLastUsable_ = false;
  }

  void record(const TelemetrySample &sample) {
    current_ = sample;
    currentValid_ = true;

    if (!sample.usable()) {
      hasLastUsable_ = false;
      return;
    }

    stats_.peakSpeedKph = std::max(stats_.peakSpeedKph, sample.speedKph);

    if (hasLastUsable_) {
      const float elapsedSeconds =
        static_cast<float>(sample.timestampMs - lastUsable_.timestampMs) / 1000.0f;
      if (elapsedSeconds > 0.0f && elapsedSeconds < 10.0f) {
        const float averageMetersPerSecond =
          ((lastUsable_.speedKph + sample.speedKph) / 2.0f) / 3.6f;
        const float distanceMeters = std::max(0.0f, averageMetersPerSecond * elapsedSeconds);
        if (distanceMeters > 0.0f) {
          stats_.distanceMeters += distanceMeters;
          appendSegment(distanceMeters, elapsedSeconds);
          updateAverages();
        }
      }
    }

    lastUsable_ = sample;
    hasLastUsable_ = true;
  }

  const TelemetrySample &current() const {
    return current_;
  }

  bool hasCurrent() const {
    return currentValid_;
  }

  const RunStats &stats() const {
    return stats_;
  }

 private:
  void appendSegment(float distanceMeters, float elapsedSeconds) {
    DistanceSegment segment;
    segment.distanceMeters = distanceMeters;
    segment.elapsedSeconds = elapsedSeconds;
    segments_.push_back(segment);
    averageWindowDistanceMeters_ += distanceMeters;
    pruneDistanceWindow();
    compactIfNeeded();
  }

  void pruneDistanceWindow() {
    while (!segments_.empty() && averageWindowDistanceMeters_ > kAverageWindowMeters) {
      const float overflow = averageWindowDistanceMeters_ - kAverageWindowMeters;
      DistanceSegment &front = segments_.front();
      if (overflow >= front.distanceMeters) {
        averageWindowDistanceMeters_ -= front.distanceMeters;
        segments_.erase(segments_.begin());
        continue;
      }

      const float ratio = overflow / front.distanceMeters;
      front.distanceMeters -= overflow;
      front.elapsedSeconds *= (1.0f - ratio);
      averageWindowDistanceMeters_ = kAverageWindowMeters;
      return;
    }
  }

  void compactIfNeeded() {
    while (segments_.size() > kMaxAverageSegments && segments_.size() >= 2) {
      segments_[1].distanceMeters += segments_[0].distanceMeters;
      segments_[1].elapsedSeconds += segments_[0].elapsedSeconds;
      segments_.erase(segments_.begin());
    }
  }

  float averageSpeedOver(float targetDistanceMeters) const {
    if (averageWindowDistanceMeters_ + 0.01f < targetDistanceMeters) {
      return NAN;
    }

    float remainingDistance = targetDistanceMeters;
    float elapsedSeconds = 0.0f;

    for (auto it = segments_.rbegin(); it != segments_.rend(); ++it) {
      if (it->distanceMeters <= 0.0f || it->elapsedSeconds <= 0.0f) {
        continue;
      }

      if (it->distanceMeters >= remainingDistance) {
        elapsedSeconds += it->elapsedSeconds * (remainingDistance / it->distanceMeters);
        return elapsedSeconds > 0.0f ? targetDistanceMeters / elapsedSeconds * 3.6f : NAN;
      }

      remainingDistance -= it->distanceMeters;
      elapsedSeconds += it->elapsedSeconds;
    }

    return NAN;
  }

  void updateAverages() {
    stats_.avg1KmKph = averageSpeedOver(1000.0f);
    stats_.avg5KmKph = averageSpeedOver(5000.0f);
    stats_.avg10KmKph = averageSpeedOver(10000.0f);
  }

  TelemetrySample current_;
  TelemetrySample lastUsable_;
  RunStats stats_;
  std::vector<DistanceSegment> segments_;
  float averageWindowDistanceMeters_ = 0.0f;
  bool currentValid_ = false;
  bool hasLastUsable_ = false;
};

class SpeedDisplayFilter {
 public:
  void reset() {
    moving_ = false;
  }

  float apply(const TelemetrySample &sample) {
    if (!sample.usable()) {
      moving_ = false;
      return 0.0f;
    }

    if (moving_) {
      if (sample.speedKph <= 2.0f) {
        moving_ = false;
      }
    } else if (sample.speedKph >= 5.0f) {
      moving_ = true;
    }

    return moving_ ? sample.speedKph : 0.0f;
  }

 private:
  bool moving_ = false;
};

class DragyBleClient {
 public:
  void begin() {
    activeClient_ = this;
    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  }

  void loop() {
    if (paused_) {
      if (client_ != nullptr && client_->isConnected()) {
        client_->disconnect();
      }
      status_ = "PAUSED";
      return;
    }

    if (client_ != nullptr && client_->isConnected()) {
      updateStreamStatus();
      return;
    }

    const uint32_t now = millis();
    if (now - lastScanMs_ < kBleRetryMs) {
      return;
    }

    lastScanMs_ = now;
    connect();
  }

  const char *status() const {
    return status_;
  }

  const std::string &deviceName() const {
    return deviceName_;
  }

  int batteryPercent() const {
    return batteryPercent_;
  }

  bool isConnected() const {
    return client_ != nullptr && client_->isConnected();
  }

  bool isPaused() const {
    return paused_;
  }

  void resetRunStats() {
    telemetry_.resetStats();
  }

  void disconnectAndPause() {
    paused_ = true;
    status_ = "PAUSED";
    reassembler_.reset();
    lastPacketMs_ = 0;
    connectedAtMs_ = 0;
    if (client_ != nullptr && client_->isConnected()) {
      client_->disconnect();
    }
    Serial.println("Dragy BLE paused.");
  }

  void resumeScanning() {
    paused_ = false;
    status_ = "SCAN";
    lastScanMs_ = 0;
    Serial.println("Dragy BLE scanning resumed.");
  }

 private:
  static bool looksLikeDragy(NimBLEAdvertisedDevice &device) {
    if (device.isAdvertisingService(kDragyServiceUuid)) {
      return true;
    }

    const std::string name = device.getName();
    return name.rfind("DRG", 0) == 0 || name.find("Dragy") != std::string::npos;
  }

  void connect() {
    status_ = "SCAN";
    Serial.println("Scanning for Dragy FD00/DRG advertisements...");

    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);
    scan->setInterval(45);
    scan->setWindow(30);
    NimBLEScanResults results = scan->start(kBleScanMs / 1000, false);

    NimBLEAdvertisedDevice candidate;
    bool hasCandidate = false;
    for (int index = 0; index < results.getCount(); ++index) {
      NimBLEAdvertisedDevice device = results.getDevice(index);
      if (looksLikeDragy(device)) {
        candidate = device;
        hasCandidate = true;
        break;
      }
    }

    if (!hasCandidate) {
      status_ = "NO DRAGY";
      Serial.println("No Dragy candidate found.");
      scan->clearResults();
      return;
    }

    deviceName_ = candidate.getName();
    status_ = "CONNECT";
    Serial.printf("Connecting to %s...\n", deviceName_.empty() ? "<unnamed>" : deviceName_.c_str());

    if (client_ == nullptr) {
      client_ = NimBLEDevice::createClient();
    }

    if (!client_->connect(&candidate)) {
      status_ = "CONNECT ERR";
      Serial.println("Dragy connection failed.");
      scan->clearResults();
      return;
    }

    NimBLERemoteService *service = client_->getService(kDragyServiceUuid);
    if (service == nullptr) {
      status_ = "NO FD00";
      Serial.println("FD00 service not found.");
      client_->disconnect();
      scan->clearResults();
      return;
    }

    NimBLERemoteCharacteristic *fd02 = service->getCharacteristic(kFd02Uuid);
    NimBLERemoteCharacteristic *fd03 = service->getCharacteristic(kFd03Uuid);
    NimBLERemoteCharacteristic *fd04 = service->getCharacteristic(kFd04Uuid);

    if (fd02 == nullptr || fd03 == nullptr) {
      status_ = "NO CHAR";
      Serial.println("Required FD02/FD03 characteristics not found.");
      client_->disconnect();
      scan->clearResults();
      return;
    }

    if (fd02->canNotify() && !fd02->subscribe(true, onFd02Notify)) {
      status_ = "FD02 SUB ERR";
      Serial.println("FD02 notify subscription failed.");
      client_->disconnect();
      scan->clearResults();
      return;
    }

    if (fd04 != nullptr) {
      if (fd04->canNotify()) {
        fd04->subscribe(true, onFd04Notify);
      }
      if (fd04->canRead()) {
        std::string value = fd04->readValue();
        handleFd04(reinterpret_cast<const uint8_t *>(value.data()), value.size());
      }
    }

    status_ = "HANDSHAKE";
    if (!performHandshake(fd03)) {
      status_ = "HS ERR";
      client_->disconnect();
      scan->clearResults();
      return;
    }

    reassembler_.reset();
    connectedAtMs_ = millis();
    lastPacketMs_ = 0;
    status_ = "CONNECTED";
    Serial.println("Dragy connected; waiting for FD02 NAV-PVT notifications.");
    scan->clearResults();
  }

  bool performHandshake(NimBLERemoteCharacteristic *fd03) {
    if (fd03 == nullptr || !fd03->canRead() || !fd03->canWrite()) {
      Serial.println("FD03 is not readable/writable.");
      return false;
    }

    std::string challenge = fd03->readValue();
    if (challenge.size() < 2) {
      Serial.println("FD03 challenge was shorter than two bytes.");
      return false;
    }

    const uint8_t a = static_cast<uint8_t>(challenge[0]);
    const uint8_t b = static_cast<uint8_t>(challenge[1]);
    uint8_t response[4] = {a, b, static_cast<uint8_t>(a ^ b), static_cast<uint8_t>(a & b)};

    Serial.printf("FD03 challenge %02X %02X; writing handshake response.\n", a, b);
    return fd03->writeValue(response, sizeof(response), true);
  }

  static void onFd02Notify(
    NimBLERemoteCharacteristic *,
    uint8_t *data,
    size_t length,
    bool
  ) {
    if (activeClient_ != nullptr) {
      activeClient_->handleFd02(data, length);
    }
  }

  static void onFd04Notify(
    NimBLERemoteCharacteristic *,
    uint8_t *data,
    size_t length,
    bool
  ) {
    if (activeClient_ != nullptr) {
      activeClient_->handleFd04(data, length);
    }
  }

  void handleFd02(const uint8_t *data, size_t length) {
    std::vector<TelemetrySample> samples = reassembler_.append(data, length);
    for (const TelemetrySample &sample : samples) {
      telemetry_.record(sample);
      lastPacketMs_ = millis();
      status_ = "CONNECTED";
      Serial.printf(
        "NAV-PVT speed=%.2f km/h sats=%u hAcc=%.1f m fix=%s\n",
        sample.speedKph,
        sample.satelliteCount,
        sample.horizontalAccuracyMeters,
        fixLabel(sample.fixQuality)
      );
    }
  }

  void handleFd04(const uint8_t *data, size_t length) {
    if (length >= 2 && data[1] <= 100) {
      batteryPercent_ = data[1];
      Serial.printf("Dragy battery=%d%%\n", batteryPercent_);
    }
  }

  void updateStreamStatus() {
    const uint32_t now = millis();
    if (lastPacketMs_ == 0 && connectedAtMs_ > 0 && now - connectedAtMs_ > kStreamStartTimeoutMs) {
      status_ = "DRAGY BUSY?";
      return;
    }

    if (lastPacketMs_ > 0 && now - lastPacketMs_ > kPacketStallMs) {
      status_ = "STREAM LOST";
    }
  }

  NimBLEClient *client_ = nullptr;
  UbxReassembler reassembler_;
  uint32_t lastScanMs_ = 0;
  uint32_t lastPacketMs_ = 0;
  uint32_t connectedAtMs_ = 0;
  const char *status_ = "IDLE";
  std::string deviceName_;
  int batteryPercent_ = -1;
  bool paused_ = false;
  static DragyBleClient *activeClient_;

 public:
  StatsTracker telemetry_;
};

DragyBleClient *DragyBleClient::activeClient_ = nullptr;

TFT_eSPI tft;
DragyBleClient bleClient;
SpeedDisplayFilter speedFilter;

uint8_t currentPage = 0;
uint32_t lastRenderMs = 0;
uint8_t brightnessMode = 2;
String actionMessage;
uint32_t actionMessageUntilMs = 0;
uint32_t bothHoldStartedMs = 0;
bool bothHoldHandled = false;

struct ButtonState {
  bool wasDown = false;
  uint32_t pressedAtMs = 0;
  bool longHandled = false;
};

ButtonState prevButtonState;
ButtonState nextButtonState;

struct DisplayCache {
  bool initialized = false;
  uint8_t page = 255;
  String speed;
  String labels[3];
  String values[3];
  String status;
  GpsQuality gpsQuality = GpsQuality::None;
};

DisplayCache displayCache;

String formatValue(float value, uint8_t decimals, const char *suffix) {
  if (!std::isfinite(value)) {
    return "--";
  }
  return String(value, static_cast<unsigned int>(decimals)) + suffix;
}

const char *gpsQualityLabel(GpsQuality quality) {
  switch (quality) {
    case GpsQuality::Good:
      return "GOOD";
    case GpsQuality::Fair:
      return "FAIR";
    case GpsQuality::Poor:
      return "POOR";
    case GpsQuality::None:
    default:
      return "NO GPS";
  }
}

uint16_t gpsQualityColor(GpsQuality quality) {
  switch (quality) {
    case GpsQuality::Good:
      return TFT_GREEN;
    case GpsQuality::Fair:
      return TFT_YELLOW;
    case GpsQuality::Poor:
      return TFT_ORANGE;
    case GpsQuality::None:
    default:
      return TFT_RED;
  }
}

String runStateLabel(bool hasCurrent, const TelemetrySample &sample) {
  if (!hasCurrent) {
    return "WAIT";
  }

  if (gpsQualityFor(sample) == GpsQuality::None || gpsQualityFor(sample) == GpsQuality::Poor) {
    return "WAIT GPS";
  }

  return sample.speedKph > 2.0f ? "ROLLING" : "READY";
}

void invalidateDisplayCache(bool full = false) {
  displayCache.page = 255;
  displayCache.speed = "";
  displayCache.status = "";
  displayCache.gpsQuality = GpsQuality::None;
  for (String &label : displayCache.labels) {
    label = "";
  }
  for (String &value : displayCache.values) {
    value = "";
  }
  if (full) {
    displayCache.initialized = false;
  }
}

void showActionMessage(const String &message) {
  actionMessage = message;
  actionMessageUntilMs = millis() + kActionMessageMs;
  displayCache.status = "";
}

void drawRightMetric(int index, int y, const String &label, const String &value, bool force) {
  if (!force && displayCache.labels[index] == label && displayCache.values[index] == value) {
    return;
  }

  tft.fillRect(170, y, 150, 42, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString(label, 315, y, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(value, 315, y + 18, 4);

  displayCache.labels[index] = label;
  displayCache.values[index] = value;
}

String statusLineFor(bool hasCurrent, const TelemetrySample &sample) {
  if (millis() < actionMessageUntilMs && actionMessage.length() > 0) {
    return actionMessage;
  }

  if (bleClient.isPaused()) {
    return "PAUSED";
  }

  String status = bleClient.status();
  if (status == "DRAGY BUSY?" || status == "STREAM LOST") {
    return status;
  }

  if (!bleClient.isConnected()) {
    return status;
  }

  status = runStateLabel(hasCurrent, sample);
  if (bleClient.batteryPercent() >= 0) {
    status += "  ";
    status += bleClient.batteryPercent();
    status += "%";
  }
  if (hasCurrent) {
    status += "  ";
    status += fixLabel(sample.fixQuality);
  }
  return status;
}

void drawStatusLine(bool hasCurrent, const TelemetrySample &sample, bool force) {
  String status = statusLineFor(hasCurrent, sample);
  const GpsQuality quality = hasCurrent ? gpsQualityFor(sample) : GpsQuality::None;
  if (!force && displayCache.status == status && displayCache.gpsQuality == quality) {
    return;
  }

  tft.fillRect(0, 150, 320, 20, TFT_BLACK);
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(millis() < actionMessageUntilMs ? TFT_WHITE : TFT_DARKGREY, TFT_BLACK);
  tft.drawString(status, 4, 168, 2);
  tft.fillCircle(306, 160, 5, gpsQualityColor(quality));
  displayCache.status = status;
  displayCache.gpsQuality = quality;
}

String speedPanelText(bool hasCurrent, const TelemetrySample &sample, float speedKph) {
  String status = bleClient.status();
  if (bleClient.isPaused()) {
    return "PAUSED";
  }
  if (status == "DRAGY BUSY?") {
    return "BUSY?";
  }
  if (status == "STREAM LOST") {
    return "STREAM";
  }
  if (!bleClient.isConnected()) {
    return status;
  }
  if (!hasCurrent || !sample.usable()) {
    return "WAIT GPS";
  }
  return String(speedKph, 0);
}

void drawSpeedPanel(const String &speed, bool isSpeed, bool force) {
  if (!force && displayCache.speed == speed) {
    return;
  }

  tft.fillRect(0, 0, 168, 148, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(speed, 84, isSpeed ? 72 : 78, isSpeed ? 8 : 4);

  if (isSpeed) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("km/h", 84, 132, 4);
  }
  displayCache.speed = speed;
}

void renderDisplay() {
  const bool hasCurrent = bleClient.telemetry_.hasCurrent();
  TelemetrySample sample = hasCurrent ? bleClient.telemetry_.current() : TelemetrySample{};
  const RunStats &stats = bleClient.telemetry_.stats();
  const float displaySpeed = speedFilter.apply(sample);
  const String speedText = speedPanelText(hasCurrent, sample, displaySpeed);
  const bool isSpeedPanel = hasCurrent && sample.usable() && bleClient.isConnected() &&
                            String(bleClient.status()) != "DRAGY BUSY?" &&
                            String(bleClient.status()) != "STREAM LOST";
  const bool firstDraw = !displayCache.initialized;
  const bool pageChanged = displayCache.page != currentPage;
  const bool force = firstDraw || pageChanged;

  if (firstDraw) {
    tft.fillScreen(TFT_BLACK);
  }

  if (pageChanged) {
    tft.fillRect(0, 0, 320, 150, TFT_BLACK);
    displayCache.speed = "";
    for (String &label : displayCache.labels) {
      label = "";
    }
    for (String &value : displayCache.values) {
      value = "";
    }
    displayCache.page = currentPage;
  }

  drawSpeedPanel(speedText, isSpeedPanel, firstDraw);

  if (currentPage == 0) {
    drawRightMetric(0, 4, "AVG 5 KM", formatValue(stats.avg5KmKph, 0, " km/h"), force);
    drawRightMetric(1, 56, "SATS", hasCurrent ? String(sample.satelliteCount) : "--", force);
    drawRightMetric(2, 108, "ACCURACY", hasCurrent ? formatValue(sample.horizontalAccuracyMeters, 1, " m") : "--", force);
  } else if (currentPage == 1) {
    drawRightMetric(0, 4, "AVG 1 KM", formatValue(stats.avg1KmKph, 0, ""), force);
    drawRightMetric(1, 56, "AVG 5 KM", formatValue(stats.avg5KmKph, 0, ""), force);
    drawRightMetric(2, 108, "AVG 10 KM", formatValue(stats.avg10KmKph, 0, ""), force);
  } else if (currentPage == 2) {
    drawRightMetric(0, 4, "PEAK", formatValue(stats.peakSpeedKph, 0, " km/h"), force);
    drawRightMetric(1, 56, "DIST", formatValue(stats.distanceMeters / 1000.0f, 2, " km"), force);
    drawRightMetric(2, 108, "ALT", hasCurrent ? formatValue(sample.altitudeMeters, 0, " m") : "--", force);
  }

  drawStatusLine(hasCurrent, sample, firstDraw);
  displayCache.initialized = true;
}

void applyBrightnessLevel(uint8_t value) {
  static uint8_t level = 0;
  constexpr uint8_t steps = 16;

  value = std::min<uint8_t>(value, steps);
  if (value == 0) {
    digitalWrite(kBacklightPin, LOW);
    delay(3);
    level = 0;
    return;
  }

  if (level == 0) {
    digitalWrite(kBacklightPin, HIGH);
    level = steps;
    delayMicroseconds(30);
  }

  const uint8_t from = steps - level;
  const uint8_t to = steps - value;
  const uint8_t pulses = (steps + to - from) % steps;
  for (uint8_t index = 0; index < pulses; ++index) {
    digitalWrite(kBacklightPin, LOW);
    digitalWrite(kBacklightPin, HIGH);
  }
  level = value;
}

void changePage(int8_t pageDelta) {
  currentPage = static_cast<uint8_t>(
    (static_cast<int>(currentPage) + static_cast<int>(kPageCount) + pageDelta) %
    static_cast<int>(kPageCount)
  );
  invalidateDisplayCache();
  renderDisplay();
  Serial.printf("Page changed to %u.\n", static_cast<unsigned>(currentPage + 1));
}

void resetRunStatsAction() {
  bleClient.resetRunStats();
  speedFilter.reset();
  showActionMessage("STATS RESET");
  invalidateDisplayCache();
  renderDisplay();
  Serial.println("Run stats reset.");
}

void toggleBlePauseAction() {
  if (bleClient.isPaused()) {
    bleClient.resumeScanning();
    showActionMessage("SCAN RESUME");
  } else {
    bleClient.disconnectAndPause();
    showActionMessage("DISCONNECTED");
  }
  invalidateDisplayCache();
  renderDisplay();
}

void cycleBrightnessAction() {
  brightnessMode = static_cast<uint8_t>((brightnessMode + 1) % kBrightnessModeCount);
  applyBrightnessLevel(kBrightnessLevels[brightnessMode]);
  showActionMessage(String("BRIGHT ") + kBrightnessLabels[brightnessMode]);
  invalidateDisplayCache();
  renderDisplay();
  Serial.printf("Brightness set to %s.\n", kBrightnessLabels[brightnessMode]);
}

void updateButton(
  bool isDown,
  ButtonState &state,
  void (*shortAction)(),
  void (*longAction)(),
  bool suppressIndividualLong
) {
  const uint32_t now = millis();
  if (isDown && !state.wasDown) {
    state.wasDown = true;
    state.pressedAtMs = now;
    state.longHandled = false;
  }

  if (isDown && !state.longHandled && !suppressIndividualLong &&
      now - state.pressedAtMs >= kButtonLongPressMs) {
    state.longHandled = true;
    longAction();
  }

  if (!isDown && state.wasDown) {
    const uint32_t heldMs = now - state.pressedAtMs;
    if (!state.longHandled && heldMs >= kButtonDebounceMs) {
      shortAction();
    }
    state.wasDown = false;
    state.longHandled = false;
  }
}

void pagePreviousAction() {
  changePage(-1);
}

void pageNextAction() {
  changePage(1);
}

void handleButtons() {
  const bool prevDown = digitalRead(kButtonPrevPin) == LOW;
  const bool nextDown = digitalRead(kButtonNextPin) == LOW;
  const bool bothDown = prevDown && nextDown;
  const uint32_t now = millis();

  updateButton(prevDown, prevButtonState, pagePreviousAction, resetRunStatsAction, bothDown);
  updateButton(nextDown, nextButtonState, pageNextAction, toggleBlePauseAction, bothDown);

  if (bothDown) {
    prevButtonState.longHandled = true;
    nextButtonState.longHandled = true;

    if (bothHoldStartedMs == 0) {
      bothHoldStartedMs = now;
    }

    if (!bothHoldHandled && now - bothHoldStartedMs >= kBothButtonHoldMs) {
      bothHoldHandled = true;
      cycleBrightnessAction();
    }
  } else {
    bothHoldStartedMs = 0;
    bothHoldHandled = false;
  }
}

void setupDisplay() {
  pinMode(kDisplayPowerPin, OUTPUT);
  digitalWrite(kDisplayPowerPin, HIGH);
  pinMode(kBacklightPin, OUTPUT);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  applyBrightnessLevel(kBrightnessLevels[brightnessMode]);
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);
  delay(250);
  Serial.println("DragyDash ESP32 starting.");

  pinMode(kButtonPrevPin, INPUT_PULLUP);
  pinMode(kButtonNextPin, INPUT_PULLUP);

  setupDisplay();
  renderDisplay();
  bleClient.begin();
}

void loop() {
  bleClient.loop();
  handleButtons();

  const uint32_t now = millis();
  if (now - lastRenderMs >= kRenderIntervalMs) {
    lastRenderMs = now;
    renderDisplay();
  }

  delay(10);
}
