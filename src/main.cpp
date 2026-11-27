#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <TFT_eSPI.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "dragy_core/speed_filter.h"
#include "dragy_core/stats_tracker.h"
#include "dragy_core/types.h"
#include "dragy_core/ubx_parser.h"
#include "dragy_core/ubx_reassembler.h"

namespace {

constexpr uint8_t kButtonPrevPin = 0;
constexpr uint8_t kButtonNextPin = 14;
constexpr uint8_t kDisplayPowerPin = 15;
constexpr uint8_t kBacklightPin = 38;

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kBleScanMs = 3000;
constexpr uint32_t kBleRetryMs = 3000;
constexpr uint32_t kRenderIntervalMs = 100;
constexpr uint32_t kButtonDebounceMs = 35;
constexpr uint32_t kButtonLongPressMs = 900;
constexpr uint32_t kBothButtonHoldMs = 1200;
constexpr uint32_t kActionMessageMs = 1600;
constexpr uint32_t kStreamStartTimeoutMs = 8000;
constexpr uint32_t kPacketStallMs = 4000;
constexpr uint32_t kTelemetryLogIntervalMs = 1000;
constexpr uint8_t kPageCount = 3;
constexpr uint8_t kBrightnessLevels[] = {4, 10, 16};
constexpr const char *kBrightnessLabels[] = {"DIM", "MID", "HIGH"};
constexpr uint8_t kBrightnessModeCount = 3;
constexpr int8_t kBleTxPowerDbm = 6;

constexpr size_t kPacketChunkMaxBytes = 128;
constexpr UBaseType_t kPacketQueueLength = 32;

const NimBLEUUID kDragyServiceUuid("fd00");
const NimBLEUUID kFd02Uuid("fd02");
const NimBLEUUID kFd03Uuid("fd03");
const NimBLEUUID kFd04Uuid("fd04");

#ifdef DRAGY_UNITS_IMPERIAL
constexpr bool kUseImperialUnits = true;
constexpr const char *kUnitsName = "imperial";
constexpr const char *kSpeedUnit = "mph";
constexpr const char *kSpeedUnitSpaced = " mph";
constexpr const char *kDistanceUnitSpaced = " mi";
constexpr const char *kLengthUnitSpaced = " ft";
constexpr const char *kAvgPrimaryLabel = "AVG 3 MI";
constexpr const char *kAvgShortLabel = "AVG 1 MI";
constexpr const char *kAvgMediumLabel = "AVG 3 MI";
constexpr const char *kAvgLongLabel = "AVG 5 MI";
#else
constexpr bool kUseImperialUnits = false;
constexpr const char *kUnitsName = "metric";
constexpr const char *kSpeedUnit = "km/h";
constexpr const char *kSpeedUnitSpaced = " km/h";
constexpr const char *kDistanceUnitSpaced = " km";
constexpr const char *kLengthUnitSpaced = " m";
constexpr const char *kAvgPrimaryLabel = "AVG 5 KM";
constexpr const char *kAvgShortLabel = "AVG 1 KM";
constexpr const char *kAvgMediumLabel = "AVG 5 KM";
constexpr const char *kAvgLongLabel = "AVG 10 KM";
#endif

float toDisplaySpeed(float speedKph) {
  if (!std::isfinite(speedKph)) {
    return NAN;
  }
  return kUseImperialUnits ? speedKph * dragy::kKphToMph : speedKph;
}

float toDisplayDistance(float distanceMeters) {
  if (!std::isfinite(distanceMeters)) {
    return NAN;
  }
  return kUseImperialUnits
           ? distanceMeters / dragy::kMetersPerMile
           : distanceMeters / 1000.0f;
}

float toDisplayLength(float meters) {
  if (!std::isfinite(meters)) {
    return NAN;
  }
  return kUseImperialUnits ? meters * dragy::kMetersToFeet : meters;
}

struct BleChunkEvent {
  uint32_t rxMs = 0;
  uint16_t length = 0;
  uint8_t data[kPacketChunkMaxBytes] = {};
};

struct ScanCandidate {
  bool valid = false;
  NimBLEAddress address;
  int rssi = -127;
  char name[48] = {};
};

class DragyBleClient {
 public:
  void begin() {
    activeClient_ = this;
    packetQueue_ = xQueueCreate(kPacketQueueLength, sizeof(BleChunkEvent));

    NimBLEDevice::init("");
    NimBLEDevice::setPower(kBleTxPowerDbm);

    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&scanCallbacks_, false);
    scan->setActiveScan(true);
    scan->setInterval(45);
    scan->setWindow(30);

    // Start scanning immediately on boot instead of waiting a full retry interval.
    nextScanAllowedMs_ = millis();
  }

  void loop() {
    processPendingDisconnect();
    drainPacketQueue();
    syncBatteryPercent();

    if (paused_) {
      stopScanIfActive();
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

    if (scanFinished_.exchange(false)) {
      scanning_ = false;
      ScanCandidate candidate;
      if (takeBestCandidate(candidate)) {
        connectToCandidate(candidate);
        return;
      }

      status_ = "NO DRAGY";
      Serial.println("No Dragy candidate found.");
      nextScanAllowedMs_ = millis() + kBleRetryMs;
      return;
    }

    if (scanning_) {
      status_ = "SCAN";
      return;
    }

    const uint32_t now = millis();
    if (static_cast<int32_t>(now - nextScanAllowedMs_) >= 0) {
      startAsyncScan();
    }
  }

  const char *status() const { return status_; }
  int batteryPercent() const { return batteryPercent_; }
  bool isConnected() const { return client_ != nullptr && client_->isConnected(); }
  bool isPaused() const { return paused_; }
  float displaySpeedKph() const { return lastDisplaySpeedKph_; }

  void resetRunStats() {
    telemetry_.resetStats();
    speedFilter_.reset();
    lastDisplaySpeedKph_ = 0.0f;
  }

  void disconnectAndPause() {
    paused_ = true;
    status_ = "PAUSED";
    stopScanIfActive();
    clearPendingPackets();
    reassembler_.reset();
    speedFilter_.reset();
    lastDisplaySpeedKph_ = 0.0f;
    lastPacketMs_ = 0;
    connectedAtMs_ = 0;
    batteryPercent_ = -1;
    pendingBatteryPercent_.store(-1);
    if (client_ != nullptr && client_->isConnected()) {
      client_->disconnect();
    }
    Serial.println("Dragy BLE paused.");
  }

  void resumeScanning() {
    paused_ = false;
    status_ = "SCAN";
    nextScanAllowedMs_ = millis();
    Serial.println("Dragy BLE scanning resumed.");
  }

  const dragy::StatsTracker &telemetry() const { return telemetry_; }

 private:
  class ScanCallbacks final : public NimBLEScanCallbacks {
   public:
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override {
      if (advertisedDevice == nullptr || activeClient_ == nullptr) {
        return;
      }

      const bool hasFd00 = advertisedDevice->isAdvertisingService(kDragyServiceUuid);
      const std::string name = advertisedDevice->getName();
      if (!dragy::looksLikeDragyDevice(name, hasFd00)) {
        return;
      }

      activeClient_->considerCandidate(
        advertisedDevice->getAddress(),
        advertisedDevice->getRSSI(),
        name
      );
    }

    void onScanEnd(const NimBLEScanResults &, int) override {
      if (activeClient_ != nullptr) {
        activeClient_->scanFinished_.store(true);
      }
    }
  };

  class ClientCallbacks final : public NimBLEClientCallbacks {
   public:
    void onDisconnect(NimBLEClient *, int reason) override {
      if (activeClient_ != nullptr) {
        activeClient_->lastDisconnectReason_.store(reason);
        activeClient_->disconnectedEvent_.store(true);
      }
    }
  };

  void considerCandidate(const NimBLEAddress &address, int rssi, const std::string &name) {
    portENTER_CRITICAL(&scanMux_);
    if (!bestCandidate_.valid || rssi > bestCandidate_.rssi) {
      bestCandidate_.valid = true;
      bestCandidate_.address = address;
      bestCandidate_.rssi = rssi;
      const size_t copyLen = std::min(name.size(), sizeof(bestCandidate_.name) - 1);
      std::memcpy(bestCandidate_.name, name.data(), copyLen);
      bestCandidate_.name[copyLen] = '\0';
    }
    portEXIT_CRITICAL(&scanMux_);
  }

  bool takeBestCandidate(ScanCandidate &out) {
    portENTER_CRITICAL(&scanMux_);
    const bool hadCandidate = bestCandidate_.valid;
    if (hadCandidate) {
      out = bestCandidate_;
      bestCandidate_.valid = false;
    }
    portEXIT_CRITICAL(&scanMux_);
    return hadCandidate;
  }

  void clearCandidate() {
    portENTER_CRITICAL(&scanMux_);
    bestCandidate_.valid = false;
    portEXIT_CRITICAL(&scanMux_);
  }

  void startAsyncScan() {
    clearCandidate();
    scanFinished_.store(false);
    status_ = "SCAN";
    Serial.println("Scanning for Dragy FD00/DRG advertisements...");

    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->clearResults();
    if (scan->start(kBleScanMs, false, true)) {
      scanning_ = true;
    } else {
      scanning_ = false;
      status_ = "SCAN ERR";
      nextScanAllowedMs_ = millis() + kBleRetryMs;
    }
  }

  void stopScanIfActive() {
    if (!scanning_) {
      return;
    }

    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) {
      scan->stop();
    }
    scan->clearResults();
    scanning_ = false;
    scanFinished_.store(false);
    clearCandidate();
  }

  void processPendingDisconnect() {
    if (!disconnectedEvent_.exchange(false)) {
      return;
    }

    const int reason = lastDisconnectReason_.load();
    clearPendingPackets();
    reassembler_.reset();
    speedFilter_.reset();
    lastDisplaySpeedKph_ = 0.0f;
    lastPacketMs_ = 0;
    connectedAtMs_ = 0;
    batteryPercent_ = -1;
    pendingBatteryPercent_.store(-1);

    if (!paused_) {
      status_ = "SCAN";
      nextScanAllowedMs_ = millis() + kBleRetryMs;
    }
    Serial.printf("Dragy disconnected (reason=%d).\n", reason);
  }

  void connectToCandidate(const ScanCandidate &candidate) {
    const std::string redactedName = dragy::redactDeviceName(candidate.name);
    status_ = "CONNECT";
    Serial.printf("Connecting to %s (rssi=%d)...\n", redactedName.c_str(), candidate.rssi);

    if (client_ == nullptr) {
      client_ = NimBLEDevice::createClient();
      client_->setClientCallbacks(&clientCallbacks_, false);
    }

    if (!client_->connect(candidate.address, true, false, true)) {
      status_ = "CONNECT ERR";
      Serial.println("Dragy connection failed.");
      nextScanAllowedMs_ = millis() + kBleRetryMs;
      return;
    }

    NimBLERemoteService *service = client_->getService(kDragyServiceUuid);
    if (service == nullptr) {
      status_ = "NO FD00";
      Serial.println("FD00 service not found.");
      client_->disconnect();
      nextScanAllowedMs_ = millis() + kBleRetryMs;
      return;
    }

    NimBLERemoteCharacteristic *fd02 = service->getCharacteristic(kFd02Uuid);
    NimBLERemoteCharacteristic *fd03 = service->getCharacteristic(kFd03Uuid);
    NimBLERemoteCharacteristic *fd04 = service->getCharacteristic(kFd04Uuid);

    if (fd02 == nullptr || fd03 == nullptr) {
      status_ = "NO CHAR";
      Serial.println("Required FD02/FD03 characteristics not found.");
      client_->disconnect();
      nextScanAllowedMs_ = millis() + kBleRetryMs;
      return;
    }

    if (!fd02->canNotify() || !fd02->subscribe(true, onFd02Notify)) {
      status_ = "FD02 SUB ERR";
      Serial.println("FD02 notify subscription failed.");
      client_->disconnect();
      nextScanAllowedMs_ = millis() + kBleRetryMs;
      return;
    }

    if (fd04 != nullptr) {
      if (fd04->canNotify()) {
        fd04->subscribe(true, onFd04Notify);
      }
      if (fd04->canRead()) {
        NimBLEAttValue value = fd04->readValue();
        handleFd04Payload(value.data(), value.size());
      }
    }

    status_ = "HANDSHAKE";
    if (!performHandshake(fd03)) {
      status_ = "HS ERR";
      client_->disconnect();
      nextScanAllowedMs_ = millis() + kBleRetryMs;
      return;
    }

    clearPendingPackets();
    reassembler_.reset();
    speedFilter_.reset();
    lastDisplaySpeedKph_ = 0.0f;
    connectedAtMs_ = millis();
    lastPacketMs_ = 0;
    status_ = "CONNECTED";
    Serial.println("Dragy connected; waiting for FD02 NAV-PVT notifications.");
  }

  bool performHandshake(NimBLERemoteCharacteristic *fd03) {
    if (fd03 == nullptr || !fd03->canRead() || !fd03->canWrite()) {
      Serial.println("FD03 is not readable/writable.");
      return false;
    }

    NimBLEAttValue challenge = fd03->readValue();
    if (challenge.size() < 2) {
      Serial.println("FD03 challenge was shorter than two bytes.");
      return false;
    }

    const auto response = dragy::buildFd03HandshakeResponse(challenge[0], challenge[1]);
    Serial.println("FD03 handshake response written.");
    return fd03->writeValue(response.data(), response.size(), true);
  }

  static void onFd02Notify(
    NimBLERemoteCharacteristic *,
    uint8_t *data,
    size_t length,
    bool
  ) {
    if (activeClient_ == nullptr || activeClient_->packetQueue_ == nullptr || data == nullptr || length == 0) {
      return;
    }

    const uint32_t rxMs = millis();
    size_t offset = 0;
    while (offset < length) {
      BleChunkEvent event;
      event.rxMs = rxMs;
      const size_t chunk = std::min(kPacketChunkMaxBytes, length - offset);
      event.length = static_cast<uint16_t>(chunk);
      std::memcpy(event.data, data + offset, chunk);
      // Non-blocking enqueue from NimBLE host task; drop oldest if saturated rather than stalling BLE.
      if (xQueueSend(activeClient_->packetQueue_, &event, 0) != pdTRUE) {
        BleChunkEvent discarded;
        xQueueReceive(activeClient_->packetQueue_, &discarded, 0);
        xQueueSend(activeClient_->packetQueue_, &event, 0);
      }
      offset += chunk;
    }
  }

  static void onFd04Notify(
    NimBLERemoteCharacteristic *,
    uint8_t *data,
    size_t length,
    bool
  ) {
    if (activeClient_ != nullptr) {
      activeClient_->handleFd04Payload(data, length);
    }
  }

  void handleFd04Payload(const uint8_t *data, size_t length) {
    int percent = -1;
    if (dragy::parseFd04BatteryPercent(data, length, percent)) {
      pendingBatteryPercent_.store(percent);
    }
  }

  void syncBatteryPercent() {
    const int latest = pendingBatteryPercent_.load();
    if (latest >= 0 && latest != batteryPercent_) {
      batteryPercent_ = latest;
      Serial.printf("Dragy battery=%d%%\n", batteryPercent_);
    }
  }

  void clearPendingPackets() {
    if (packetQueue_ != nullptr) {
      xQueueReset(packetQueue_);
    }
  }

  void drainPacketQueue() {
    if (packetQueue_ == nullptr) {
      return;
    }

    BleChunkEvent event;
    while (xQueueReceive(packetQueue_, &event, 0) == pdTRUE) {
      auto samples = reassembler_.append(event.data, event.length, event.rxMs);
      for (const dragy::TelemetrySample &sample : samples) {
        const float filteredSpeedKph = speedFilter_.apply(sample);
        lastDisplaySpeedKph_ = filteredSpeedKph;
        telemetry_.record(sample, filteredSpeedKph);
        lastPacketMs_ = millis();
        status_ = "CONNECTED";

        const uint32_t now = millis();
        if (now - lastTelemetryLogMs_ >= kTelemetryLogIntervalMs) {
          lastTelemetryLogMs_ = now;
          Serial.printf(
            "NAV-PVT speed=%.2f km/h sats=%u hAcc=%.1f m fix=%s\n",
            filteredSpeedKph,
            sample.satelliteCount,
            sample.horizontalAccuracyMeters,
            dragy::fixLabel(sample.fixQuality)
          );
        }
      }
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
      speedFilter_.reset();
      lastDisplaySpeedKph_ = 0.0f;
    }
  }

  NimBLEClient *client_ = nullptr;
  QueueHandle_t packetQueue_ = nullptr;
  ScanCallbacks scanCallbacks_;
  ClientCallbacks clientCallbacks_;
  portMUX_TYPE scanMux_ = portMUX_INITIALIZER_UNLOCKED;
  ScanCandidate bestCandidate_{};

  dragy::UbxReassembler reassembler_;
  dragy::SpeedDisplayFilter speedFilter_;
  dragy::StatsTracker telemetry_;

  uint32_t nextScanAllowedMs_ = 0;
  uint32_t lastPacketMs_ = 0;
  uint32_t connectedAtMs_ = 0;
  uint32_t lastTelemetryLogMs_ = 0;
  float lastDisplaySpeedKph_ = 0.0f;
  const char *status_ = "SCAN";
  int batteryPercent_ = -1;
  bool scanning_ = false;
  bool paused_ = false;

  std::atomic<int> pendingBatteryPercent_{-1};
  std::atomic<bool> scanFinished_{false};
  std::atomic<bool> disconnectedEvent_{false};
  std::atomic<int> lastDisconnectReason_{0};

  static DragyBleClient *activeClient_;
};

DragyBleClient *DragyBleClient::activeClient_ = nullptr;

TFT_eSPI tft;
Preferences prefs;
DragyBleClient bleClient;

uint8_t currentPage = 0;
uint32_t lastRenderMs = 0;
uint8_t brightnessMode = 2;
String actionMessage;
uint32_t actionMessageUntilMs = 0;
bool hasActiveActionMessage = false;
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
  bool isSpeed = false;
  String labels[3];
  String values[3];
  String status;
  dragy::GpsQuality gpsQuality = dragy::GpsQuality::None;
};

DisplayCache displayCache;

bool isActionMessageActive(uint32_t now) {
  if (!hasActiveActionMessage || actionMessage.length() == 0) {
    return false;
  }
  if (static_cast<int32_t>(now - actionMessageUntilMs) >= 0) {
    hasActiveActionMessage = false;
    return false;
  }
  return true;
}

String formatValue(float value, uint8_t decimals, const char *suffix) {
  if (!std::isfinite(value)) {
    return "--";
  }
  return String(value, static_cast<unsigned int>(decimals)) + suffix;
}

uint16_t gpsQualityColor(dragy::GpsQuality quality) {
  switch (quality) {
    case dragy::GpsQuality::Good:
      return TFT_GREEN;
    case dragy::GpsQuality::Fair:
      return TFT_YELLOW;
    case dragy::GpsQuality::Poor:
      return TFT_ORANGE;
    case dragy::GpsQuality::None:
    default:
      return TFT_RED;
  }
}

String runStateLabel(bool hasCurrent, const dragy::TelemetrySample &sample, float filteredSpeedKph) {
  if (!hasCurrent) {
    return "WAIT";
  }

  const dragy::GpsQuality quality = dragy::gpsQualityFor(sample);
  if (quality == dragy::GpsQuality::None || quality == dragy::GpsQuality::Poor) {
    return "WAIT GPS";
  }

  return filteredSpeedKph > dragy::SpeedDisplayFilter::kStopMovingKph ? "ROLLING" : "READY";
}

void invalidateDisplayCache(bool full = false) {
  displayCache.page = 255;
  displayCache.speed = "";
  displayCache.isSpeed = false;
  displayCache.status = "";
  displayCache.gpsQuality = dragy::GpsQuality::None;
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
  hasActiveActionMessage = true;
  displayCache.status = "";
}

void drawRightMetric(int index, int y, const String &label, const String &value, bool force) {
  if (!force && displayCache.labels[index] == label && displayCache.values[index] == value) {
    return;
  }

  tft.fillRect(171, y, 149, 44, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString(label, 315, y, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(value, 315, y + 18, 4);

  displayCache.labels[index] = label;
  displayCache.values[index] = value;
}

String statusLineFor(bool hasCurrent, const dragy::TelemetrySample &sample, float filteredSpeedKph) {
  const uint32_t now = millis();
  if (isActionMessageActive(now)) {
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

  status = runStateLabel(hasCurrent, sample, filteredSpeedKph);
  if (bleClient.batteryPercent() >= 0) {
    status += "  ";
    status += bleClient.batteryPercent();
    status += "%";
  }
  if (hasCurrent) {
    status += "  ";
    status += dragy::fixLabel(sample.fixQuality);
  }
  return status;
}

void drawStatusLine(bool hasCurrent, const dragy::TelemetrySample &sample, float filteredSpeedKph, bool force) {
  const uint32_t now = millis();
  const bool showingAction = isActionMessageActive(now);
  String status = statusLineFor(hasCurrent, sample, filteredSpeedKph);
  const dragy::GpsQuality quality =
    (bleClient.isConnected() && hasCurrent) ? dragy::gpsQualityFor(sample) : dragy::GpsQuality::None;
  if (!force && displayCache.status == status && displayCache.gpsQuality == quality) {
    return;
  }

  tft.fillRect(0, 150, 320, 20, TFT_BLACK);
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(showingAction ? TFT_WHITE : TFT_DARKGREY, TFT_BLACK);
  tft.drawString(status, 4, 168, 2);
  tft.fillCircle(306, 160, 5, gpsQualityColor(quality));
  displayCache.status = status;
  displayCache.gpsQuality = quality;
}

String speedPanelText(bool hasCurrent, const dragy::TelemetrySample &sample, float displaySpeed) {
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
  return String(displaySpeed, 0);
}

void drawSpeedPanel(const String &speed, bool isSpeed, bool force) {
  if (!force && displayCache.speed == speed && displayCache.isSpeed == isSpeed) {
    return;
  }

  tft.fillRect(0, 0, 168, 148, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(speed, 84, isSpeed ? 72 : 78, isSpeed ? 8 : 4);

  if (isSpeed) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString(kSpeedUnit, 84, 132, 4);
  }
  displayCache.speed = speed;
  displayCache.isSpeed = isSpeed;
}

void renderDisplay() {
  const auto &tracker = bleClient.telemetry();
  const bool hasCurrent = tracker.hasCurrent();
  const dragy::TelemetrySample sample = hasCurrent ? tracker.current() : dragy::TelemetrySample{};
  const dragy::RunStats &stats = tracker.stats();
  const float filteredSpeedKph = bleClient.displaySpeedKph();
  const float displaySpeed = toDisplaySpeed(filteredSpeedKph);
  const String speedText = speedPanelText(hasCurrent, sample, displaySpeed);
  const String bleStatus = bleClient.status();
  const bool isSpeedPanel = hasCurrent && sample.usable() && bleClient.isConnected() &&
                            bleStatus != "DRAGY BUSY?" && bleStatus != "STREAM LOST";
  const bool firstDraw = !displayCache.initialized;
  const bool pageChanged = displayCache.page != currentPage;
  const bool force = firstDraw || pageChanged;

  if (firstDraw) {
    tft.fillScreen(TFT_BLACK);
  }

  if (pageChanged) {
    tft.fillRect(0, 0, 320, 150, TFT_BLACK);
    tft.drawFastVLine(169, 8, 136, 0x2104);  // Subtle dark-slate divider
    displayCache.speed = "";
    displayCache.isSpeed = false;
    for (String &label : displayCache.labels) {
      label = "";
    }
    for (String &value : displayCache.values) {
      value = "";
    }
    displayCache.page = currentPage;
  }

  drawSpeedPanel(speedText, isSpeedPanel, force);

  const float avgShort = toDisplaySpeed(kUseImperialUnits ? stats.avg1MiKph : stats.avg1KmKph);
  const float avgMedium = toDisplaySpeed(kUseImperialUnits ? stats.avg3MiKph : stats.avg5KmKph);
  const float avgLong = toDisplaySpeed(kUseImperialUnits ? stats.avg5MiKph : stats.avg10KmKph);

  if (currentPage == 0) {
    drawRightMetric(0, 4, kAvgPrimaryLabel, formatValue(avgMedium, 0, kSpeedUnitSpaced), force);
    drawRightMetric(1, 56, "SATS", hasCurrent ? String(sample.satelliteCount) : "--", force);
    drawRightMetric(
      2,
      108,
      "ACCURACY",
      hasCurrent ? formatValue(toDisplayLength(sample.horizontalAccuracyMeters), 1, kLengthUnitSpaced) : "--",
      force
    );
  } else if (currentPage == 1) {
    drawRightMetric(0, 4, kAvgShortLabel, formatValue(avgShort, 0, kSpeedUnitSpaced), force);
    drawRightMetric(1, 56, kAvgMediumLabel, formatValue(avgMedium, 0, kSpeedUnitSpaced), force);
    drawRightMetric(2, 108, kAvgLongLabel, formatValue(avgLong, 0, kSpeedUnitSpaced), force);
  } else if (currentPage == 2) {
    drawRightMetric(0, 4, "PEAK", formatValue(toDisplaySpeed(stats.peakSpeedKph), 0, kSpeedUnitSpaced), force);
    drawRightMetric(1, 56, "DIST", formatValue(toDisplayDistance(stats.distanceMeters), 2, kDistanceUnitSpaced), force);
    drawRightMetric(
      2,
      108,
      "ALT",
      hasCurrent ? formatValue(toDisplayLength(sample.altitudeMeters), 0, kLengthUnitSpaced) : "--",
      force
    );
  }

  drawStatusLine(hasCurrent, sample, filteredSpeedKph, firstDraw);
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
    delayMicroseconds(1);
    digitalWrite(kBacklightPin, HIGH);
    delayMicroseconds(1);
  }
  level = value;
}

void savePreferences() {
  prefs.putUChar("page", currentPage);
  prefs.putUChar("bright", brightnessMode);
}

void changePage(int8_t pageDelta) {
  currentPage = static_cast<uint8_t>(
    (static_cast<int>(currentPage) + static_cast<int>(kPageCount) + pageDelta) %
    static_cast<int>(kPageCount)
  );
  savePreferences();
  invalidateDisplayCache();
  renderDisplay();
  Serial.printf("Page changed to %u.\n", static_cast<unsigned>(currentPage + 1));
}

void resetRunStatsAction() {
  bleClient.resetRunStats();
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
  savePreferences();
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

void pagePreviousAction() { changePage(-1); }
void pageNextAction() { changePage(1); }

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
  Serial.printf("DragyDash ESP32 starting (units=%s).\n", kUnitsName);

  prefs.begin("dragydash", false);
  currentPage = prefs.getUChar("page", 0) % kPageCount;
  brightnessMode = prefs.getUChar("bright", 2) % kBrightnessModeCount;

  pinMode(kButtonPrevPin, INPUT_PULLUP);
  pinMode(kButtonNextPin, INPUT_PULLUP);

  setupDisplay();
  bleClient.begin();
  renderDisplay();
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
