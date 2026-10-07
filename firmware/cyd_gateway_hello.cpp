#include <Arduino.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <Preferences.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <stdarg.h>
#include <esp_timer.h>

#include "espnow_smoke_config.h"
#include "serial_json_protocol.h"
#include "calibration_protocol.h"
#include "receive_freshness.h"
#include "runtime_protocol.h"

namespace {
using namespace antenna_controller;

constexpr uint8_t DisplayBacklightPin = 21;
constexpr uint32_t SerialBaudRate = 115200;
constexpr uint8_t ReceiveQueueSize = 8;
constexpr uint32_t RssiFreshnessMs = 10000;
constexpr uint32_t SensorOfflineMs = 3000;
constexpr uint8_t EventLogLineCount = 4;
constexpr uint8_t EventLogLineLength = 42;
constexpr size_t SerialLineLength = 768;
constexpr int HeaderDebugButtonX = 88;
constexpr int HeaderDebugButtonWidth = 42;
constexpr uint8_t TouchIrqPin = 36;
constexpr uint8_t TouchMosiPin = 32;
constexpr uint8_t TouchMisoPin = 39;
constexpr uint8_t TouchClockPin = 25;
constexpr uint8_t TouchCsPin = 33;
constexpr uint32_t TouchCalibrationMagic = 0x5443414CUL;  // "TCAL"

enum class Panel : uint8_t { Home, Keypad, Debug };

TFT_eSPI display;
SPIClass touchscreenSpi(VSPI);
XPT2046_Touchscreen touchscreen(TouchCsPin, TouchIrqPin);

struct ReceivedPacket {
    uint8_t payload[64];
    uint64_t receivedMs;
    uint8_t payloadLength;
    uint8_t sourceMac[6];
};

struct NodeStatus {
    int16_t azimuthDeciDegrees = NoAzimuthDeciDegrees;
    int16_t targetDeciDegrees = NoAzimuthDeciDegrees;
    uint8_t flags = 0;
    int8_t rssiAtGateway = RssiUnavailable;
    int8_t rssiAtNode = RssiUnavailable;
    ReceiveFreshness link;
    uint32_t nodeEpochSeconds = 0;
    uint64_t nodeUtcMs = 0;
    ReportingMode mode = ReportingMode::Normal;
    uint32_t quietRemainingMs = 0;
    uint32_t lastModeMs = 0;
    uint32_t lastTimeSyncMs = 0;
    uint8_t sensorFlags = 0;
    int16_t magneticHeadingDeciDegrees = NoAzimuthDeciDegrees;
    int16_t magneticXDeciMicrotesla = 0;
    int16_t magneticYDeciMicrotesla = 0;
    int16_t magneticZDeciMicrotesla = 0;
    int16_t accelerationXMilliG = 0;
    int16_t accelerationYMilliG = 0;
    int16_t accelerationZMilliG = 0;
    int16_t rollDeciDegrees = 0;
    int16_t pitchDeciDegrees = 0;
    ReceiveFreshness sensor;
    bool hasSensorSample = false;
};

ReceivedPacket receiveQueue[ReceiveQueueSize]{};
volatile uint8_t receiveQueueWriteIndex = 0;
volatile uint8_t receiveQueueReadIndex = 0;
volatile int8_t nodeRssiDbm[AntennaNodeCount] = {RssiUnavailable,
                                                  RssiUnavailable};
volatile uint32_t nodeRssiUpdatedMs[AntennaNodeCount]{};
NodeStatus nodeStatus[AntennaNodeCount]{};
char eventLog[EventLogLineCount][EventLogLineLength]{};
char serialLine[SerialLineLength]{};
size_t serialLineLength = 0;
bool serialLineOverflow = false;
uint32_t sequenceNumber = 0;
uint32_t lastGatewayReportMs = 0;
UtcClock utcClock;
uint32_t lastUiClockSecond = UINT32_MAX;
bool timeValid = false;
Panel activePanel = Panel::Home;
uint8_t keypadNodeId = 1;
char keypadEntry[6]{};
uint8_t keypadEntryLength = 0;
char keypadFeedback[32]{};
bool uiDirty = true;
bool headerDirty = true;
bool nodeCardDirty[AntennaNodeCount] = {true, true};
bool debugDirty = true;
uint32_t lastMotionBlinkPhase = UINT32_MAX;
bool touchWasPressed = false;
bool touchCalibrating = false;
uint8_t touchCalibrationPoint = 0;
int16_t touchRawX[4]{};
int16_t touchRawY[4]{};

struct TouchCalibration {
    uint32_t magic = TouchCalibrationMagic;
    float screenXFromRawX = 0.0F;
    float screenXFromRawY = 0.0F;
    float screenXOffset = 0.0F;
    float screenYFromRawX = 0.0F;
    float screenYFromRawY = 0.0F;
    float screenYOffset = 0.0F;
};

TouchCalibration touchCalibration{};

int screenWidth() {
    return display.width();
}

int screenHeight() {
    return display.height();
}

int nodeIndex(uint8_t nodeId) {
    return nodeId >= 1 && nodeId <= AntennaNodeCount ? nodeId - 1 : -1;
}

bool macMatches(const uint8_t* first, const uint8_t* second) {
    return memcmp(first, second, 6) == 0;
}

bool nodeOnline(uint8_t nodeId) {
    const int index = nodeIndex(nodeId);
    return index >= 0 && nodeStatus[index].link.fresh(millis(), linkTimeout(nodeStatus[index].mode));
}

bool sensorTelemetryFresh(uint8_t nodeId) {
    const int index = nodeIndex(nodeId);
    return index >= 0 && nodeStatus[index].sensor.fresh(millis(), nodeStatus[index].mode == ReportingMode::Quiet ? 90000 : nodeStatus[index].mode == ReportingMode::Normal ? 25000 : SensorOfflineMs);
}

uint64_t utcMilliseconds() { return utcClock.at(uint64_t(esp_timer_get_time())/1000); }
uint32_t epochNow() { return uint32_t(utcMilliseconds()/1000); }

void setEpochMs(uint64_t epochMs) {
    utcClock.synchronize(epochMs, uint64_t(esp_timer_get_time())/1000);
    timeValid = utcClock.valid;
    headerDirty = true;
}
void setEpoch(uint32_t epochSeconds) { setEpochMs(uint64_t(epochSeconds)*1000); }

void formatTime(uint32_t epochSeconds, char* buffer, size_t bufferLength) {
    if (!timeValid || epochSeconds == 0) {
        snprintf(buffer, bufferLength, "UTC --:--:--");
        return;
    }
    const uint32_t secondsOfDay = epochSeconds % 86400UL;
    snprintf(buffer, bufferLength, "UTC %02lu:%02lu:%02lu",
             static_cast<unsigned long>(secondsOfDay / 3600UL),
             static_cast<unsigned long>((secondsOfDay % 3600UL) / 60UL),
             static_cast<unsigned long>(secondsOfDay % 60UL));
}

const char* rssiText(int8_t rssiDbm, char* buffer, size_t bufferLength) {
    if (rssiDbm == RssiUnavailable) {
        snprintf(buffer, bufferLength, "--");
    } else {
        snprintf(buffer, bufferLength, "%d", rssiDbm);
    }
    return buffer;
}

const char* azimuthText(int16_t azimuthDeciDegrees,
                        char* buffer,
                        size_t bufferLength) {
    if (azimuthDeciDegrees == NoAzimuthDeciDegrees) {
        snprintf(buffer, bufferLength, "--.-T");
    } else {
        snprintf(buffer, bufferLength, "%.1fT", azimuthDeciDegrees / 10.0F);
    }
    return buffer;
}

int8_t latestRssiForNode(uint8_t nodeId) {
    const int index = nodeIndex(nodeId);
    if (index < 0 || millis() - nodeRssiUpdatedMs[index] > RssiFreshnessMs) {
        return RssiUnavailable;
    }
    return nodeRssiDbm[index];
}

void onPromiscuousPacket(void* rawPacket, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) {
        return;
    }
    const auto* packet = static_cast<const wifi_promiscuous_pkt_t*>(rawPacket);
    constexpr size_t SourceMacOffset = 10;
    if (packet->rx_ctrl.sig_len < SourceMacOffset + 6) {
        return;
    }
    const uint8_t* sourceMac = packet->payload + SourceMacOffset;
    for (uint8_t nodeId = 1; nodeId <= AntennaNodeCount; ++nodeId) {
        if (macMatches(sourceMac, antennaNodeMac(nodeId))) {
            const int index = nodeIndex(nodeId);
            nodeRssiDbm[index] = packet->rx_ctrl.rssi;
            nodeRssiUpdatedMs[index] = millis();
            return;
        }
    }
}

void drawButton(int x,
                int y,
                int width,
                int height,
                const char* label,
                uint16_t fillColor) {
    display.fillRoundRect(x, y, width, height, 4, fillColor);
    display.drawRoundRect(x, y, width, height, 4, TFT_WHITE);
    display.setTextColor(TFT_WHITE, fillColor);
    const int labelWidth = display.textWidth(label, 2);
    display.drawString(label, x + (width - labelWidth) / 2, y + 4, 2);
}

void drawHeader(const char* title) {
    char timeBuffer[16];
    display.fillRect(0, 0, screenWidth(), 24, TFT_NAVY);
    display.setTextColor(TFT_YELLOW, TFT_NAVY);
    display.drawString(title, 4, 4, 2);
    formatTime(epochNow(), timeBuffer, sizeof(timeBuffer));
    display.setTextColor(TFT_WHITE, TFT_NAVY);
    display.drawRightString(timeBuffer, screenWidth() - 4, 4, 2);
    if (activePanel == Panel::Home) {
        display.fillRoundRect(HeaderDebugButtonX, 3, HeaderDebugButtonWidth,
                              18, 3, TFT_DARKGREY);
        display.drawRoundRect(HeaderDebugButtonX, 3, HeaderDebugButtonWidth,
                              18, 3, TFT_WHITE);
        display.setTextColor(TFT_WHITE, TFT_DARKGREY);
        display.drawCentreString("DBG",
                                 HeaderDebugButtonX + HeaderDebugButtonWidth / 2,
                                 5, 2);
    } else {
        display.fillRoundRect(HeaderDebugButtonX, 3, 48, 18, 3, TFT_DARKGREY);
        display.drawRoundRect(HeaderDebugButtonX, 3, 48, 18, 3, TFT_WHITE);
        display.setTextColor(TFT_WHITE, TFT_DARKGREY);
        display.drawCentreString("BACK", HeaderDebugButtonX + 24, 5, 2);
    }
    display.drawFastHLine(0, 24, screenWidth(), TFT_DARKGREY);
}

void drawNodeCard(uint8_t nodeId, int y) {
    const NodeStatus& status = nodeStatus[nodeIndex(nodeId)];
    const bool online = nodeOnline(nodeId);
    const bool moving = online && hasFlag(status.flags, StatusFlag::Moving);
    const int cardHeight = 98;
    const int buttonY = y + 72;
    char azimuth[12];
    char target[12];
    char magneticHeading[12];
    char line[34];

    display.fillRoundRect(4, y, screenWidth() - 8, cardHeight, 6,
                          online ? TFT_DARKGREY : TFT_MAROON);
    display.drawRoundRect(4, y, screenWidth() - 8, cardHeight, 6, TFT_WHITE);
    display.setTextColor(TFT_WHITE, online ? TFT_DARKGREY : TFT_MAROON);
    snprintf(line, sizeof(line), "NODE %u  %s", nodeId,
             online ? "ONLINE" : "OFFLINE");
    display.drawString(line, 10, y + 4, 2);

    if (moving && ((millis() / 500UL) % 2UL == 0UL)) {
        display.setTextColor(TFT_YELLOW, online ? TFT_DARKGREY : TFT_MAROON);
        display.drawRightString("* SLEWING", screenWidth() - 10, y + 4, 2);
    }

    display.setTextColor(TFT_CYAN, online ? TFT_DARKGREY : TFT_MAROON);
    snprintf(line, sizeof(line), "Az %s",
             azimuthText(status.azimuthDeciDegrees, azimuth, sizeof(azimuth)));
    display.drawString(line, 10, y + 23, 4);
    display.setTextColor(TFT_WHITE, online ? TFT_DARKGREY : TFT_MAROON);
    if (sensorTelemetryFresh(nodeId) &&
        (status.sensorFlags & SensorFlag::MagnetometerValid) != 0) {
        snprintf(line, sizeof(line), "Target %s  Mag %s",
                 azimuthText(status.targetDeciDegrees, target, sizeof(target)),
                 azimuthText(status.magneticHeadingDeciDegrees,
                             magneticHeading, sizeof(magneticHeading)));
        const size_t length = strlen(line);
        if (length > 0 && line[length - 1] == 'T') {
            line[length - 1] = 'M';
        }
    } else {
        snprintf(line, sizeof(line), "Target %s",
                 azimuthText(status.targetDeciDegrees, target, sizeof(target)));
    }
    display.drawString(line, 10, y + 52, 2);

    drawButton(10, buttonY, 88, 22, "-10", TFT_BLUE);
    drawButton(116, buttonY, 88, 22, "Az", TFT_DARKGREEN);
    drawButton(222, buttonY, 88, 22, "+10", TFT_BLUE);
}

void drawHomePanel() {
    display.fillScreen(TFT_BLACK);
    drawHeader("ANT CTRL");
    drawNodeCard(1, 28);
    drawNodeCard(2, 134);
}

void drawKeypadPanel() {
    display.fillScreen(TFT_BLACK);
    drawHeader("NEW AZIMUTH");
    char title[30];
    snprintf(title, sizeof(title), "NODE %u  enter true degrees", keypadNodeId);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.drawString(title, 8, 31, 2);
    display.fillRoundRect(8, 52, screenWidth() - 16, 36, 4, TFT_DARKGREY);
    display.setTextColor(TFT_CYAN, TFT_DARKGREY);
    display.drawRightString(keypadEntryLength == 0 ? "---.-" : keypadEntry,
                            screenWidth() - 16, 58, 4);

    const char* labels[] = {"1", "2", "3", "4", "5", "6",
                            "7", "8", "9", "CLR", "0", "OK"};
    for (int index = 0; index < 12; ++index) {
        const int column = index % 3;
        const int row = index / 3;
        drawButton(12 + column * 103, 95 + row * 31, 92, 26, labels[index],
                   index == 11 ? TFT_DARKGREEN : TFT_BLUE);
    }
    display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    display.drawString("Enter 0..359, then press OK", 8, 220, 2);
    if (keypadFeedback[0] != '\0') {
        display.setTextColor(TFT_YELLOW, TFT_BLACK);
        display.drawRightString(keypadFeedback, screenWidth() - 8, 31, 2);
    }
}

// Draw the degree glyph explicitly; the built-in small font is ASCII-only.
void drawDegreeText(const char* text, int x, int y) {
    const char* part = text;
    while (*part) {
        const char* degree = strstr(part, "\xC2\xB0");
        if (!degree) { display.drawString(part, x, y, 1); break; }
        char segment[96];
        const size_t count = size_t(degree-part);
        if (count >= sizeof(segment)) return;
        memcpy(segment, part, count); segment[count] = '\0';
        display.drawString(segment, x, y, 1);
        x += display.textWidth(segment, 1);
        display.drawCircle(x+2, y+1, 1, TFT_CYAN);
        x += 5;
        part = degree+2;
    }
}

void drawVector(uint8_t node, const char* label, float x, float y, float z,
                const char* units, int precision, int top, bool valid) {
    char row[96], azimuth[16], elevation[16];
    display.setTextColor(TFT_CYAN, TFT_BLACK);
    if (!valid) {
        snprintf(row, sizeof(row), "N%u %s XYZ:(---,---,---) / MAE:(---,---,---) %s", node, label, units);
        display.drawString(row, 4, top, 1);
        return;
    }
    snprintf(row, sizeof(row), "N%u %s XYZ:(%.*f,%.*f,%.*f) %s", node, label, precision, x, precision, y, precision, z, units);
    display.drawString(row, 4, top, 1);
    const VectorMae mae = vectorMae(x, y, z);
    if (isfinite(mae.azimuth)) snprintf(azimuth, sizeof(azimuth), "%.1f\xC2\xB0", mae.azimuth);
    else snprintf(azimuth, sizeof(azimuth), "---");
    if (isfinite(mae.elevation)) snprintf(elevation, sizeof(elevation), "%.1f\xC2\xB0", mae.elevation);
    else snprintf(elevation, sizeof(elevation), "---");
    snprintf(row, sizeof(row), "     MAE:(%.*f,%s,%s) %s", precision, mae.magnitude, azimuth, elevation, units);
    drawDegreeText(row, 4, top+12);
}

void drawDebugPanel() {
    display.fillScreen(TFT_BLACK);
    drawHeader("DEBUG");
    for (uint8_t node = 1; node <= AntennaNodeCount; ++node) {
        const NodeStatus& sensor = nodeStatus[nodeIndex(node)];
        const int top = node == 1 ? 28 : 101;
        char row[64];
        const uint32_t elapsed = millis()-sensor.lastModeMs;
        const uint32_t remaining = elapsed < sensor.quietRemainingMs ? (sensor.quietRemainingMs-elapsed+999)/1000 : 0;
        if (sensor.mode == ReportingMode::Quiet)
            snprintf(row, sizeof(row), "N%u %s quiet %lus | N>G %d dBm", node, nodeOnline(node) ? "ONLINE" : "OFFLINE", (unsigned long)remaining, sensor.rssiAtGateway);
        else snprintf(row, sizeof(row), "N%u %s %s | N>G %d dBm", node, nodeOnline(node) ? "ONLINE" : "OFFLINE", reportingName(sensor.mode), sensor.rssiAtGateway);
        display.setTextColor(TFT_YELLOW, TFT_BLACK);
        display.drawString(row, 4, top, 1);
        const bool fresh = sensor.hasSensorSample;
        drawVector(node, "B", sensor.magneticXDeciMicrotesla/10.0F, sensor.magneticYDeciMicrotesla/10.0F,
                   sensor.magneticZDeciMicrotesla/10.0F, "uT", 1, top+13, fresh && (sensor.sensorFlags & MagnetometerValid));
        drawVector(node, "A", sensor.accelerationXMilliG/1000.0F, sensor.accelerationYMilliG/1000.0F,
                   sensor.accelerationZMilliG/1000.0F, "g", 3, top+39, fresh && (sensor.sensorFlags & AccelerometerValid));
        display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        if (sensor.hasSensorSample)
            snprintf(row, sizeof(row), "Last raw sample: %lu s ago", (unsigned long)((millis()-sensor.sensor.lastReceivedMs)/1000));
        else snprintf(row, sizeof(row), "Awaiting raw sample");
        display.drawString(row, 4, top+65, 1);
    }
    display.drawFastHLine(0, 176, screenWidth(), TFT_DARKGREY);
    display.setTextColor(TFT_YELLOW, TFT_BLACK);
    display.drawString("Events / sensor-frame XYZ and MAE", 4, 180, 1);
    display.setTextColor(TFT_CYAN, TFT_BLACK);
    for (int line = 0; line < EventLogLineCount; ++line)
        display.drawString(eventLog[line], 4, 192+line*11, 1);
}

void redrawDisplay() {
    switch (activePanel) {
        case Panel::Home:
            drawHomePanel();
            break;
        case Panel::Keypad:
            drawKeypadPanel();
            break;
        case Panel::Debug:
            drawDebugPanel();
            break;
    }
    uiDirty = false;
    headerDirty = false;
    debugDirty = false;
    for (uint8_t index = 0; index < AntennaNodeCount; ++index) {
        nodeCardDirty[index] = false;
    }
}

void refreshHomePanel() {
    if (headerDirty) {
        drawHeader("ANT CTRL");
        headerDirty = false;
    }
    for (uint8_t nodeId = 1; nodeId <= AntennaNodeCount; ++nodeId) {
        const int index = nodeIndex(nodeId);
        if (nodeCardDirty[index]) {
            drawNodeCard(nodeId, nodeId == 1 ? 28 : 134);
            nodeCardDirty[index] = false;
        }
    }
}

void appendEvent(const char* format, ...) {
    for (int line = 0; line < EventLogLineCount - 1; ++line) {
        strncpy(eventLog[line], eventLog[line + 1], EventLogLineLength);
    }
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(eventLog[EventLogLineCount - 1], EventLogLineLength, format,
              arguments);
    va_end(arguments);
    if (activePanel == Panel::Debug) {
        debugDirty = true;
    }
}

void onDataSent(const uint8_t*, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("ESP-NOW delivery: failed");
    }
}

void refreshNodeFreshness(uint32_t nowMs) {
    for (uint8_t nodeId = 1; nodeId <= AntennaNodeCount; ++nodeId) {
        const int index = nodeIndex(nodeId);
        NodeStatus& status = nodeStatus[index];
        const bool linkChanged = status.link.update(nowMs, linkTimeout(status.mode));
        const bool sensorChanged = status.sensor.update(nowMs, status.mode == ReportingMode::Quiet ? 90000 : status.mode == ReportingMode::Normal ? 25000 : SensorOfflineMs);
        if (linkChanged || sensorChanged) {
            nodeCardDirty[index] = true;
            debugDirty = true;
        }
        if (linkChanged) {
            const char* state = status.link.wasFresh ? "ONLINE" : "OFFLINE";
            appendEvent("N%u %s", nodeId, state);
            Serial.printf("LINK N%u %s\n", nodeId, state);
        }
    }
}

void onDataReceived(const uint8_t* sourceMac,
                    const uint8_t* data,
                    int dataLength) {
    if (dataLength != sizeof(Packet) && dataLength != sizeof(CalibrationReport) &&
        dataLength != sizeof(SensorTelemetry)) {
        return;
    }
    const uint8_t nextWriteIndex =
        (receiveQueueWriteIndex + 1) % ReceiveQueueSize;
    if (nextWriteIndex == receiveQueueReadIndex) {
        return;
    }
    ReceivedPacket& received = receiveQueue[receiveQueueWriteIndex];
    received.receivedMs = uint64_t(esp_timer_get_time())/1000;
    memcpy(received.payload, data, dataLength);
    received.payloadLength = static_cast<uint8_t>(dataLength);
    memcpy(received.sourceMac, sourceMac, sizeof(received.sourceMac));
    receiveQueueWriteIndex = nextWriteIndex;
}

bool initializeEspNow() {
    WiFi.mode(WIFI_STA);
    delay(100);
    esp_wifi_set_channel(Channel, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) {
        return false;
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);
    wifi_promiscuous_filter_t filter{};
    filter.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(onPromiscuousPacket);
    esp_wifi_set_promiscuous(true);
    for (uint8_t nodeId = 1; nodeId <= AntennaNodeCount; ++nodeId) {
        esp_now_peer_info_t peerInfo{};
        memcpy(peerInfo.peer_addr, antennaNodeMac(nodeId),
               sizeof(peerInfo.peer_addr));
        peerInfo.channel = Channel;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            return false;
        }
    }
    return true;
}

void sendPacket(uint8_t nodeId,
                PacketType packetType,
                CommandType command = CommandType::None,
                int16_t commandValue = NoAzimuthDeciDegrees,
                uint32_t commandEpochSeconds = 0) {
    Packet packet{
        static_cast<uint8_t>(packetType),
        static_cast<uint8_t>(DeviceId::CydGateway),
        ProtocolVersion,
        timeValid ? static_cast<uint8_t>(StatusFlag::TimeValid)
                  : static_cast<uint8_t>(0),
        ++sequenceNumber,
        commandEpochSeconds == 0 ? epochNow() : commandEpochSeconds,
        NoAzimuthDeciDegrees,
        NoAzimuthDeciDegrees,
        commandValue,
        static_cast<uint8_t>(command),
        latestRssiForNode(nodeId)};
    packet.utcMilliseconds = utcMilliseconds();
    const esp_err_t result =
        esp_now_send(antennaNodeMac(nodeId),
                     reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    Serial.printf("TX node %u %s %s\n", nodeId,
                  packetType == PacketType::TimeSync
                      ? "time-sync"
                      : packetType == PacketType::Command ? "command" : "packet",
                  result == ESP_OK ? "queued" : "failed");
}

void sendTimeSync(bool force = false) {
    if (!timeValid) return;
    for (uint8_t nodeId = 1; nodeId <= AntennaNodeCount; ++nodeId) {
        auto& status = nodeStatus[nodeIndex(nodeId)];
        const uint32_t period = status.mode == ReportingMode::Quiet ? 60000 :
                                status.mode == ReportingMode::Continuous ? 3000 : 10000;
        if (force || millis()-status.lastTimeSyncMs >= period) {
            sendPacket(nodeId, PacketType::TimeSync);
            status.lastTimeSyncMs = millis();
        }
    }
}

void emitGatewayStatus() {
    JsonDocument doc;
    doc["t"] = "gs"; doc["ts_ms"] = utcMilliseconds(); doc["tv"] = timeValid;
    doc["up_ms"] = millis(); doc["sync_age_ms"] = utcClock.syncAge(uint64_t(esp_timer_get_time())/1000);
    doc["sync_step_ms"] = utcClock.correctionMs; doc["pv"] = ProtocolVersion;
    const uint64_t now = uint64_t(esp_timer_get_time())/1000;
    doc["sync_error_ms"] = int32_t(lround(utcClock.filteredErrorMs));
    doc["sync_rate_ppm"] = int16_t(lround(utcClock.ratePpm(now)));
    doc["sync_state"] = clockStateName(utcClock.state(now));
    emitJsonFrame(doc);
}

void sendCommand(uint8_t nodeId,
                 CommandType command,
                 int16_t value = NoAzimuthDeciDegrees,
                 uint32_t executeAtEpochSeconds = 0) {
    if (nodeIndex(nodeId) < 0) {
        Serial.println("ERR node must be 1 or 2");
        return;
    }
    sendPacket(nodeId, PacketType::Command, command, value,
               executeAtEpochSeconds);
    appendEvent("TX N%u cmd %u", nodeId, static_cast<uint8_t>(command));
}

void emitStatusJson(const Packet& packet, uint64_t receivedMs) {
    emitRuntimeStatus(packet, "espnow", utcClock.at(receivedMs), latestRssiForNode(packet.senderId));
}
void emitSensorJson(const SensorTelemetry& telemetry) { emitRawSensor(telemetry, "espnow"); }

void processReceivedPackets() {
    while (receiveQueueReadIndex != receiveQueueWriteIndex) {
        const ReceivedPacket received = receiveQueue[receiveQueueReadIndex];
        receiveQueueReadIndex =
            (receiveQueueReadIndex + 1) % ReceiveQueueSize;

        if (received.payloadLength == sizeof(CalibrationReport)) {
            CalibrationReport report{};
            memcpy(&report, received.payload, sizeof(report));
            if (report.type == 7 && report.version == ProtocolVersion &&
                nodeIndex(report.node) >= 0 &&
                macMatches(received.sourceMac, antennaNodeMac(report.node))) {
                nodeStatus[nodeIndex(report.node)].link.receive(millis());
                emitCalibrationJson(report, "espnow");
            }
            continue;
        }
        if (received.payloadLength == sizeof(SensorTelemetry)) {
            SensorTelemetry telemetry{};
            memcpy(&telemetry, received.payload, sizeof(telemetry));
            const int sensorIndex = nodeIndex(telemetry.senderId);
            if (telemetry.packetType !=
                    static_cast<uint8_t>(PacketType::SensorTelemetry) ||
                telemetry.protocolVersion != ProtocolVersion ||
                sensorIndex < 0 ||
                !macMatches(received.sourceMac,
                            antennaNodeMac(telemetry.senderId))) {
                continue;
            }

            NodeStatus& sensorStatus = nodeStatus[sensorIndex];
            sensorStatus.sensorFlags = telemetry.flags;
            sensorStatus.magneticHeadingDeciDegrees =
                telemetry.magneticHeadingDeciDegrees;
            sensorStatus.magneticXDeciMicrotesla =
                telemetry.magneticXDeciMicrotesla;
            sensorStatus.magneticYDeciMicrotesla =
                telemetry.magneticYDeciMicrotesla;
            sensorStatus.magneticZDeciMicrotesla =
                telemetry.magneticZDeciMicrotesla;
            sensorStatus.accelerationXMilliG = telemetry.accelerationXMilliG;
            sensorStatus.accelerationYMilliG = telemetry.accelerationYMilliG;
            sensorStatus.accelerationZMilliG = telemetry.accelerationZMilliG;
            sensorStatus.rollDeciDegrees = telemetry.rollDeciDegrees;
            sensorStatus.pitchDeciDegrees = telemetry.pitchDeciDegrees;
            sensorStatus.sensor.receive(millis());
            sensorStatus.hasSensorSample = true;
            sensorStatus.link.receive(millis());

            const float x = telemetry.magneticXDeciMicrotesla / 10.0F;
            const float y = telemetry.magneticYDeciMicrotesla / 10.0F;
            const float z = telemetry.magneticZDeciMicrotesla / 10.0F;
            Serial.printf(
                "SENSOR N%u flags=0x%02X heading=%.1fM "
                "mag=(%.1f,%.1f,%.1f)uT accel=(%.3f,%.3f,%.3f)g "
                "roll=%.1f pitch=%.1f\n",
                telemetry.senderId, telemetry.flags,
                telemetry.magneticHeadingDeciDegrees / 10.0F, x, y, z,
                telemetry.accelerationXMilliG / 1000.0F,
                telemetry.accelerationYMilliG / 1000.0F,
                telemetry.accelerationZMilliG / 1000.0F,
                telemetry.rollDeciDegrees / 10.0F,
                telemetry.pitchDeciDegrees / 10.0F);
            emitSensorJson(telemetry);

            if (activePanel == Panel::Home) {
                nodeCardDirty[sensorIndex] = true;
            } else if (activePanel == Panel::Debug) {
                debugDirty = true;
            }
            continue;
        }

        Packet packet{};
        memcpy(&packet, received.payload, sizeof(packet));
        const int index = nodeIndex(packet.senderId);
        if (packet.protocolVersion != ProtocolVersion || index < 0 ||
            !macMatches(received.sourceMac, antennaNodeMac(packet.senderId))) {
            continue;
        }
        if (packet.packetType != static_cast<uint8_t>(PacketType::Status) &&
            packet.packetType !=
                static_cast<uint8_t>(PacketType::CommandAcknowledgment) &&
            packet.packetType != static_cast<uint8_t>(PacketType::ReportingAcknowledgment)) {
            continue;
        }

        NodeStatus& status = nodeStatus[index];
        status.azimuthDeciDegrees = packet.azimuthDeciDegrees;
        status.targetDeciDegrees = packet.targetDeciDegrees;
        status.flags = packet.flags;
        status.nodeEpochSeconds = packet.epochSeconds;
        status.nodeUtcMs = packet.utcMilliseconds;
        if (packet.reportingMode <= 2) status.mode = ReportingMode(packet.reportingMode);
        status.quietRemainingMs = packet.quietRemainingMs;
        status.lastModeMs = millis();
        status.link.receive(millis());
        const int8_t rssiAtGateway = latestRssiForNode(packet.senderId);
        if (rssiAtGateway != RssiUnavailable) {
            status.rssiAtGateway = rssiAtGateway;
        }
        if (packet.receiverRssiDbm != RssiUnavailable) {
            status.rssiAtNode = packet.receiverRssiDbm;
        }
        if (packet.packetType ==
            static_cast<uint8_t>(PacketType::CommandAcknowledgment)) {
            appendEvent("ACK N%u cmd %u", packet.senderId, packet.commandType);
        }
        Serial.printf("RX N%u az=%.1fT target=%.1fT flags=0x%02X\n",
                      packet.senderId, packet.azimuthDeciDegrees / 10.0F,
                      packet.targetDeciDegrees == NoAzimuthDeciDegrees
                          ? -1.0F
                          : packet.targetDeciDegrees / 10.0F,
                      packet.flags);
        emitStatusJson(packet, received.receivedMs);
        if (activePanel == Panel::Home) {
            nodeCardDirty[index] = true;
        } else if (activePanel == Panel::Debug) {
            debugDirty = true;
        }
    }
}

int64_t daysFromCivil(int year, unsigned month, unsigned day) {
    year -= month <= 2;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yearOfEra = static_cast<unsigned>(year - era * 400);
    const unsigned dayOfYear =
        (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const unsigned dayOfEra =
        yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
    return era * 146097 + static_cast<int>(dayOfEra) - 719468;
}

bool parseTimestamp(const char* text, uint32_t* epochSeconds) {
    int year, month, day, hour, minute, second;
    int fields = sscanf(text, "%d-%d-%dT%d:%d:%dZ", &year, &month, &day,
                        &hour, &minute, &second);
    if (fields != 6) {
        fields = sscanf(text, "%d/%d/%d_%d:%d:%d", &month, &day, &year, &hour,
                        &minute, &second);
    }
    if (fields != 6) {
        fields = sscanf(text, "%d/%d/%d %d:%d:%d", &month, &day, &year, &hour,
                        &minute, &second);
    }
    if (fields == 6 && year < 100) {
        if (year < 100) {
            year += 2000;
        }
    }
    if (fields != 6 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 ||
        second > 59) {
        return false;
    }
    const int64_t epoch =
        daysFromCivil(year, month, day) * 86400LL + hour * 3600LL +
        minute * 60LL + second;
    if (epoch <= 0 || epoch > UINT32_MAX) {
        return false;
    }
    *epochSeconds = static_cast<uint32_t>(epoch);
    return true;
}

void printStatus() {
    char timeBuffer[16];
    formatTime(epochNow(), timeBuffer, sizeof(timeBuffer));
    Serial.printf("%s\n", timeBuffer);
    for (uint8_t nodeId = 1; nodeId <= AntennaNodeCount; ++nodeId) {
        const NodeStatus& status = nodeStatus[nodeIndex(nodeId)];
        Serial.printf("N%u %s az=%.1fT target=%.1fT flags=0x%02X\n", nodeId,
                      nodeOnline(nodeId) ? "ONLINE" : "OFFLINE",
                      status.azimuthDeciDegrees / 10.0F,
                      status.targetDeciDegrees == NoAzimuthDeciDegrees
                          ? -1.0F
                          : status.targetDeciDegrees / 10.0F,
                      status.flags);
        if (sensorTelemetryFresh(nodeId)) {
            Serial.printf(
                "  sensor flags=0x%02X heading=%.1fM "
                "mag=(%.1f,%.1f,%.1f)uT accel=(%.3f,%.3f,%.3f)g "
                "roll=%.1f pitch=%.1f\n",
                status.sensorFlags,
                status.magneticHeadingDeciDegrees / 10.0F,
                status.magneticXDeciMicrotesla / 10.0F,
                status.magneticYDeciMicrotesla / 10.0F,
                status.magneticZDeciMicrotesla / 10.0F,
                status.accelerationXMilliG / 1000.0F,
                status.accelerationYMilliG / 1000.0F,
                status.accelerationZMilliG / 1000.0F,
                status.rollDeciDegrees / 10.0F,
                status.pitchDeciDegrees / 10.0F);
        }
    }
}

void keypadKey(const char* key) {
    if (strcmp(key, "CLR") == 0) {
        keypadEntryLength = 0;
        keypadEntry[0] = '\0';
        snprintf(keypadFeedback, sizeof(keypadFeedback), "Cleared");
    } else if (strcmp(key, "CANCEL") == 0) {
        activePanel = Panel::Home;
    } else if (strcmp(key, "OK") == 0) {
        if (keypadEntryLength == 0) {
            snprintf(keypadFeedback, sizeof(keypadFeedback), "Enter an azimuth");
            Serial.println("Keypad: OK ignored; no azimuth entered.");
        } else {
            const float azimuth = atof(keypadEntry);
            if (azimuth >= 0.0F && azimuth < 360.0F) {
                Serial.printf("Keypad: sending node %u to %.1fT\n", keypadNodeId,
                              azimuth);
                sendCommand(keypadNodeId, CommandType::SetAzimuth,
                            static_cast<int16_t>(lroundf(azimuth * 10.0F)));
                activePanel = Panel::Home;
                keypadFeedback[0] = '\0';
            } else {
                snprintf(keypadFeedback, sizeof(keypadFeedback), "Use 0..359");
                Serial.println("ERR azimuth must be 0.0 through 359.9");
            }
        }
    } else if (strlen(key) == 1 && key[0] >= '0' && key[0] <= '9' &&
               keypadEntryLength < sizeof(keypadEntry) - 1) {
        keypadEntry[keypadEntryLength++] = key[0];
        keypadEntry[keypadEntryLength] = '\0';
        keypadFeedback[0] = '\0';
    }
    uiDirty = true;
}

void handleSerialLine(char* line) {
    if (line[0] == '{') {
        uint64_t utcMs = 0;
        if (parseClockCommand(line, &utcMs)) {
            const bool firstSync = !timeValid;
            setEpochMs(utcMs);
            if (firstSync) sendTimeSync(true);
            emitGatewayStatus();
            return;
        }
        ReportingCommand reportingCommand{};
        if (parseReportingCommand(line, &reportingCommand)) {
            esp_now_send(antennaNodeMac(reportingCommand.node), reinterpret_cast<const uint8_t*>(&reportingCommand), sizeof(reportingCommand));
            return;
        }
        CalibrationCommand calibrationCommand{};
        if (parseCalibrationCommand(line, &calibrationCommand)) {
            const esp_err_t result = esp_now_send(antennaNodeMac(calibrationCommand.node),
                reinterpret_cast<const uint8_t*>(&calibrationCommand), sizeof(calibrationCommand));
            // Only the node can acknowledge applying or saving calibration.
            if (result != ESP_OK) Serial.println("CAL gateway radio send failed");
            return;
        }
        SerialRotatorCommand serialCommand{};
        if (!parseSerialRotatorCommand(line, &serialCommand)) {
            Serial.println(
                "{\"t\":\"ra\",\"e\":1,\"detail\":\"invalid command\"}");
            return;
        }
        if (serialCommand.command == CommandType::None) {
            setEpoch(serialCommand.executeAtEpochSeconds);
            sendTimeSync(true);
        } else {
            sendCommand(
                serialCommand.nodeId, serialCommand.command,
                static_cast<int16_t>(
                    lroundf(serialCommand.valueDegrees * 10.0F)),
                serialCommand.executeAtEpochSeconds);
        }
        Serial.printf(
            "{\"t\":\"ra\",\"n\":%u,\"q\":%lu,\"c\":\"%s\","
            "\"e\":0,\"detail\":\"accepted by gateway\","
            "\"src\":\"gateway_serial\"}\n",
            serialCommand.nodeId,
            static_cast<unsigned long>(serialCommand.sequence),
            serialCommandName(serialCommand.command));
        return;
    }
    char keyword[16]{};
    sscanf(line, "%15s", keyword);
    if (strcmp(keyword, "HELP") == 0) {
        Serial.println(
            "TIME <ISO>; AZ n deg; STEP n +/-deg; QUEUE n ISO deg; STOP n; "
            "OVERLAP n deg; PANEL HOME|DEBUG; KEYPAD n; KEY digit|CLR|OK; STATUS");
    } else if (strcmp(keyword, "TIME") == 0) {
        const char* timestamp = line + 5;
        uint32_t epochSeconds = 0;
        if (parseTimestamp(timestamp, &epochSeconds)) {
            setEpoch(epochSeconds);
            sendTimeSync(true);
            Serial.printf("TIME set to %lu UTC seconds\n",
                          static_cast<unsigned long>(epochSeconds));
        } else {
            Serial.println("ERR TIME uses 2026-09-01T12:03:06Z");
        }
    } else if (strcmp(keyword, "AZ") == 0) {
        unsigned int nodeId;
        float azimuth;
        if (sscanf(line, "AZ %u %f", &nodeId, &azimuth) == 2) {
            sendCommand(nodeId, CommandType::SetAzimuth,
                        static_cast<int16_t>(lroundf(azimuth * 10.0F)));
        }
    } else if (strcmp(keyword, "STEP") == 0) {
        unsigned int nodeId;
        float delta;
        if (sscanf(line, "STEP %u %f", &nodeId, &delta) == 2) {
            sendCommand(nodeId, CommandType::StepAzimuth,
                        static_cast<int16_t>(lroundf(delta * 10.0F)));
        }
    } else if (strcmp(keyword, "QUEUE") == 0) {
        unsigned int nodeId;
        char timestamp[32]{};
        float azimuth;
        if (!timeValid) {
            Serial.println("ERR set TIME before QUEUE");
        } else if (sscanf(line, "QUEUE %u %31s %f", &nodeId, timestamp,
                          &azimuth) == 3) {
            uint32_t executeAt = 0;
            if (parseTimestamp(timestamp, &executeAt)) {
                sendCommand(nodeId, CommandType::QueueAzimuth,
                            static_cast<int16_t>(lroundf(azimuth * 10.0F)),
                            executeAt);
            } else {
                Serial.println("ERR QUEUE timestamp must be ISO UTC");
            }
        }
    } else if (strcmp(keyword, "STOP") == 0) {
        unsigned int nodeId;
        if (sscanf(line, "STOP %u", &nodeId) == 1) {
            sendCommand(nodeId, CommandType::Stop);
        }
    } else if (strcmp(keyword, "OVERLAP") == 0) {
        unsigned int nodeId;
        float azimuth;
        if (sscanf(line, "OVERLAP %u %f", &nodeId, &azimuth) == 2) {
            sendCommand(nodeId, CommandType::SetOverlapSouth,
                        static_cast<int16_t>(lroundf(azimuth * 10.0F)));
        }
    } else if (strcmp(keyword, "PANEL") == 0) {
        char panel[16]{};
        if (sscanf(line, "PANEL %15s", panel) == 1) {
            activePanel =
                strcmp(panel, "DEBUG") == 0 ? Panel::Debug : Panel::Home;
            uiDirty = true;
        }
    } else if (strcmp(keyword, "KEYPAD") == 0) {
        unsigned int nodeId;
        if (sscanf(line, "KEYPAD %u", &nodeId) == 1 && nodeIndex(nodeId) >= 0) {
            keypadNodeId = static_cast<uint8_t>(nodeId);
            keypadEntryLength = 0;
            keypadEntry[0] = '\0';
            activePanel = Panel::Keypad;
            uiDirty = true;
        }
    } else if (strcmp(keyword, "KEY") == 0) {
        char key[12]{};
        if (sscanf(line, "KEY %11s", key) == 1) {
            keypadKey(key);
        }
    } else if (strcmp(keyword, "STATUS") == 0) {
        printStatus();
    } else if (keyword[0] != '\0') {
        Serial.println("ERR unknown command; send HELP");
    }
}

void pollSerial() {
    while (Serial.available() > 0) {
        const char character = static_cast<char>(Serial.read());
        if (character == '\r') {
            continue;
        }
        if (character == '\n') {
            serialLine[serialLineLength] = '\0';
            if (!serialLineOverflow && serialLineLength) handleSerialLine(serialLine);
            serialLineLength = 0;
            serialLineOverflow = false;
        } else if (!serialLineOverflow && serialLineLength < SerialLineLength - 1) {
            serialLine[serialLineLength++] = character;
        } else {
            serialLineOverflow = true;
        }
    }
}

bool readRawTouch(int16_t* rawX, int16_t* rawY) {
    if (!touchscreen.tirqTouched() || !touchscreen.touched()) {
        return false;
    }
    const TS_Point point = touchscreen.getPoint();
    *rawX = point.x;
    *rawY = point.y;
    return true;
}

void calibrationTarget(uint8_t point, int* x, int* y) {
    const int margin = 35;
    switch (point) {
        case 0:
            *x = margin;
            *y = margin;
            break;
        case 1:
            *x = margin;
            *y = screenHeight() - margin;
            break;
        case 2:
            *x = screenWidth() - margin;
            *y = margin;
            break;
        default:
            *x = screenWidth() - margin;
            *y = screenHeight() - margin;
            break;
    }
}

void drawCalibrationScreen() {
    int x = 0;
    int y = 0;
    calibrationTarget(touchCalibrationPoint, &x, &y);
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_YELLOW, TFT_BLACK);
    display.drawCentreString("TOUCH CALIBRATION", screenWidth() / 2, 12, 4);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.drawCentreString("Touch the yellow cross", screenWidth() / 2, 52, 2);
    char progress[20];
    snprintf(progress, sizeof(progress), "Target %u of 4",
             touchCalibrationPoint + 1);
    display.drawCentreString(progress, screenWidth() / 2, 72, 2);
    display.drawCircle(x, y, 22, TFT_YELLOW);
    display.drawCircle(x, y, 12, TFT_YELLOW);
    display.drawFastHLine(x - 28, y, 57, TFT_YELLOW);
    display.drawFastVLine(x, y - 28, 57, TFT_YELLOW);
}

bool solveTouchCalibration() {
    const float rawX0 = touchRawX[0];
    const float rawY0 = touchRawY[0];
    const float rawX1 = touchRawX[1];
    const float rawY1 = touchRawY[1];
    const float rawX2 = touchRawX[2];
    const float rawY2 = touchRawY[2];
    int screenX[4];
    int screenY[4];
    for (uint8_t point = 0; point < 4; ++point) {
        calibrationTarget(point, &screenX[point], &screenY[point]);
    }

    const float determinant =
        rawX0 * (rawY1 - rawY2) + rawX1 * (rawY2 - rawY0) +
        rawX2 * (rawY0 - rawY1);
    if (fabsf(determinant) < 1.0F) {
        return false;
    }

    const auto solve = [&](float output0, float output1, float output2,
                           float* rawXCoefficient, float* rawYCoefficient,
                           float* offset) {
        *rawXCoefficient =
            (output0 * (rawY1 - rawY2) + output1 * (rawY2 - rawY0) +
             output2 * (rawY0 - rawY1)) /
            determinant;
        *rawYCoefficient =
            (rawX0 * (output1 - output2) + rawX1 * (output2 - output0) +
             rawX2 * (output0 - output1)) /
            determinant;
        *offset =
            (rawX0 * (rawY1 * output2 - rawY2 * output1) +
             rawX1 * (rawY2 * output0 - rawY0 * output2) +
             rawX2 * (rawY0 * output1 - rawY1 * output0)) /
            determinant;
    };

    solve(screenX[0], screenX[1], screenX[2],
          &touchCalibration.screenXFromRawX,
          &touchCalibration.screenXFromRawY, &touchCalibration.screenXOffset);
    solve(screenY[0], screenY[1], screenY[2],
          &touchCalibration.screenYFromRawX,
          &touchCalibration.screenYFromRawY, &touchCalibration.screenYOffset);
    touchCalibration.magic = TouchCalibrationMagic;
    return true;
}

void startTouchCalibration() {
    touchCalibrating = true;
    touchCalibrationPoint = 0;
    touchWasPressed = true;  // Require touch release before target 1 is sampled.
    drawCalibrationScreen();
    Serial.println("Touch calibration started; target 1 of 4.");
}

void initializeTouch() {
    touchscreenSpi.begin(TouchClockPin, TouchMisoPin, TouchMosiPin, TouchCsPin);
    touchscreen.begin(touchscreenSpi);
    touchscreen.setRotation(0);

    Preferences preferences;
    preferences.begin("antctrl", false);
    const bool calibrationAvailable =
        preferences.getBytesLength("touchAffine") == sizeof(touchCalibration);
    if (calibrationAvailable) {
        preferences.getBytes("touchAffine", &touchCalibration,
                             sizeof(touchCalibration));
    }
    preferences.end();

    if (calibrationAvailable && touchCalibration.magic == TouchCalibrationMagic) {
        Serial.println("Touch calibration loaded.");
    } else {
        startTouchCalibration();
    }
}

void finishTouchCalibration() {
    if (!solveTouchCalibration()) {
        Serial.println("Touch calibration invalid; restarting.");
        startTouchCalibration();
        return;
    }
    Preferences preferences;
    preferences.begin("antctrl", false);
    preferences.putBytes("touchAffine", &touchCalibration,
                          sizeof(touchCalibration));
    preferences.end();
    touchCalibrating = false;
    uiDirty = true;
    Serial.println("Touch calibration saved.");
}

bool isInside(uint16_t x,
              uint16_t y,
              int left,
              int top,
              int width,
              int height) {
    return x >= left && x < left + width && y >= top && y < top + height;
}

void handleTouch(uint16_t x, uint16_t y) {
    if (isInside(x, y, HeaderDebugButtonX, 0, 52, 24)) {
        activePanel = activePanel == Panel::Home ? Panel::Debug : Panel::Home;
        uiDirty = true;
        return;
    }

    if (activePanel == Panel::Home) {
        for (uint8_t nodeId = 1; nodeId <= AntennaNodeCount; ++nodeId) {
            const int cardY = nodeId == 1 ? 28 : 134;
            const int buttonY = cardY + 72;
            if (isInside(x, y, 10, buttonY, 88, 22)) {
                sendCommand(nodeId, CommandType::StepAzimuth, -100);
                return;
            }
            if (isInside(x, y, 116, buttonY, 88, 22)) {
                keypadNodeId = nodeId;
                keypadEntryLength = 0;
                keypadEntry[0] = '\0';
                activePanel = Panel::Keypad;
                uiDirty = true;
                return;
            }
            if (isInside(x, y, 222, buttonY, 88, 22)) {
                sendCommand(nodeId, CommandType::StepAzimuth, 100);
                return;
            }
        }
    } else if (activePanel == Panel::Keypad) {
        // Accept a generous lower-right touch region for the send action.
        if (isInside(x, y, 210, 184, 110, 42)) {
            Serial.printf("Touch keypad OK at %u,%u\n", x, y);
            keypadKey("OK");
            return;
        }
        const char* labels[] = {"1", "2", "3", "4", "5", "6",
                                "7", "8", "9", "CLR", "0", "OK"};
        for (int index = 0; index < 12; ++index) {
            const int column = index % 3;
            const int row = index / 3;
            if (isInside(x, y, 12 + column * 103, 95 + row * 31, 92, 26)) {
                Serial.printf("Touch keypad %s at %u,%u\n", labels[index], x,
                              y);
                keypadKey(labels[index]);
                return;
            }
        }
    }
}

void pollTouch() {
    int16_t rawX = 0;
    int16_t rawY = 0;
    const bool touchPressed = readRawTouch(&rawX, &rawY);
    if (touchPressed && !touchWasPressed) {
        if (touchCalibrating) {
            touchRawX[touchCalibrationPoint] = rawX;
            touchRawY[touchCalibrationPoint] = rawY;
            ++touchCalibrationPoint;
            if (touchCalibrationPoint == 4) {
                finishTouchCalibration();
            } else {
                touchWasPressed = true;
                drawCalibrationScreen();
                Serial.printf("Touch calibration target %u of 4.\n",
                              touchCalibrationPoint + 1);
            }
        } else {
            const int x = lroundf(touchCalibration.screenXFromRawX * rawX +
                                  touchCalibration.screenXFromRawY * rawY +
                                  touchCalibration.screenXOffset);
            const int y = lroundf(touchCalibration.screenYFromRawX * rawX +
                                  touchCalibration.screenYFromRawY * rawY +
                                  touchCalibration.screenYOffset);
            if (x >= 0 && x < screenWidth() && y >= 0 && y < screenHeight()) {
                handleTouch(x, y);
            }
        }
    }
    touchWasPressed = touchPressed;
}
}  // namespace

void setup() {
    Serial.begin(SerialBaudRate);
    pinMode(DisplayBacklightPin, OUTPUT);
    digitalWrite(DisplayBacklightPin, HIGH);

    display.init();
    display.setRotation(1);
    initializeTouch();
    if (!touchCalibrating) {
        redrawDisplay();
    }
    Serial.printf("CYD v0.1 antenna-control UI %d x %d; Wi-Fi MAC: %s\n",
                  screenWidth(), screenHeight(), WiFi.macAddress().c_str());
    if (!initializeEspNow()) {
        Serial.println("ESP-NOW initialization FAILED");
        return;
    }
    Serial.println("ESP-NOW ready; send HELP for USB serial commands.");
}

void loop() {
    if (touchCalibrating) {
        pollTouch();
        return;
    }
    pollSerial();
    pollTouch();
    processReceivedPackets();

    const uint32_t nowMs = millis();
    refreshNodeFreshness(nowMs);
    sendTimeSync();
    if (nowMs-lastGatewayReportMs >= 1000) {
        lastGatewayReportMs = nowMs;
        emitGatewayStatus();
    }
    const uint32_t nowEpoch = epochNow();
    if (nowEpoch != lastUiClockSecond) {
        lastUiClockSecond = nowEpoch;
        headerDirty = true;
    }
    const uint32_t motionBlinkPhase = millis() / 500UL;
    if (motionBlinkPhase != lastMotionBlinkPhase) {
        lastMotionBlinkPhase = motionBlinkPhase;
        for (uint8_t nodeId = 1; nodeId <= AntennaNodeCount; ++nodeId) {
            if (nodeOnline(nodeId) && hasFlag(nodeStatus[nodeIndex(nodeId)].flags,
                        StatusFlag::Moving)) {
                nodeCardDirty[nodeIndex(nodeId)] = true;
            }
        }
    }
    if (uiDirty) {
        redrawDisplay();
    } else if (activePanel == Panel::Home) {
        refreshHomePanel();
    } else if (activePanel == Panel::Debug && (headerDirty || debugDirty)) {
        redrawDisplay();
    }
}
