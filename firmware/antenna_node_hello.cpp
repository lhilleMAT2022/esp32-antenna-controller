#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

#include "espnow_smoke_config.h"

#ifndef ANTENNA_NODE_ID
#error "ANTENNA_NODE_ID must be defined by the PlatformIO environment"
#endif

namespace {
using namespace antenna_controller;

constexpr uint8_t BuiltInLedPin = 2;
constexpr uint32_t SerialBaudRate = 115200;
constexpr uint8_t ReceiveQueueSize = 8;
constexpr uint8_t ScheduledCommandCount = 4;
constexpr uint32_t RssiFreshnessMs = 10000;
constexpr float MaximumSpeedDegPerSec = 15.0F;  // 2.5 rpm
constexpr float AccelerationDegPerSec2 = 7.5F;  // 2 seconds to/from max speed
constexpr float MechanicalTravelDeg = 370.0F;
constexpr float PositionToleranceDeg = 0.5F;
constexpr uint32_t DirectionTogglePressMs = 100;
constexpr uint8_t NodeId = ANTENNA_NODE_ID;

struct ReceivedPacket {
    Packet packet;
};

struct ScheduledCommand {
    bool active = false;
    uint32_t executeAtEpochSeconds = 0;
    int16_t azimuthDeciDegrees = NoAzimuthDeciDegrees;
};

ReceivedPacket receiveQueue[ReceiveQueueSize]{};
volatile uint8_t receiveQueueWriteIndex = 0;
volatile uint8_t receiveQueueReadIndex = 0;
volatile int8_t cydRssiDbm = RssiUnavailable;
volatile uint32_t cydRssiUpdatedMs = 0;
uint32_t nextStatusMs = 500 + NodeId * 100;
uint32_t sequenceNumber = 0;
bool timeValid = false;
int32_t epochOffsetSeconds = 0;
ScheduledCommand scheduledCommands[ScheduledCommandCount]{};

float mechanicalPositionDeg = NodeId == 1 ? 95.0F : 270.0F;
float mechanicalVelocityDegPerSec = 0.0F;
float overlapSouthTrueDeg = 180.0F;
float targetMechanicalDeg = NAN;
bool modelButtonPressed = false;
int8_t activeDirection = 1;
int8_t nextPressDirection = 1;
bool directionToggleInProgress = false;
uint32_t togglePressReleaseMs = 0;
uint32_t lastModelUpdateMs = 0;

bool macMatches(const uint8_t* first, const uint8_t* second) {
    return memcmp(first, second, 6) == 0;
}

float normalizeDegrees(float value) {
    value = fmodf(value, 360.0F);
    return value < 0.0F ? value + 360.0F : value;
}

float moveToward(float current, float target, float maximumChange) {
    if (current < target) {
        return min(current + maximumChange, target);
    }
    return max(current - maximumChange, target);
}

int signOf(float value) {
    return value > 0.0F ? 1 : value < 0.0F ? -1 : 0;
}

uint32_t epochNow() {
    return timeValid
               ? static_cast<uint32_t>(
                     static_cast<int32_t>(millis() / 1000) + epochOffsetSeconds)
               : 0;
}

void synchronizeTime(uint32_t epochSeconds) {
    if (epochSeconds == 0) {
        return;
    }
    epochOffsetSeconds =
        static_cast<int32_t>(epochSeconds) - static_cast<int32_t>(millis() / 1000);
    timeValid = true;
}

float trueAzimuthDeg() {
    return normalizeDegrees(overlapSouthTrueDeg + mechanicalPositionDeg);
}

int16_t trueAzimuthDeciDegrees() {
    return static_cast<int16_t>(lroundf(trueAzimuthDeg() * 10.0F));
}

int16_t targetTrueAzimuthDeciDegrees() {
    return isnan(targetMechanicalDeg)
               ? NoAzimuthDeciDegrees
               : static_cast<int16_t>(
                     lroundf(normalizeDegrees(overlapSouthTrueDeg +
                                              targetMechanicalDeg) *
                             10.0F));
}

float mechanicalForTrueAzimuth(float trueAzimuth) {
    const float lowerCandidate =
        normalizeDegrees(trueAzimuth - overlapSouthTrueDeg);
    float selected = lowerCandidate;
    float selectedDistance = fabsf(selected - mechanicalPositionDeg);

    // The 0–10 degree mechanical region overlaps 360–370 degrees and represents
    // the same true bearings. Pick the reachable representation nearest to now.
    if (lowerCandidate <= MechanicalTravelDeg - 360.0F) {
        const float upperCandidate = lowerCandidate + 360.0F;
        const float upperDistance = fabsf(upperCandidate - mechanicalPositionDeg);
        if (upperDistance < selectedDistance) {
            selected = upperCandidate;
        }
    }
    return selected;
}

bool isMoving() {
    return modelButtonPressed || directionToggleInProgress ||
           fabsf(mechanicalVelocityDegPerSec) > 0.05F ||
           !isnan(targetMechanicalDeg);
}

uint8_t statusFlags() {
    uint8_t flags = 0;
    if (timeValid) {
        flags |= StatusFlag::TimeValid;
    }
    if (isMoving()) {
        flags |= StatusFlag::Moving;
    }
    if (modelButtonPressed) {
        flags |= StatusFlag::ModelButtonPressed;
    }
    for (const ScheduledCommand& item : scheduledCommands) {
        if (item.active) {
            flags |= StatusFlag::QueuePending;
            break;
        }
    }
    return flags;
}

void onPromiscuousPacket(void* rawPacket, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) {
        return;
    }

    const auto* packet = static_cast<const wifi_promiscuous_pkt_t*>(rawPacket);
    constexpr size_t SourceMacOffset = 10;
    if (packet->rx_ctrl.sig_len < SourceMacOffset + 6 ||
        !macMatches(packet->payload + SourceMacOffset, CydGatewayMac)) {
        return;
    }

    cydRssiDbm = packet->rx_ctrl.rssi;
    cydRssiUpdatedMs = millis();
}

int8_t latestCydRssi() {
    if (millis() - cydRssiUpdatedMs > RssiFreshnessMs) {
        return RssiUnavailable;
    }
    return cydRssiDbm;
}

void onDataSent(const uint8_t*, esp_now_send_status_t status) {
    Serial.printf("ESP-NOW delivery: %s\n",
                  status == ESP_NOW_SEND_SUCCESS ? "ok" : "failed");
}

void onDataReceived(const uint8_t*, const uint8_t* data, int dataLength) {
    if (dataLength != sizeof(Packet)) {
        return;
    }

    const uint8_t nextWriteIndex =
        (receiveQueueWriteIndex + 1) % ReceiveQueueSize;
    if (nextWriteIndex == receiveQueueReadIndex) {
        return;
    }
    memcpy(&receiveQueue[receiveQueueWriteIndex].packet, data, sizeof(Packet));
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

    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, CydGatewayMac, sizeof(peerInfo.peer_addr));
    peerInfo.channel = Channel;
    peerInfo.encrypt = false;
    return esp_now_add_peer(&peerInfo) == ESP_OK;
}

void sendPacket(PacketType packetType, CommandType command = CommandType::None) {
    const Packet packet{
        static_cast<uint8_t>(packetType),
        NodeId,
        ProtocolVersion,
        statusFlags(),
        ++sequenceNumber,
        epochNow(),
        trueAzimuthDeciDegrees(),
        targetTrueAzimuthDeciDegrees(),
        NoAzimuthDeciDegrees,
        static_cast<uint8_t>(command),
        latestCydRssi()};
    const esp_err_t result =
        esp_now_send(CydGatewayMac, reinterpret_cast<const uint8_t*>(&packet),
                     sizeof(packet));
    Serial.printf("TX %s az=%.1fT target=%.1fT %s\n",
                  packetType == PacketType::Status
                      ? "status"
                      : packetType == PacketType::CommandAcknowledgment
                            ? "command-ack"
                            : "packet",
                  trueAzimuthDeg(),
                  targetTrueAzimuthDeciDegrees() == NoAzimuthDeciDegrees
                      ? -1.0F
                      : targetTrueAzimuthDeciDegrees() / 10.0F,
                  result == ESP_OK ? "queued" : "failed");
}

void beginPress() {
    modelButtonPressed = true;
    activeDirection = nextPressDirection;
    nextPressDirection = -nextPressDirection;
}

void stopModel() {
    modelButtonPressed = false;
    directionToggleInProgress = false;
    targetMechanicalDeg = NAN;
}

void beginMotionTowardTarget() {
    if (isnan(targetMechanicalDeg)) {
        return;
    }
    const int desiredDirection =
        signOf(targetMechanicalDeg - mechanicalPositionDeg);
    if (desiredDirection == 0) {
        targetMechanicalDeg = NAN;
        return;
    }

    if (nextPressDirection == desiredDirection) {
        beginPress();
        return;
    }

    // A short non-actuating model press consumes the alternate-direction toggle.
    // It deliberately models the small movement a real controller would make.
    beginPress();
    directionToggleInProgress = true;
    togglePressReleaseMs = millis() + DirectionTogglePressMs;
}

void requestTrueAzimuth(float trueAzimuth) {
    targetMechanicalDeg = mechanicalForTrueAzimuth(normalizeDegrees(trueAzimuth));
    directionToggleInProgress = false;
    Serial.printf("MODEL target %.1fT (mechanical %.1f deg)\n", trueAzimuth,
                  targetMechanicalDeg);
}

bool queueAzimuth(uint32_t executeAtEpochSeconds, int16_t azimuthDeciDegrees) {
    for (ScheduledCommand& item : scheduledCommands) {
        if (!item.active) {
            item.active = true;
            item.executeAtEpochSeconds = executeAtEpochSeconds;
            item.azimuthDeciDegrees = azimuthDeciDegrees;
            return true;
        }
    }
    return false;
}

void serviceQueuedCommands() {
    if (!timeValid) {
        return;
    }
    const uint32_t now = epochNow();
    for (ScheduledCommand& item : scheduledCommands) {
        if (item.active && static_cast<int32_t>(now - item.executeAtEpochSeconds) >=
                               0) {
            requestTrueAzimuth(item.azimuthDeciDegrees / 10.0F);
            item.active = false;
            Serial.printf("MODEL queued command started at %lu\n",
                          static_cast<unsigned long>(now));
        }
    }
}

void updatePlantModel() {
    const uint32_t nowMs = millis();
    if (lastModelUpdateMs == 0) {
        lastModelUpdateMs = nowMs;
        return;
    }
    const float dt =
        min((nowMs - lastModelUpdateMs) / 1000.0F, 0.10F);
    lastModelUpdateMs = nowMs;

    const float desiredVelocity =
        modelButtonPressed ? activeDirection * MaximumSpeedDegPerSec : 0.0F;
    mechanicalVelocityDegPerSec =
        moveToward(mechanicalVelocityDegPerSec, desiredVelocity,
                   AccelerationDegPerSec2 * dt);
    mechanicalPositionDeg += mechanicalVelocityDegPerSec * dt;

    if (mechanicalPositionDeg < 0.0F) {
        mechanicalPositionDeg = -mechanicalPositionDeg;
        mechanicalVelocityDegPerSec = -mechanicalVelocityDegPerSec;
        activeDirection = -activeDirection;
        Serial.println("MODEL lower end-stop bounce");
    } else if (mechanicalPositionDeg > MechanicalTravelDeg) {
        mechanicalPositionDeg = 2.0F * MechanicalTravelDeg - mechanicalPositionDeg;
        mechanicalVelocityDegPerSec = -mechanicalVelocityDegPerSec;
        activeDirection = -activeDirection;
        Serial.println("MODEL upper end-stop bounce");
    }
}

void serviceMotionController() {
    const uint32_t nowMs = millis();
    if (directionToggleInProgress && nowMs >= togglePressReleaseMs) {
        modelButtonPressed = false;
        directionToggleInProgress = false;
    }

    if (isnan(targetMechanicalDeg) || directionToggleInProgress) {
        return;
    }

    const float error = targetMechanicalDeg - mechanicalPositionDeg;
    if (!modelButtonPressed && fabsf(mechanicalVelocityDegPerSec) <= 0.05F) {
        if (fabsf(error) <= PositionToleranceDeg) {
            targetMechanicalDeg = NAN;
            return;
        }
        beginMotionTowardTarget();
        return;
    }

    if (modelButtonPressed) {
        const int desiredDirection = signOf(error);
        const float stoppingDistance =
            mechanicalVelocityDegPerSec * mechanicalVelocityDegPerSec /
                (2.0F * AccelerationDegPerSec2) +
            PositionToleranceDeg;
        if (desiredDirection != activeDirection ||
            fabsf(error) <= stoppingDistance) {
            modelButtonPressed = false;
        }
    }
}

void handleCommand(const Packet& packet) {
    const auto command = static_cast<CommandType>(packet.commandType);
    switch (command) {
        case CommandType::SetAzimuth:
            requestTrueAzimuth(packet.commandValueDeciDegrees / 10.0F);
            break;
        case CommandType::StepAzimuth:
            requestTrueAzimuth(trueAzimuthDeg() +
                                packet.commandValueDeciDegrees / 10.0F);
            break;
        case CommandType::QueueAzimuth:
            if (!timeValid ||
                !queueAzimuth(packet.epochSeconds, packet.commandValueDeciDegrees)) {
                Serial.println("MODEL queue rejected (time invalid or full)");
            } else {
                Serial.printf("MODEL queued %.1fT for %lu\n",
                              packet.commandValueDeciDegrees / 10.0F,
                              static_cast<unsigned long>(packet.epochSeconds));
            }
            break;
        case CommandType::SetOverlapSouth:
            overlapSouthTrueDeg =
                normalizeDegrees(packet.commandValueDeciDegrees / 10.0F);
            Serial.printf("MODEL overlap south set to %.1fT\n",
                          overlapSouthTrueDeg);
            break;
        case CommandType::Stop:
            stopModel();
            Serial.println("MODEL stop");
            break;
        default:
            return;
    }
    sendPacket(PacketType::CommandAcknowledgment, command);
}

void processReceivedPackets() {
    while (receiveQueueReadIndex != receiveQueueWriteIndex) {
        const Packet packet = receiveQueue[receiveQueueReadIndex].packet;
        receiveQueueReadIndex =
            (receiveQueueReadIndex + 1) % ReceiveQueueSize;
        if (packet.protocolVersion != ProtocolVersion ||
            packet.senderId != static_cast<uint8_t>(DeviceId::CydGateway)) {
            continue;
        }
        if (packet.packetType == static_cast<uint8_t>(PacketType::TimeSync)) {
            synchronizeTime(packet.epochSeconds);
        } else if (packet.packetType ==
                   static_cast<uint8_t>(PacketType::Command)) {
            handleCommand(packet);
        }
    }
}
}  // namespace

void setup() {
    Serial.begin(SerialBaudRate);
    pinMode(BuiltInLedPin, OUTPUT);
    digitalWrite(BuiltInLedPin, LOW);

    Serial.printf("Antenna node %u v0.1 rotator MODEL; Wi-Fi MAC: %s\n", NodeId,
                  WiFi.macAddress().c_str());
    if (!initializeEspNow()) {
        Serial.println("ESP-NOW initialization FAILED");
        return;
    }
    Serial.println("ESP-NOW ready; no physical relay is actuated.");
}

void loop() {
    updatePlantModel();
    processReceivedPackets();
    serviceQueuedCommands();
    serviceMotionController();
    digitalWrite(BuiltInLedPin, modelButtonPressed ? HIGH : LOW);

    const uint32_t nowMs = millis();
    if (nowMs >= nextStatusMs) {
        sendPacket(PacketType::Status);
        nextStatusMs = nowMs + StatusPeriodMs;
    }
}
