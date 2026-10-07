#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <Preferences.h>
#include <esp_timer.h>

#ifdef ANTENNA_NODE_HAS_LSM303AGR
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>
#include <Wire.h>
#endif

#include "espnow_smoke_config.h"
#include "serial_json_protocol.h"
#include "calibration_protocol.h"
#include "runtime_protocol.h"

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
constexpr size_t SerialLineLength = 768;

#ifdef ANTENNA_NODE_HAS_LSM303AGR
constexpr uint8_t SensorSdaPin = 21;
constexpr uint8_t SensorSclPin = 22;
constexpr uint32_t SensorSamplePeriodMs = 100;
constexpr float StandardGravityMetersPerSecondSquared = 9.80665F;

Adafruit_LSM303_Accel_Unified accelerometer(30301);
Adafruit_LIS2MDL magnetometer(30302);
bool accelerometerPresent = false;
bool magnetometerPresent = false;
bool accelerometerValid = false;
bool magnetometerValid = false;
uint32_t nextSensorSampleMs = 0;
float accelerationX = 0.0F;
float accelerationY = 0.0F;
float accelerationZ = 0.0F;
float magneticX = 0.0F;
float magneticY = 0.0F;
float magneticZ = 0.0F;
float magneticHeadingDeg = 0.0F;
float rollDeg = 0.0F;
float pitchDeg = 0.0F;
#endif

struct ReceivedPacket {
    uint8_t payload[sizeof(CalibrationCommand)];
    uint8_t length;
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
uint32_t lastSerialReportMs = 0;
bool serialReported = false;
uint32_t sequenceNumber = 0;
bool timeValid = false;
UtcClock utcClock;
ReportingPolicy reporting;
OrientationReference quietOrientation;
ReportingRequests reportingRequests;
uint8_t lastCalibrationFlags = 0;
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
char serialLine[SerialLineLength]{};
size_t serialLineLength = 0;
bool serialLineOverflow = false;
CalibrationConfig calibration = defaultCalibration();
bool calibrationSaved = false;
bool calibrationStorageInvalid = false;
uint32_t calibrationBoot = 0;

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
    return uint32_t(utcClock.at(uint64_t(esp_timer_get_time())/1000)/1000);
}

void synchronizeTime(uint64_t epochMilliseconds) {
    utcClock.synchronize(epochMilliseconds, uint64_t(esp_timer_get_time())/1000);
    timeValid = utcClock.valid;
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

void onDataReceived(const uint8_t* source, const uint8_t* data, int dataLength) {
    if (!macMatches(source, CydGatewayMac) ||
        (dataLength != sizeof(Packet) && dataLength != sizeof(CalibrationCommand) &&
         dataLength != sizeof(ReportingCommand))) {
        return;
    }

    const uint8_t nextWriteIndex =
        (receiveQueueWriteIndex + 1) % ReceiveQueueSize;
    if (nextWriteIndex == receiveQueueReadIndex) {
        return;
    }
    memcpy(receiveQueue[receiveQueueWriteIndex].payload, data, dataLength);
    receiveQueue[receiveQueueWriteIndex].length = dataLength;
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

void sendPacket(PacketType packetType, CommandType command = CommandType::None,
                bool radio = true, bool serialOutput = true,
                uint32_t reportingRequest = 0, uint8_t reportingError = 0) {
    Packet packet{
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
    const uint64_t now = uint64_t(esp_timer_get_time())/1000;
    packet.utcMilliseconds = utcClock.at(now); packet.uptimeMs = millis();
    packet.reportingMode = uint8_t(reporting.mode); packet.quietRemainingMs = reporting.remaining(millis());
    packet.clockSyncAgeMs = utcClock.syncAge(now); packet.clockCorrectionMs = utcClock.correctionMs;
    packet.reportingRequest = reportingRequest; packet.reportingError = reportingError;
    packet.boot = calibrationBoot; packet.calibrationFlags = lastCalibrationFlags;
    if (radio) esp_now_send(CydGatewayMac, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    if (serialOutput) emitRuntimeStatus(packet, "node_serial");
}

void handleReportingCommand(const ReportingCommand& command) {
    if (command.node != NodeId || !command.request || command.type != 8 || command.version != ProtocolVersion) return;
    bool changed = false;
    const uint8_t error = reportingRequests.apply(command, reporting, millis(), &changed);
    if (changed) quietOrientation.valid = false;
    sendPacket(PacketType::ReportingAcknowledgment, CommandType::None, true, true, command.request, error);
}

#ifdef ANTENNA_NODE_HAS_LSM303AGR
int16_t scaledSensorValue(float value, float scale) {
    const float scaled = value * scale;
    return static_cast<int16_t>(
        lroundf(constrain(scaled, -32767.0F, 32767.0F)));
}

void initializeOrientationSensor() {
    Wire.begin(SensorSdaPin, SensorSclPin);
    Wire.setClock(400000);

    accelerometerPresent = accelerometer.begin();
    magnetometerPresent = magnetometer.begin();
    if (accelerometerPresent) {
        accelerometer.setRange(LSM303_RANGE_4G);
        accelerometer.setMode(LSM303_MODE_HIGH_RESOLUTION);
    }
    if (magnetometerPresent) {
        magnetometer.enableAutoRange(true);
        magnetometer.setDataRate(LIS2MDL_RATE_100_HZ);
    }

    Serial.printf(
        "LSM303AGR I2C SDA=%u SCL=%u accelerometer=%s magnetometer=%s\n",
        SensorSdaPin, SensorSclPin, accelerometerPresent ? "found" : "missing",
        magnetometerPresent ? "found" : "missing");
}

void sampleOrientationSensor() {
    const uint32_t nowMs = millis();
    if (nowMs < nextSensorSampleMs) {
        return;
    }
    nextSensorSampleMs = nowMs + SensorSamplePeriodMs;

    if (accelerometerPresent) {
        sensors_event_t event{};
        accelerometerValid = accelerometer.getEvent(&event);
        if (accelerometerValid) {
            accelerationX = event.acceleration.x;
            accelerationY = event.acceleration.y;
            accelerationZ = event.acceleration.z;
            rollDeg = atan2f(accelerationY, accelerationZ) * 180.0F / PI;
            pitchDeg =
                atan2f(-accelerationX,
                       sqrtf(accelerationY * accelerationY +
                             accelerationZ * accelerationZ)) *
                180.0F / PI;
        }
    }

    if (magnetometerPresent) {
        sensors_event_t event{};
        magnetometerValid = magnetometer.getEvent(&event);
        if (magnetometerValid) {
            magneticX = event.magnetic.x;
            magneticY = event.magnetic.y;
            magneticZ = event.magnetic.z;
            magneticHeadingDeg =
                normalizeDegrees(atan2f(magneticY, magneticX) * 180.0F / PI);
        }
    }
}

uint8_t sensorFlags() {
    uint8_t flags = 0;
    if (accelerometerPresent) {
        flags |= SensorFlag::AccelerometerPresent;
    }
    if (magnetometerPresent) {
        flags |= SensorFlag::MagnetometerPresent;
    }
    if (accelerometerValid) {
        flags |= SensorFlag::AccelerometerValid;
    }
    if (magnetometerValid) {
        flags |= SensorFlag::MagnetometerValid;
    }
    return flags;
}

void sendSensorTelemetry(bool radio = true, bool serialOutput = true) {
    const int16_t magneticHeading =
        magnetometerValid
            ? scaledSensorValue(magneticHeadingDeg, 10.0F)
            : static_cast<int16_t>(NoAzimuthDeciDegrees);
    const int16_t magneticXValue =
        magnetometerValid ? scaledSensorValue(magneticX, 10.0F)
                          : static_cast<int16_t>(0);
    const int16_t magneticYValue =
        magnetometerValid ? scaledSensorValue(magneticY, 10.0F)
                          : static_cast<int16_t>(0);
    const int16_t magneticZValue =
        magnetometerValid ? scaledSensorValue(magneticZ, 10.0F)
                          : static_cast<int16_t>(0);
    const int16_t accelerationXValue =
        accelerometerValid
            ? scaledSensorValue(
                  accelerationX / StandardGravityMetersPerSecondSquared, 1000.0F)
            : static_cast<int16_t>(0);
    const int16_t accelerationYValue =
        accelerometerValid
            ? scaledSensorValue(
                  accelerationY / StandardGravityMetersPerSecondSquared, 1000.0F)
            : static_cast<int16_t>(0);
    const int16_t accelerationZValue =
        accelerometerValid
            ? scaledSensorValue(
                  accelerationZ / StandardGravityMetersPerSecondSquared, 1000.0F)
            : static_cast<int16_t>(0);
    const int16_t rollValue =
        accelerometerValid ? scaledSensorValue(rollDeg, 10.0F)
                           : static_cast<int16_t>(0);
    const int16_t pitchValue =
        accelerometerValid ? scaledSensorValue(pitchDeg, 10.0F)
                           : static_cast<int16_t>(0);

    const SensorTelemetry telemetry{
        static_cast<uint8_t>(PacketType::SensorTelemetry),
        NodeId,
        ProtocolVersion,
        sensorFlags(),
        ++sequenceNumber,
        magneticHeading,
        magneticXValue,
        magneticYValue,
        magneticZValue,
        accelerationXValue,
        accelerationYValue,
        accelerationZValue,
        rollValue,
        pitchValue};

    if (radio) esp_now_send(CydGatewayMac, reinterpret_cast<const uint8_t*>(&telemetry), sizeof(telemetry));
    if (serialOutput) emitRawSensor(telemetry, "node_serial");
}
#endif

void loadCalibration() {
    calibrationBoot = esp_random();
#ifdef ANTENNA_NODE_HAS_LSM303AGR
    Preferences preferences;
    // Read-only opening does not create a namespace or write defaults.
    if (!preferences.begin("sensor-cal", true)) return;
    CalibrationConfig stored{};
    const int result = readCalibrationRecord(preferences, stored);
    if (result == 1) {
        calibration = stored;
        calibrationSaved = (stored.flags & 7) != 0;
    } else if (result < 0) calibrationStorageInvalid = true;
    preferences.end();
#endif
}

bool persistCalibration(CalibrationConfig candidate) {
    Preferences preferences;
    if (!preferences.begin("sensor-cal", false)) return false;
    const bool ok = writeCalibrationRecord(preferences, candidate);
    preferences.end();
    if (ok) {
        calibration = candidate;
        calibrationSaved = (candidate.flags & 7) != 0;
        calibrationStorageInvalid = false;
    }
    return ok;
}

void sendCalibrationReport(uint32_t request = 0, uint8_t error = 0, bool radio = true, bool serialOutput = true) {
    CalibrationReport report{};
    report.type = 7; report.node = NodeId; report.version = ProtocolVersion;
    report.sequence = ++sequenceNumber; report.boot = calibrationBoot;
    report.request = request; report.error = error;
    report.flags = calibration.flags | (calibrationSaved ? CalibrationSaved : 0) |
        (calibrationStorageInvalid ? CalibrationStorageInvalid : 0);
#ifdef ANTENNA_NODE_HAS_LSM303AGR
    const float rawMag[3] = {magneticX, magneticY, magneticZ};
    const float rawAccel[3] = {accelerationX/StandardGravityMetersPerSecondSquared,
        accelerationY/StandardGravityMetersPerSecondSquared, accelerationZ/StandardGravityMetersPerSecondSquared};
    // Use aligned local arrays rather than taking pointers into packed packets.
    float mag[3]{}, acc[3]{};
    correctVector(rawMag, calibration.magOffset, calibration.magMatrix, mag);
    correctVector(rawAccel, calibration.accelOffset, calibration.accelMatrix, acc);
    const bool magOk = magnetometerValid && std::isfinite(dot3(mag, mag));
    const bool accOk = accelerometerValid && std::isfinite(dot3(acc, acc));
    if (magOk) report.flags |= CalibrationMagValid;
    if (accOk) report.flags |= CalibrationAccelValid;
    memcpy(report.magnetic, mag, sizeof(mag));
    memcpy(report.acceleration, acc, sizeof(acc));
    float heading = 0, tilt = 0;
    if ((calibration.flags & 7) == 7 && magOk && accOk &&
        calibratedOrientation(calibration, mag, acc, &heading, &tilt)) {
        report.flags |= CalibrationHeadingValid;
        report.heading = heading; report.tilt = tilt;
    }
#endif
    lastCalibrationFlags = report.flags;
    if (radio) esp_now_send(CydGatewayMac, reinterpret_cast<const uint8_t*>(&report), sizeof(report));
    if (serialOutput) emitCalibrationJson(report, "node_serial");
}

void handleCalibrationCommand(const CalibrationCommand& command) {
    if (command.type != 6 || command.version != ProtocolVersion ||
        command.node != NodeId || command.request == 0) return;
#ifndef ANTENNA_NODE_HAS_LSM303AGR
    sendCalibrationReport(command.request, 1); // Unsupported sensor hardware.
#else
    CalibrationConfig candidate = calibration;
    const auto op = static_cast<CalibrationOperation>(command.operation);
    uint8_t error = 0;
    if (op == CalibrationOperation::Mag || op == CalibrationOperation::Accel) {
        const bool mag = op == CalibrationOperation::Mag;
        if ((mag && !magnetometerPresent) || (!mag && !accelerometerPresent)) error = 1;
        memcpy(mag ? candidate.magOffset : candidate.accelOffset, command.values, 3*sizeof(float));
        memcpy(mag ? candidate.magMatrix : candidate.accelMatrix, command.values+3, 9*sizeof(float));
        candidate.flags |= mag ? MagCalibrated : AccelCalibrated;
    } else if (op == CalibrationOperation::Align) {
        const float forward = command.values[0], up = command.values[1];
        if (!std::isfinite(forward) || !std::isfinite(up) ||
            std::fabs(forward) < 1 || std::fabs(forward) > 3 ||
            std::fabs(up) < 1 || std::fabs(up) > 3 ||
            forward != std::trunc(forward) || up != std::trunc(up)) error = 2;
        else { candidate.forwardAxis = forward; candidate.upAxis = up; }
        candidate.declination = command.values[2];
        candidate.flags |= MountConfigured;
    } else if (op == CalibrationOperation::Save) {
        if (!(calibration.flags & 7)) error = 2;
        else if (!persistCalibration(calibration)) { error = 3; calibrationSaved = false; }
    } else if (op == CalibrationOperation::Clear) {
        if (!persistCalibration(defaultCalibration())) error = 3;
    } else if (op != CalibrationOperation::Status) error = 2;
    if (op == CalibrationOperation::Mag || op == CalibrationOperation::Accel || op == CalibrationOperation::Align) {
        if (!validCalibration(candidate)) error = 2;
        if (!error) {
            candidate.checksum = calibrationChecksum(candidate);
            if (memcmp(&candidate, &calibration, sizeof(candidate))) {
                calibration = candidate;
                calibrationSaved = false;
            }
        }
    }
    sendCalibrationReport(command.request, error);
#endif
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
        const ReceivedPacket received = receiveQueue[receiveQueueReadIndex];
        receiveQueueReadIndex =
            (receiveQueueReadIndex + 1) % ReceiveQueueSize;
        if (received.length == sizeof(ReportingCommand)) {
            ReportingCommand command{};
            memcpy(&command, received.payload, sizeof(command));
            handleReportingCommand(command);
            continue;
        }
        if (received.length == sizeof(CalibrationCommand)) {
            CalibrationCommand command{};
            memcpy(&command, received.payload, sizeof(command));
            handleCalibrationCommand(command);
            continue;
        }
        Packet packet{};
        memcpy(&packet, received.payload, sizeof(packet));
        if (packet.protocolVersion != ProtocolVersion ||
            packet.senderId != static_cast<uint8_t>(DeviceId::CydGateway)) {
            continue;
        }
        if (packet.packetType == static_cast<uint8_t>(PacketType::TimeSync)) {
            synchronizeTime(packet.utcMilliseconds);
        } else if (packet.packetType ==
                   static_cast<uint8_t>(PacketType::Command)) {
            handleCommand(packet);
        }
    }
}

void emitSerialCommandResult(const SerialRotatorCommand& command,
                             bool accepted,
                             const char* detail) {
    Serial.printf(
        "{\"t\":\"ra\",\"n\":%u,\"q\":%lu,\"c\":\"%s\","
        "\"e\":%u,\"detail\":\"%s\",\"src\":\"node_serial\"}\n",
        NodeId, static_cast<unsigned long>(command.sequence),
        serialCommandName(command.command), accepted ? 0U : 1U, detail);
}

void handleSerialJson(char* line) {
    ReportingCommand reportingCommand{};
    if (parseReportingCommand(line, &reportingCommand)) {
        handleReportingCommand(reportingCommand);
        return;
    }
    uint64_t utcMs = 0;
    if (parseClockCommand(line, &utcMs)) {
        synchronizeTime(utcMs);
        return;
    }
    CalibrationCommand calibrationCommand{};
    if (parseCalibrationCommand(line, &calibrationCommand)) {
        handleCalibrationCommand(calibrationCommand);
        return;
    }
    SerialRotatorCommand serialCommand{};
    if (!parseSerialRotatorCommand(line, &serialCommand) ||
        serialCommand.nodeId != NodeId) {
        Serial.println(
            "{\"t\":\"ra\",\"e\":1,\"detail\":\"invalid command or node\"}");
        return;
    }

    if (serialCommand.command == CommandType::None) {
        synchronizeTime(uint64_t(serialCommand.executeAtEpochSeconds)*1000);
        emitSerialCommandResult(serialCommand, true, "time synchronized");
        return;
    }

    Packet packet{};
    packet.packetType = static_cast<uint8_t>(PacketType::Command);
    packet.senderId = static_cast<uint8_t>(DeviceId::CydGateway);
    packet.protocolVersion = ProtocolVersion;
    packet.sequence = serialCommand.sequence;
    packet.epochSeconds = serialCommand.executeAtEpochSeconds;
    packet.commandValueDeciDegrees =
        static_cast<int16_t>(lroundf(serialCommand.valueDegrees * 10.0F));
    packet.commandType = static_cast<uint8_t>(serialCommand.command);
    handleCommand(packet);
    emitSerialCommandResult(serialCommand, true, "accepted");
}

void pollSerial() {
    while (Serial.available() > 0) {
        const char character = static_cast<char>(Serial.read());
        if (character == '\r') {
            continue;
        }
        if (character == '\n') {
            serialLine[serialLineLength] = '\0';
            if (serialLineLength > 0 && !serialLineOverflow) {
                handleSerialJson(serialLine);
            }
            serialLineLength = 0;
            serialLineOverflow = false;
        } else if (!serialLineOverflow && serialLineLength < SerialLineLength - 1) {
            serialLine[serialLineLength++] = character;
        } else {
            serialLineOverflow = true;
        }
    }
}
}  // namespace

void setup() {
    Serial.begin(SerialBaudRate);
    loadCalibration();
    pinMode(BuiltInLedPin, OUTPUT);
    digitalWrite(BuiltInLedPin, LOW);

    Serial.printf("Antenna node %u v0.1 rotator MODEL; Wi-Fi MAC: %s\n", NodeId,
                  WiFi.macAddress().c_str());
#ifdef ANTENNA_NODE_HAS_LSM303AGR
    initializeOrientationSensor();
#endif
    if (!initializeEspNow()) {
        Serial.println("ESP-NOW initialization FAILED");
        return;
    }
    Serial.println("ESP-NOW ready; no physical relay is actuated.");
}

void loop() {
    pollSerial();
#ifdef ANTENNA_NODE_HAS_LSM303AGR
    sampleOrientationSensor();
#endif
    updatePlantModel();
    processReceivedPackets();
    serviceQueuedCommands();
    serviceMotionController();
    digitalWrite(BuiltInLedPin, modelButtonPressed ? HIGH : LOW);

    const uint32_t nowMs = millis();
    const bool moving = isMoving();
    float unexpectedTurn = NAN;
#ifdef ANTENNA_NODE_HAS_LSM303AGR
    float correctedMag[3], correctedAccel[3];
    const float rawMag[3] = {magneticX, magneticY, magneticZ};
    const float rawAccel[3] = {accelerationX/StandardGravityMetersPerSecondSquared,
        accelerationY/StandardGravityMetersPerSecondSquared, accelerationZ/StandardGravityMetersPerSecondSquared};
    correctVector(rawMag, calibration.magOffset, calibration.magMatrix, correctedMag);
    correctVector(rawAccel, calibration.accelOffset, calibration.accelMatrix, correctedAccel);
    if (magnetometerValid && accelerometerValid)
        unexpectedTurn = quietOrientation.change(correctedAccel, correctedMag,
                                                moving || reporting.mode != ReportingMode::Quiet);
#endif
    const RadioReports radio = reporting.poll(nowMs, moving, unexpectedTurn);
    const bool serialTick = !serialReported || nowMs-lastSerialReportMs >= StatusPeriodMs;
    if (serialTick) { lastSerialReportMs = nowMs; serialReported = true; }
    if (serialTick || radio.sensor) sendCalibrationReport(0, 0, radio.sensor, serialTick);
    if (serialTick || radio.heartbeat) sendPacket(PacketType::Status, CommandType::None, radio.heartbeat, serialTick);
#ifdef ANTENNA_NODE_HAS_LSM303AGR
    if (serialTick || radio.sensor) sendSensorTelemetry(radio.sensor, serialTick);
    if (radio.sensor && magnetometerValid && accelerometerValid)
        quietOrientation.change(correctedAccel, correctedMag, true);
#endif
}
