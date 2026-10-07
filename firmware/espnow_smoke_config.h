#pragma once

#include <Arduino.h>

namespace antenna_controller {
constexpr uint8_t CydGatewayMac[6] = {
    0x04, 0xB2, 0x47, 0x82, 0x97, 0x18};
constexpr uint8_t AntennaNodeMac[6] = {
    0x5C, 0x01, 0x3B, 0x34, 0x44, 0xD8};
constexpr uint8_t AntennaNode2Mac[6] = {
    0x78, 0x42, 0x1C, 0x68, 0x39, 0x68};

constexpr uint8_t Channel = 1;
constexpr uint32_t TimeSyncPeriodMs = 3000;
constexpr uint32_t StatusPeriodMs = 1000;
constexpr uint8_t AntennaNodeCount = 2;
constexpr uint8_t ProtocolVersion = 5;
constexpr int8_t RssiUnavailable = 127;
constexpr int16_t NoAzimuthDeciDegrees = -1;

enum class DeviceId : uint8_t {
    CydGateway = 0,
    AntennaNode1 = 1,
    AntennaNode2 = 2,
};

enum class PacketType : uint8_t {
    Status = 1,
    Command = 2,
    CommandAcknowledgment = 3,
    TimeSync = 4,
    SensorTelemetry = 5,
    ReportingCommand = 8,
    ReportingAcknowledgment = 9,
};

enum class CommandType : uint8_t {
    None = 0,
    SetAzimuth = 1,
    StepAzimuth = 2,
    QueueAzimuth = 3,
    SetOverlapSouth = 4,
    Stop = 5,
};

enum StatusFlag : uint8_t {
    TimeValid = 1 << 0,
    Moving = 1 << 1,
    ModelButtonPressed = 1 << 2,
    QueuePending = 1 << 3,
};

enum SensorFlag : uint8_t {
    AccelerometerPresent = 1 << 0,
    MagnetometerPresent = 1 << 1,
    AccelerometerValid = 1 << 2,
    MagnetometerValid = 1 << 3,
};

struct __attribute__((packed)) Packet {
    uint8_t packetType;
    uint8_t senderId;
    uint8_t protocolVersion;
    uint8_t flags;
    uint32_t sequence;
    uint32_t epochSeconds;
    int16_t azimuthDeciDegrees;
    int16_t targetDeciDegrees;
    int16_t commandValueDeciDegrees;
    uint8_t commandType;
    int8_t receiverRssiDbm;
    uint64_t utcMilliseconds;
    uint32_t uptimeMs;
    uint8_t reportingMode;
    uint32_t quietRemainingMs;
    uint32_t clockSyncAgeMs;
    int32_t clockCorrectionMs;
    uint32_t reportingRequest;
    uint8_t reportingError;
    uint32_t boot;
    uint8_t calibrationFlags;
    int32_t clockErrorMs;
    int16_t clockRatePpm;
    uint8_t clockState;
};

static_assert(sizeof(Packet) == 62, "Antenna-controller packet size changed");

struct __attribute__((packed)) SensorTelemetry {
    uint8_t packetType;
    uint8_t senderId;
    uint8_t protocolVersion;
    uint8_t flags;
    uint32_t sequence;
    int16_t magneticHeadingDeciDegrees;
    int16_t magneticXDeciMicrotesla;
    int16_t magneticYDeciMicrotesla;
    int16_t magneticZDeciMicrotesla;
    int16_t accelerationXMilliG;
    int16_t accelerationYMilliG;
    int16_t accelerationZMilliG;
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
};

static_assert(sizeof(SensorTelemetry) == 26,
              "Sensor-telemetry packet size changed");

inline const uint8_t* antennaNodeMac(uint8_t nodeId) {
    switch (nodeId) {
        case static_cast<uint8_t>(DeviceId::AntennaNode1):
            return AntennaNodeMac;
        case static_cast<uint8_t>(DeviceId::AntennaNode2):
            return AntennaNode2Mac;
        default:
            return nullptr;
    }
}

inline bool hasFlag(uint8_t flags, StatusFlag flag) {
    return (flags & static_cast<uint8_t>(flag)) != 0;
}
}  // namespace antenna_controller
