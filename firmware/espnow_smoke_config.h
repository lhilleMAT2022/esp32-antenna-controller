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
constexpr uint8_t ProtocolVersion = 3;
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
};

static_assert(sizeof(Packet) == 20, "Antenna-controller packet size changed");

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
