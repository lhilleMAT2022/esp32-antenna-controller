#pragma once
#include <ArduinoJson.h>
#include "espnow_smoke_config.h"
#include "node_runtime.h"

namespace antenna_controller {
struct __attribute__((packed)) ReportingCommand {
    uint8_t type = 8, node = 0, version = ProtocolVersion, mode = 1;
    uint32_t request = 0, durationSeconds = 0;
};
static_assert(sizeof(ReportingCommand) == 12, "Reporting command layout changed");

inline bool validReportingCommand(const ReportingCommand& command) {
    return command.type == 8 && command.version == ProtocolVersion &&
        command.node >= 1 && command.node <= AntennaNodeCount && command.request && command.mode <= 2 &&
        (command.mode == 2 ? command.durationSeconds > 0 && command.durationSeconds <= MaxQuietSeconds : command.durationSeconds == 0);
}
struct ReportingRequests {
    ReportingCommand recent[8]{};
    uint8_t next = 0;
    uint8_t apply(const ReportingCommand& command, ReportingPolicy& policy, uint32_t now, bool* changed) {
        *changed = false;
        if (!validReportingCommand(command)) return 2;
        for (const auto& previous : recent) {
            if (previous.request != command.request) continue;
            return previous.mode == command.mode && previous.durationSeconds == command.durationSeconds ? 0 : 2;
        }
        policy.set(ReportingMode(command.mode), command.durationSeconds, now);
        recent[next] = command; next = (next+1)%8;
        *changed = true;
        return 0;
    }
};
inline bool parseReportingCommand(const char* line, ReportingCommand* out) {
    JsonDocument doc;
    if (deserializeJson(doc, line) || doc["t"] != "rm" || !doc["n"].is<uint8_t>() ||
        !doc["q"].is<uint32_t>() || !doc["mode"].is<const char*>() || !doc["duration_s"].is<uint32_t>()) return false;
    const char* mode = doc["mode"];
    out->mode = !strcmp(mode, "continuous") ? 0 : !strcmp(mode, "normal") ? 1 : !strcmp(mode, "quiet") ? 2 : 255;
    out->node = doc["n"]; out->request = doc["q"]; out->durationSeconds = doc["duration_s"];
    return validReportingCommand(*out);
}
inline bool parseClockCommand(const char* line, uint64_t* utcMs) {
    JsonDocument doc;
    if (deserializeJson(doc, line) || doc["t"] != "gt" || !doc["utc_ms"].is<uint64_t>()) return false;
    *utcMs = doc["utc_ms"];
    return *utcMs > 0 && *utcMs <= 4102444800000ULL;
}
inline void emitJsonFrame(const JsonDocument& doc) {
    char line[768];
    if (measureJson(doc) >= sizeof(line)-1) return;
    const size_t size = serializeJson(doc, line, sizeof(line)-1);
    line[size] = '\n';
    Serial.write(reinterpret_cast<const uint8_t*>(line), size+1);
}
inline void emitRuntimeStatus(const Packet& packet, const char* source,
                              uint64_t gatewayRxMs = 0, int8_t gatewayRssi = RssiUnavailable) {
    JsonDocument doc;
    doc["t"] = "rp"; doc["n"] = packet.senderId; doc["q"] = packet.sequence;
    doc["ts"] = packet.epochSeconds; doc["ts_ms"] = packet.utcMilliseconds;
    doc["tv"] = hasFlag(packet.flags, StatusFlag::TimeValid);
    doc["h"] = packet.azimuthDeciDegrees/10.0F;
    if (packet.targetDeciDegrees != NoAzimuthDeciDegrees) doc["tg"] = packet.targetDeciDegrees/10.0F;
    else doc["tg"] = nullptr;
    doc["mv"] = hasFlag(packet.flags, StatusFlag::Moving); doc["e"] = 0;
    doc["ack"] = packet.packetType == uint8_t(PacketType::CommandAcknowledgment) ? packet.commandType : 0;
    doc["mode"] = reportingName(ReportingMode(packet.reportingMode));
    doc["quiet_left_ms"] = packet.quietRemainingMs; doc["up_ms"] = packet.uptimeMs;
    doc["sync_age_ms"] = packet.clockSyncAgeMs; doc["sync_step_ms"] = packet.clockCorrectionMs;
    doc["mr"] = packet.reportingRequest; doc["me"] = packet.reportingError;
    doc["boot"] = packet.boot; doc["cf"] = packet.calibrationFlags;
    doc["rn"] = packet.receiverRssiDbm;
    if (gatewayRxMs) doc["gw_rx_ms"] = gatewayRxMs;
    if (gatewayRssi != RssiUnavailable) doc["rg"] = gatewayRssi;
    doc["src"] = source;
    emitJsonFrame(doc);
}
inline void emitRawSensor(const SensorTelemetry& sample, const char* source) {
    JsonDocument doc;
    doc["t"] = "rs"; doc["n"] = sample.senderId; doc["q"] = sample.sequence;
    doc["mh"] = sample.magneticHeadingDeciDegrees/10.0F;
    auto mag = doc["m"].to<JsonArray>();
    auto acc = doc["a"].to<JsonArray>();
    mag.add(sample.magneticXDeciMicrotesla/10.0F); mag.add(sample.magneticYDeciMicrotesla/10.0F); mag.add(sample.magneticZDeciMicrotesla/10.0F);
    acc.add(sample.accelerationXMilliG/1000.0F); acc.add(sample.accelerationYMilliG/1000.0F); acc.add(sample.accelerationZMilliG/1000.0F);
    const auto mae = vectorMae(mag[0], mag[1], mag[2]);
    doc["f"] = mae.magnitude; doc["r"] = sample.rollDeciDegrees/10.0F;
    doc["p"] = sample.pitchDeciDegrees/10.0F; doc["sf"] = sample.flags; doc["src"] = source;
    emitJsonFrame(doc);
}
}  // namespace antenna_controller
