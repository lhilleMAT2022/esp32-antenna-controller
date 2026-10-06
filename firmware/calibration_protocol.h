#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "espnow_smoke_config.h"
#include "sensor_calibration.h"

namespace antenna_controller {

inline bool parseCalibrationCommand(const char* line, CalibrationCommand* out) {
    JsonDocument doc;
    if (deserializeJson(doc, line) || !doc.is<JsonObject>() ||
        doc["t"] != "cc" || !doc["n"].is<unsigned int>() ||
        !doc["q"].is<uint32_t>() || !doc["op"].is<const char*>() ||
        !doc["v"].is<JsonArray>()) return false;
    const unsigned int node = doc["n"];
    if (node < 1 || node > AntennaNodeCount || doc["q"].as<uint32_t>() == 0) return false;
    const char* op = doc["op"];
    unsigned count = 0;
    if (!strcmp(op, "mag")) { out->operation = 1; count = 12; }
    else if (!strcmp(op, "accel")) { out->operation = 2; count = 12; }
    else if (!strcmp(op, "align")) { out->operation = 3; count = 3; }
    else if (!strcmp(op, "save")) out->operation = 4;
    else if (!strcmp(op, "clear")) out->operation = 5;
    else if (!strcmp(op, "status")) out->operation = 6;
    else return false;
    const JsonArray values = doc["v"].as<JsonArray>();
    if (values.size() != count) return false;
    for (unsigned i = 0; i < count; ++i) {
        if (!values[i].is<float>()) return false;
        out->values[i] = values[i].as<float>();
        if (!std::isfinite(out->values[i])) return false;
    }
    out->type = 6;
    out->node = node;
    out->version = ProtocolVersion;
    out->request = doc["q"];
    return true;
}

inline void emitCalibrationJson(const CalibrationReport& r, const char* source) {
    JsonDocument doc;
    doc["t"] = "cs"; doc["n"] = r.node; doc["q"] = r.sequence;
    doc["boot"] = r.boot; doc["req"] = r.request;
    doc["cf"] = r.flags; doc["e"] = r.error; doc["src"] = source;
    auto mag = doc["mc"].to<JsonArray>();
    auto acc = doc["ac"].to<JsonArray>();
    for (int i = 0; i < 3; ++i) {
        if (r.flags & CalibrationMagValid) mag.add(r.magnetic[i]); else mag.add(nullptr);
        if (r.flags & CalibrationAccelValid) acc.add(r.acceleration[i]); else acc.add(nullptr);
    }
    if (r.flags & CalibrationHeadingValid) {
        doc["ch"] = r.heading;
        doc["ti"] = r.tilt;
    } else { doc["ch"] = nullptr; doc["ti"] = nullptr; }
    // A single write prevents asynchronous radio diagnostics from splitting
    // this JSON frame between serializeJson's individual writes.
    char line[512];
    if (measureJson(doc) >= sizeof(line)-1) return;
    const size_t length = serializeJson(doc, line, sizeof(line)-1);
    line[length] = '\n';
    Serial.write(reinterpret_cast<const uint8_t*>(line), length+1);
}

} // namespace antenna_controller
