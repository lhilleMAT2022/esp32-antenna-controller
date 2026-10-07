#pragma once
#include "runtime_protocol.h"
#include "slew_plan.h"
namespace antenna_controller {
static_assert(ProtocolVersion == 7, "Update slew protocol version with wire protocol");
inline bool parseSlewCommand(const char* line, SlewCommand* out) {
    JsonDocument doc;
    if (deserializeJson(doc,line) || doc["t"]!="sc" || !doc["n"].is<uint8_t>() ||
        !doc["q"].is<uint32_t>() || !doc["at"].is<uint32_t>() ||
        !doc["dur"].is<uint32_t>() || !doc["steps"].is<uint16_t>() || !doc["coef"].is<JsonArray>()) return false;
    const auto coef=doc["coef"].as<JsonArray>();
    if (coef.size()<1 || coef.size()>SlewCoefficientCount || doc["dur"].as<uint32_t>()>86400) return false;
    out->node=doc["n"]; out->request=doc["q"]; out->startMs=uint64_t(doc["at"].as<uint32_t>())*1000;
    out->durationMs=doc["dur"].as<uint32_t>()*1000; out->steps=doc["steps"]; out->count=coef.size();
    for (uint8_t i=0;i<out->count;++i) {
        if (!coef[i].is<double>()) return false;
        out->coefficients[i]=coef[i].as<double>();
    }
    return validSlew(*out);
}
inline const char* slewPhaseName(uint8_t phase) {
    switch (SlewPhase(phase)) {
        case SlewPhase::Queued: return "accepted";
        case SlewPhase::Started: return "started";
        case SlewPhase::TargetsSent: return "targets_sent";
        case SlewPhase::Cancelled: return "cancelled";
        default: return "rejected";
    }
}
inline void emitSlewReply(const SlewReply& reply, const char* source) {
    JsonDocument doc;
    doc["t"]="sa"; doc["n"]=reply.node; doc["q"]=reply.request;
    doc["phase"]=slewPhaseName(reply.phase); doc["e"]=reply.error; doc["src"]=source;
    emitJsonFrame(doc);
}
} // namespace antenna_controller
