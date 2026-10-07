#include <cassert>
#include "../../firmware/runtime_protocol.h"
using namespace antenna_controller;

int main() {
    auto mae = vectorMae(0, -1, 1);
    assert(fabsf(mae.magnitude-sqrtf(2)) < 1e-5F && fabsf(mae.azimuth-270) < 1e-5F && fabsf(mae.elevation-45) < 1e-5F);
    assert(isnan(vectorMae(0, 0, 0).azimuth));
    assert(isnan(vectorMae(0, 0, 1).azimuth) && vectorMae(0, 0, 1).elevation == 90);
    OrientationReference orientation;
    const float up[3] = {0,0,1}, north[3] = {30,0,40};
    assert(orientation.change(up, north) == 0);
    const float angle = 11*RuntimePi/180;
    const float turned[3] = {30*cosf(angle),30*sinf(angle),40};
    assert(fabsf(orientation.change(up, turned)-11) < .001F);
    orientation.change(up, turned, true);
    assert(orientation.change(up, turned) < .05F);
    const float zero[3] = {0,0,0};
    assert(isnan(orientation.change(zero, turned)));

    ReportingPolicy policy;
    ReportingCommand command{};
    assert(parseReportingCommand("{\"t\":\"rm\",\"n\":1,\"q\":42,\"mode\":\"quiet\",\"duration_s\":30}", &command));
    ReportingRequests requests;
    bool changed = false;
    assert(requests.apply(command, policy, 0, &changed) == 0 && changed);
    assert(requests.apply(command, policy, 10000, &changed) == 0 && !changed);
    assert(policy.remaining(10000) == 20000);
    policy.poll(30000, false, 0);
    assert(requests.apply(command, policy, 40000, &changed) == 0 && !changed);
    assert(policy.mode == ReportingMode::Normal); // late duplicate cannot re-enter quiet
    command.durationSeconds = 31;
    assert(requests.apply(command, policy, 40000, &changed) == 2);
    assert(!parseReportingCommand("{\"t\":\"rm\",\"n\":1,\"q\":1,\"mode\":\"quiet\",\"duration_s\":-1}", &command));
    assert(!parseReportingCommand("{\"t\":\"rm\",\"n\":1,\"q\":1,\"mode\":\"quiet\",\"duration_s\":1.5}", &command));
    uint64_t utc = 0;
    assert(parseClockCommand("{\"t\":\"gt\",\"utc_ms\":1791378000123}", &utc) && utc == 1791378000123ULL);
    assert(!parseClockCommand("{\"t\":\"gt\",\"utc_ms\":-1}", &utc));
    Packet packet{};
    packet.senderId = 1; packet.flags = StatusFlag::TimeValid; packet.utcMilliseconds = 1791378000123ULL;
    packet.reportingMode = 2; packet.quietRemainingMs = 30000; packet.reportingRequest = 42;
    emitRuntimeStatus(packet, "espnow", packet.utcMilliseconds+3);
    JsonDocument doc;
    assert(!deserializeJson(doc, Serial.output));
    assert(doc["ts_ms"].as<uint64_t>() == packet.utcMilliseconds && doc["mode"] == "quiet");
    assert(doc["quiet_left_ms"] == 30000 && doc["mr"] == 42 && doc["gw_rx_ms"].as<uint64_t>() == packet.utcMilliseconds+3);
}
