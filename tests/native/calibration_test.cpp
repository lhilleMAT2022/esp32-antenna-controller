#include <cassert>
#include <vector>
#include "../../firmware/calibration_protocol.h"
using namespace antenna_controller;

struct MemoryStore {
    std::vector<uint8_t> blob;
    int writes = 0;
    bool fail = false, corrupt = false;
    size_t getBytesLength(const char*) { return blob.size(); }
    size_t getBytes(const char*, void* out, size_t size) {
        if (size != blob.size()) return 0;
        memcpy(out, blob.data(), size); return size;
    }
    size_t putBytes(const char*, const void* data, size_t size) {
        if (fail) return 0; // Failed atomic write preserves the previous blob.
        const auto* bytes = static_cast<const uint8_t*>(data);
        blob.assign(bytes, bytes+size); ++writes;
        if (corrupt) blob.back() ^= 1;
        return size;
    }
};

int main() {
    MemoryStore store;
    auto config = defaultCalibration();
    auto rebooted = defaultCalibration();
    assert(readCalibrationRecord(store, rebooted) == 0);
    assert(store.writes == 0);
    config.flags = 7;
    config.magOffset[0] = 12;
    assert(writeCalibrationRecord(store, config));
    assert(readCalibrationRecord(store, rebooted) == 1);
    assert(rebooted.flags == 7 && rebooted.magOffset[0] == 12);
    assert(writeCalibrationRecord(store, config) && store.writes == 1);
    store.fail = true;
    auto candidate = config;
    candidate.magOffset[0] = 22;
    assert(!writeCalibrationRecord(store, candidate));
    assert(readCalibrationRecord(store, rebooted) == 1 && rebooted.magOffset[0] == 12);
    store.fail = false;
    store.corrupt = true;
    assert(!writeCalibrationRecord(store, candidate));
    assert(readCalibrationRecord(store, rebooted) == -1);
    store.corrupt = false;
    candidate = defaultCalibration();
    assert(writeCalibrationRecord(store, candidate));
    assert(readCalibrationRecord(store, rebooted) == 1 && rebooted.flags == 0);
    candidate.version = 2;
    assert(!writeCalibrationRecord(store, candidate));
    candidate = defaultCalibration();
    candidate.magMatrix[0] = NAN;
    assert(!validCalibration(candidate));
    candidate = defaultCalibration();
    candidate.magMatrix[4] = -1;
    assert(!validCalibration(candidate));

    float heading = 0, tilt = 0;
    config = defaultCalibration();
    // With +X forward and +Z up, +Y points left. Facing east puts north at +Y.
    const float north[3] = {30, 0, 40}, east[3] = {0, 30, 40}, up[3] = {0, 0, 1};
    assert(calibratedOrientation(config, north, up, &heading, &tilt));
    assert(std::fabs(heading) < .001 && std::fabs(tilt) < .001);
    assert(calibratedOrientation(config, east, up, &heading, &tilt));
    assert(std::fabs(heading-90) < .001);
    const float tiltedUp[3] = {0, -.5F, .8660254F};
    const float tiltedMag[3] = {30, -20, 34.641016F};
    assert(calibratedOrientation(config, tiltedMag, tiltedUp, &heading, &tilt));
    assert(std::fabs(heading) < .001 && std::fabs(tilt-30) < .001);
    config.declination = -12;
    assert(calibratedOrientation(config, north, up, &heading, &tilt));
    assert(std::fabs(heading-348) < .001);
    const float invalid[3] = {0, 0, 0};
    assert(!calibratedOrientation(config, north, invalid, &heading, &tilt));

    CalibrationCommand command{};
    assert(parseCalibrationCommand("{\"t\":\"cc\",\"n\":2,\"q\":123,\"op\":\"align\",\"v\":[1,3,-12]}", &command));
    assert(command.request == 123 && command.operation == 3 && command.values[2] == -12);
    assert(!parseCalibrationCommand("{\"t\":\"cc\",\"n\":2,\"q\":123,\"op\":\"mag\",\"v\":[1,2]}", &command));
    assert(!parseCalibrationCommand("{\"t\":\"cc\",\"n\":2,\"q\":0,\"op\":\"save\",\"v\":[]}", &command));
    assert(!parseCalibrationCommand("{\"t\":\"cc\",\"n\":2,\"q\":123,\"op\":\"align\",\"v\":[1,3,1e999]}", &command));
    CalibrationReport report{};
    report.node = 2; report.request = 123; report.flags = 7|8;
    emitCalibrationJson(report, "espnow");
    JsonDocument doc;
    assert(!deserializeJson(doc, Serial.output));
    assert(doc["req"] == 123 && doc["cf"] == 15 && doc["ch"].isNull());
    return 0;
}
