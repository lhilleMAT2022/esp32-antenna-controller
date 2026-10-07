#pragma once
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include "clock_discipline.h"

namespace antenna_controller {
enum class ReportingMode : uint8_t { Continuous = 0, Normal = 1, Quiet = 2, Silent = 3 };
constexpr uint32_t MaxQuietSeconds = 86400;
constexpr float QuietTurnDegrees = 10.0F;
constexpr float RuntimePi = 3.14159265358979323846F;

inline const char* reportingName(ReportingMode mode) {
    return mode == ReportingMode::Continuous ? "continuous" :
           mode == ReportingMode::Silent ? "silent" :
           mode == ReportingMode::Quiet ? "quiet" : "normal";
}
inline uint32_t heartbeatPeriod(ReportingMode mode, bool moving = false) {
    return mode == ReportingMode::Continuous ? 1000 :
           mode == ReportingMode::Quiet ? (moving ? 10000 : 60000) :
           10000;
}
inline uint32_t linkTimeout(ReportingMode mode, uint32_t remainingMs = 0) {
    if (mode == ReportingMode::Silent) return remainingMs + 90000;
    return mode == ReportingMode::Continuous ? 5000 :
           mode == ReportingMode::Quiet ? 90000 : 25000;
}

struct VectorMae { float magnitude, azimuth, elevation; };
inline VectorMae vectorMae(float x, float y, float z) {
    const float horizontal = hypotf(x, y), magnitude = hypotf(horizontal, z);
    if (!isfinite(magnitude) || magnitude < 1e-9F) return {magnitude, NAN, NAN};
    float azimuth = horizontal < 1e-9F ? NAN : atan2f(y, x) * 180.0F / RuntimePi;
    if (azimuth < 0) azimuth += 360.0F;
    return {magnitude, azimuth, atan2f(z, horizontal) * 180.0F / RuntimePi};
}

// A gravity/magnetic triad measures full 3-D rotation, including heading wrap.
struct OrientationReference {
    float reference[9]{};
    bool valid = false;
    static bool basis(const float* acceleration, const float* magnetic, float* out) {
        const float g = sqrtf(acceleration[0]*acceleration[0] + acceleration[1]*acceleration[1] + acceleration[2]*acceleration[2]);
        if (!isfinite(g) || g < .8F || g > 1.2F) return false;
        for (int i = 0; i < 3; ++i) out[6+i] = acceleration[i]/g;
        float projection = 0;
        for (int i = 0; i < 3; ++i) projection += magnetic[i]*out[6+i];
        float horizontal = 0;
        for (int i = 0; i < 3; ++i) { out[i] = magnetic[i]-projection*out[6+i]; horizontal += out[i]*out[i]; }
        horizontal = sqrtf(horizontal);
        if (!isfinite(horizontal) || horizontal < 1.0F) return false;
        for (int i = 0; i < 3; ++i) out[i] /= horizontal;
        out[3] = out[7]*out[2]-out[8]*out[1];
        out[4] = out[8]*out[0]-out[6]*out[2];
        out[5] = out[6]*out[1]-out[7]*out[0];
        return true;
    }
    float change(const float* acceleration, const float* magnetic, bool accept = false) {
        float current[9];
        if (!basis(acceleration, magnetic, current)) return NAN;
        float trace = 0;
        for (int i = 0; i < 9; ++i) trace += current[i]*reference[i];
        const float cosine = fmaxf(-1, fminf(1, (trace-1)*.5F));
        const float angle = valid ? acosf(cosine)*180.0F/RuntimePi : 0;
        if (!valid || accept) { for (int i = 0; i < 9; ++i) reference[i] = current[i]; valid = true; }
        return angle;
    }
};

// Absolute slot scheduling: no replay of missed slots, no mode-relative phase.
struct PeriodicSlot {
    uint64_t nextMs = 0, previousTime = 0, lastSentMono = 0;
    uint32_t periodMs = 0, generation = 0;
    bool initialized = false, sent = false;
    static uint32_t phase(uint8_t node, uint32_t period) {
        return period == 1000 ? (node%10)*100 : (node%(period/1000))*1000;
    }
    static uint64_t next(uint64_t time, uint32_t period, uint32_t offset) {
        const uint64_t candidate = time-time%period+offset;
        return candidate > time ? candidate : candidate+period;
    }
    bool poll(uint64_t mono, uint64_t time, uint32_t clockGeneration,
              uint32_t period, uint8_t node) {
        const uint32_t offset = phase(node, period);
        // A step or uptime-to-UTC transition establishes a new future slot.
        const bool reset = !initialized || periodMs != period || generation != clockGeneration || time < previousTime;
        const bool due = !reset && time >= nextMs;
        const bool onTime = due && time-nextMs <= 100;
        const bool spaced = !sent || mono-lastSentMono >= (period == 1000 ? 500 : 1000);
        if (reset || due) nextMs = next(time, period, offset);
        periodMs = period; generation = clockGeneration; initialized = true;
        previousTime = time;
        if (onTime && spaced) { lastSentMono = mono; sent = true; return true; }
        return false;
    }
};

struct RadioReports { bool heartbeat = false, sensor = false, modeChanged = false; };
struct ReportingPolicy {
    ReportingMode mode = ReportingMode::Normal;
    uint8_t nodeId = 1;
    uint32_t quietStartedMs = 0, quietDurationMs = 0, lastSensorMs = 0;
    bool wasMoving = false;
    PeriodicSlot heartbeatSlot, sensorSlot;
    explicit ReportingPolicy(uint8_t node = 1) : nodeId(node) {}
    uint32_t remaining(uint32_t now) const {
        const uint32_t elapsed = now-quietStartedMs;
        return (mode == ReportingMode::Quiet || mode == ReportingMode::Silent) && elapsed < quietDurationMs ? quietDurationMs-elapsed : 0;
    }
    bool allowRadio(bool reportingReply = false) const {
        return mode != ReportingMode::Silent || reportingReply;
    }
    bool set(ReportingMode next, uint32_t seconds, uint32_t now) {
        if (uint8_t(next) > 3 || ((next == ReportingMode::Quiet || next == ReportingMode::Silent) ? (!seconds || seconds > MaxQuietSeconds) : seconds != 0)) return false;
        mode = next; quietStartedMs = now; quietDurationMs = seconds*1000;
        lastSensorMs = now;
        heartbeatSlot.initialized = sensorSlot.initialized = false;
        return true;
    }
    RadioReports poll(uint64_t mono, bool moving, float uncommandedTurn,
                      uint64_t utc = 0, uint32_t clockGeneration = 0) {
        const uint32_t now = uint32_t(mono);
        const uint64_t time = utc ? utc : mono;
        const uint32_t generation = utc ? clockGeneration : 0;
        RadioReports result;
        if (mode == ReportingMode::Silent) {
            wasMoving = moving;
            if (remaining(now)) return result;
            // Start a full quiet interval when silence expires. No deferred
            // command replies or slew lifecycle messages are replayed.
            mode = ReportingMode::Quiet; quietStartedMs = now;
            heartbeatSlot.initialized = sensorSlot.initialized = false;
            result.modeChanged = result.heartbeat = result.sensor = true;
        }
        if (mode == ReportingMode::Quiet && remaining(now) == 0) {
            mode = ReportingMode::Normal; quietDurationMs = 0;
            result.modeChanged = result.heartbeat = result.sensor = true;
        }
        const bool completed = wasMoving && !moving;
        wasMoving = moving;
        const bool heartbeatDue = heartbeatSlot.poll(mono, time, generation, heartbeatPeriod(mode, moving), nodeId);
        result.heartbeat |= completed || heartbeatDue;
        if (mode != ReportingMode::Quiet || moving) {
            const uint32_t period = mode == ReportingMode::Normal && moving ? 1000 : heartbeatPeriod(mode, moving);
            const bool sensorDue = sensorSlot.poll(mono, time, generation, period, nodeId);
            result.sensor |= completed || sensorDue;
        } else {
            sensorSlot.initialized = false;
            result.sensor |= completed || (isfinite(uncommandedTurn) && uncommandedTurn > QuietTurnDegrees && now-lastSensorMs >= 1000);
        }
        if (result.sensor) lastSensorMs = now;
        return result;
    }
};
}  // namespace antenna_controller
