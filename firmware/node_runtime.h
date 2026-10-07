#pragma once
#include <stdint.h>
#include <math.h>
#include <limits.h>

namespace antenna_controller {
enum class ReportingMode : uint8_t { Continuous = 0, Normal = 1, Quiet = 2 };
constexpr uint32_t MaxQuietSeconds = 86400;
constexpr float QuietTurnDegrees = 10.0F;
constexpr float RuntimePi = 3.14159265358979323846F;

inline const char* reportingName(ReportingMode mode) {
    return mode == ReportingMode::Continuous ? "continuous" :
           mode == ReportingMode::Quiet ? "quiet" : "normal";
}
inline uint32_t heartbeatPeriod(ReportingMode mode, bool moving = false) {
    return mode == ReportingMode::Continuous ? 1000 :
           mode == ReportingMode::Quiet ? (moving ? 10000 : 60000) :
           10000;
}
inline uint32_t linkTimeout(ReportingMode mode) {
    return mode == ReportingMode::Continuous ? 5000 :
           mode == ReportingMode::Quiet ? 90000 : 25000;
}

struct UtcClock {
    bool valid = false;
    uint64_t epochMs = 0, anchorMs = 0;
    int32_t correctionMs = 0;
    uint64_t at(uint64_t monotonicMs) const {
        return valid ? uint64_t(int64_t(epochMs) + int64_t(monotonicMs) - int64_t(anchorMs)) : 0;
    }
    void synchronize(uint64_t utcMs, uint64_t monotonicMs) {
        if (!utcMs) return;
        const int64_t correction = valid ? int64_t(utcMs) - int64_t(at(monotonicMs)) : 0;
        correctionMs = correction > INT32_MAX ? INT32_MAX :
                       correction < INT32_MIN ? INT32_MIN : int32_t(correction);
        epochMs = utcMs; anchorMs = monotonicMs; valid = true;
    }
    uint32_t syncAge(uint64_t monotonicMs) const {
        const uint64_t age = monotonicMs - anchorMs;
        return !valid || age > UINT32_MAX ? UINT32_MAX : uint32_t(age);
    }
};

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

struct RadioReports { bool heartbeat = false, sensor = false, modeChanged = false; };
struct ReportingPolicy {
    ReportingMode mode = ReportingMode::Normal;
    uint32_t quietStartedMs = 0, quietDurationMs = 0;
    uint32_t lastHeartbeatMs = 0, lastSensorMs = 0;
    bool heartbeatSent = false, sensorSent = false, wasMoving = false;
    uint32_t remaining(uint32_t now) const {
        const uint32_t elapsed = now-quietStartedMs;
        return mode == ReportingMode::Quiet && elapsed < quietDurationMs ? quietDurationMs-elapsed : 0;
    }
    bool set(ReportingMode next, uint32_t seconds, uint32_t now) {
        if (uint8_t(next) > 2 || (next == ReportingMode::Quiet ? (!seconds || seconds > MaxQuietSeconds) : seconds != 0)) return false;
        mode = next; quietStartedMs = now; quietDurationMs = seconds*1000;
        // The command acknowledgment is the initial heartbeat for this mode.
        lastHeartbeatMs = lastSensorMs = now; heartbeatSent = sensorSent = true;
        return true;
    }
    RadioReports poll(uint32_t now, bool moving, float uncommandedTurn) {
        RadioReports result;
        if (mode == ReportingMode::Quiet && remaining(now) == 0) {
            mode = ReportingMode::Normal; quietDurationMs = 0;
            result.modeChanged = result.heartbeat = result.sensor = true;
        }
        const bool completed = wasMoving && !moving;
        wasMoving = moving;
        result.heartbeat |= completed || !heartbeatSent || now-lastHeartbeatMs >= heartbeatPeriod(mode, moving);
        if (mode != ReportingMode::Quiet || moving) {
            const uint32_t sensorPeriod = mode == ReportingMode::Normal && moving ? 1000 : heartbeatPeriod(mode, moving);
            result.sensor |= completed || !sensorSent || now-lastSensorMs >= sensorPeriod;
        } else {
            result.sensor |= completed || (isfinite(uncommandedTurn) && uncommandedTurn > QuietTurnDegrees && now-lastSensorMs >= 1000);
        }
        if (result.heartbeat) { lastHeartbeatMs = now; heartbeatSent = true; }
        if (result.sensor) { lastSensorMs = now; sensorSent = true; }
        return result;
    }
};
}  // namespace antenna_controller
