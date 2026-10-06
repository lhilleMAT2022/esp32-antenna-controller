#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace antenna_controller {

enum CalibrationFlag : uint8_t {
    MagCalibrated = 1, AccelCalibrated = 2, MountConfigured = 4,
    CalibrationSaved = 8, CalibrationStorageInvalid = 16,
    CalibrationMagValid = 32, CalibrationAccelValid = 64,
    CalibrationHeadingValid = 128,
};
enum class CalibrationOperation : uint8_t {
    Mag = 1, Accel = 2, Align = 3, Save = 4, Clear = 5, Status = 6,
};

struct CalibrationConfig {
    uint32_t magic;
    uint16_t version;
    uint8_t flags;
    int8_t forwardAxis;
    int8_t upAxis;
    uint8_t reserved[3];
    float declination;
    float magOffset[3];
    float magMatrix[9];
    float accelOffset[3];
    float accelMatrix[9];
    uint32_t checksum;
};

inline CalibrationConfig defaultCalibration() {
    CalibrationConfig c{};
    c.magic = 0x41434331;
    c.version = 1;
    c.forwardAxis = 1;
    c.upAxis = 3;
    for (int i = 0; i < 3; ++i) {
        c.magMatrix[i*3+i] = c.accelMatrix[i*3+i] = 1.0F;
    }
    return c;
}

inline uint32_t calibrationChecksum(const CalibrationConfig& c) {
    uint32_t hash = 2166136261UL;
    const auto* bytes = reinterpret_cast<const uint8_t*>(&c);
    for (size_t i = 0; i < offsetof(CalibrationConfig, checksum); ++i) {
        hash = (hash ^ bytes[i]) * 16777619UL;
    }
    return hash;
}

inline bool validCalibration(const CalibrationConfig& c) {
    if (c.magic != 0x41434331 || c.version != 1 || (c.flags & ~7) ||
        std::abs(c.forwardAxis) < 1 || std::abs(c.forwardAxis) > 3 ||
        std::abs(c.upAxis) < 1 || std::abs(c.upAxis) > 3 ||
        std::abs(c.forwardAxis) == std::abs(c.upAxis) ||
        !std::isfinite(c.declination) || std::fabs(c.declination) > 180) return false;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(c.magOffset[i]) || std::fabs(c.magOffset[i]) > 1000 ||
            !std::isfinite(c.accelOffset[i]) || std::fabs(c.accelOffset[i]) > 0.3F) return false;
    }
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(c.magMatrix[i]) || std::fabs(c.magMatrix[i]) > 4 ||
            !std::isfinite(c.accelMatrix[i])) return false;
        if (i % 4 == 0) {
            if (c.magMatrix[i] < 0.25F || c.accelMatrix[i] < 0.7F || c.accelMatrix[i] > 1.3F) return false;
        } else if (std::fabs(c.accelMatrix[i]) > 0.000001F) return false;
    }
    const float* m = c.magMatrix;
    if (std::fabs(m[1]-m[3]) > 0.00001F || std::fabs(m[2]-m[6]) > 0.00001F ||
        std::fabs(m[5]-m[7]) > 0.00001F) return false;
    const float determinant = m[0]*(m[4]*m[8]-m[5]*m[7]) -
        m[1]*(m[3]*m[8]-m[5]*m[6]) + m[2]*(m[3]*m[7]-m[4]*m[6]);
    return m[0]*m[4]-m[1]*m[3] > 0.01F && determinant > 0.01F && determinant < 64;
}

inline bool validStoredCalibration(const CalibrationConfig& c) {
    return validCalibration(c) && c.checksum == calibrationChecksum(c);
}

// Store is an already-open Preferences namespace, or an in-memory test store.
// One NVS blob is committed atomically; no boot-time writes or partial records.
template <typename Store>
int readCalibrationRecord(Store& store, CalibrationConfig& result) {
    const size_t length = store.getBytesLength("config");
    if (!length) return 0;
    CalibrationConfig candidate{};
    if (length != sizeof(candidate) ||
        store.getBytes("config", &candidate, sizeof(candidate)) != sizeof(candidate) ||
        !validStoredCalibration(candidate)) return -1;
    result = candidate;
    return 1;
}

template <typename Store>
bool writeCalibrationRecord(Store& store, CalibrationConfig& candidate) {
    if (!validCalibration(candidate)) return false;
    candidate.checksum = calibrationChecksum(candidate);
    CalibrationConfig stored{};
    const bool same = readCalibrationRecord(store, stored) == 1 &&
        std::memcmp(&stored, &candidate, sizeof(stored)) == 0;
    if (!same && store.putBytes("config", &candidate, sizeof(candidate)) != sizeof(candidate)) return false;
    return readCalibrationRecord(store, stored) == 1 &&
        std::memcmp(&stored, &candidate, sizeof(stored)) == 0;
}

inline void correctVector(const float* raw, const float* offset,
                          const float* matrix, float* result) {
    for (int row = 0; row < 3; ++row) {
        result[row] = 0;
        for (int col = 0; col < 3; ++col)
            result[row] += matrix[row*3+col]*(raw[col]-offset[col]);
    }
}

inline float dot3(const float* a, const float* b) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

// Accelerometer points up when stationary. Heading is clockwise from magnetic
// north, then corrected by east-positive declination. Forward is the Yagi axis.
inline bool calibratedOrientation(const CalibrationConfig& c, const float* mag,
                                  const float* accel, float* heading, float* tilt) {
    const float gravity = std::sqrt(dot3(accel, accel));
    if (!std::isfinite(gravity) || gravity < 0.85F || gravity > 1.15F) return false;
    float up[3], north[3], forward[3]{};
    for (int i = 0; i < 3; ++i) up[i] = accel[i]/gravity;
    const float vertical = dot3(mag, up);
    for (int i = 0; i < 3; ++i) north[i] = mag[i]-vertical*up[i];
    const float horizontal = std::sqrt(dot3(north, north));
    if (!std::isfinite(horizontal) || horizontal < 5) return false;
    for (float& value : north) value /= horizontal;
    const float east[3] = {north[1]*up[2]-north[2]*up[1],
                          north[2]*up[0]-north[0]*up[2],
                          north[0]*up[1]-north[1]*up[0]};
    forward[std::abs(c.forwardAxis)-1] = c.forwardAxis > 0 ? 1 : -1;
    const float elevation = dot3(forward, up);
    if (std::fabs(elevation) > 0.95F) return false;
    constexpr float degrees = 57.295779513F;
    *heading = std::fmod(std::atan2(dot3(forward, east), dot3(forward, north))*degrees + c.declination + 720, 360);
    float cosine = up[std::abs(c.upAxis)-1]*(c.upAxis > 0 ? 1 : -1);
    if (cosine > 1) cosine = 1;
    if (cosine < -1) cosine = -1;
    *tilt = std::acos(cosine)*degrees;
    return true;
}

// Separate packet types extend v3 without changing its existing packet sizes.
struct __attribute__((packed)) CalibrationCommand {
    uint8_t type, node, version, operation;
    uint32_t request;
    float values[12];
};
struct __attribute__((packed)) CalibrationReport {
    uint8_t type, node, version, flags;
    uint32_t sequence, boot, request;
    uint8_t error;
    uint8_t reserved[3];
    float magnetic[3], acceleration[3], heading, tilt;
};
static_assert(sizeof(CalibrationCommand) == 56, "Calibration command layout");
static_assert(sizeof(CalibrationReport) == 52, "Calibration report layout");

} // namespace antenna_controller
