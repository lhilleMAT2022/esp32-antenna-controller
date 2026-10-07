#pragma once
#include <stdint.h>
#include <limits.h>
#include <math.h>

namespace antenna_controller {
enum class ClockState : uint8_t { Unsynced, Tracking, Slewing, Holdover, Checking };
inline const char* clockStateName(uint8_t state) {
    switch (ClockState(state)) {
        case ClockState::Tracking: return "tracking";
        case ClockState::Slewing: return "slewing";
        case ClockState::Holdover: return "holdover";
        case ClockState::Checking: return "checking";
        default: return "unsynced";
    }
}

// One-way clock discipline. It stabilizes UTC, but does not estimate link delay.
struct UtcClock {
    static constexpr double MaxRatePpm = 500, MaxDriftPpm = 200;
    static constexpr double StepThresholdMs = 2000, CandidateToleranceMs = 250;
    static constexpr uint64_t SlewHorizonMs = 120000, HoldoverAfterMs = 180000;
    bool valid = false;
    uint64_t anchorMs = 0, lastSyncMs = 0;
    double epochMs = 0, driftPpm = 0, phasePpm = 0, filteredErrorMs = 0;
    int32_t correctionMs = 0; // Last actual step, not an incoming phase error.
    uint32_t generation = 0;
    double errors[5]{};
    uint8_t errorCount = 0, errorNext = 0, candidates = 0;
    double candidateError = 0;
    uint64_t candidateMs = 0;

    static double limit(double value, double low, double high) {
        return fmax(low, fmin(high, value));
    }
    double estimate(uint64_t monotonicMs) const {
        const double elapsed = double(int64_t(monotonicMs)-int64_t(anchorMs));
        // Finite phase correction: holdover keeps frequency, not endless slewing.
        const double phaseElapsed = limit(elapsed, 0, double(SlewHorizonMs));
        return epochMs + elapsed*(1+driftPpm/1e6) + phaseElapsed*phasePpm/1e6;
    }
    uint64_t at(uint64_t monotonicMs) const {
        return valid ? uint64_t(fmax(0, floor(estimate(monotonicMs)))) : 0;
    }
    double ratePpm(uint64_t now) const {
        return driftPpm + (now-anchorMs < SlewHorizonMs ? phasePpm : 0);
    }
    uint32_t syncAge(uint64_t now) const {
        const uint64_t age = now-lastSyncMs;
        return !valid || age > UINT32_MAX ? UINT32_MAX : uint32_t(age);
    }
    uint8_t state(uint64_t now) const {
        return uint8_t(!valid ? ClockState::Unsynced :
            syncAge(now) > HoldoverAfterMs ? ClockState::Holdover :
            candidates ? ClockState::Checking :
            now-anchorMs < SlewHorizonMs && fabs(phasePpm) > 1 ? ClockState::Slewing : ClockState::Tracking);
    }
    void step(uint64_t utcMs, uint64_t now, double error) {
        correctionMs = int32_t(limit(error, INT32_MIN, INT32_MAX));
        epochMs = double(utcMs); anchorMs = lastSyncMs = now; valid = true;
        driftPpm = phasePpm = filteredErrorMs = 0;
        candidates = errorCount = errorNext = 0;
        ++generation;
    }
    void synchronize(uint64_t utcMs, uint64_t now) {
        if (!utcMs) return;
        if (!valid) { step(utcMs, now, 0); return; }
        // Reject duplicate/backward monotonic sample times.
        if (now <= lastSyncMs || (candidates && now <= candidateMs)) return;
        const double current = estimate(now), error = double(utcMs)-current;
        if (fabs(error) > StepThresholdMs) {
            if (!candidates || now-candidateMs > HoldoverAfterMs ||
                fabs(error-candidateError) > CandidateToleranceMs) {
                candidates = 1;
            } else ++candidates;
            candidateError = error; candidateMs = now;
            if (candidates >= 3) step(utcMs, now, error);
            return;
        }
        candidates = 0;
        if (now-lastSyncMs > HoldoverAfterMs) {
            errorCount = errorNext = 0;
            filteredErrorMs = 0;
        }
        errors[errorNext] = error; errorNext = (errorNext+1)%5;
        if (errorCount < 5) ++errorCount;
        double sorted[5];
        for (uint8_t i=0; i<errorCount; ++i) {
            sorted[i] = errors[i];
            for (uint8_t j=i; j>0 && sorted[j]<sorted[j-1]; --j) {
                const double temp=sorted[j]; sorted[j]=sorted[j-1]; sorted[j-1]=temp;
            }
        }
        // Wait for three samples before steering the clock.
        const double median = sorted[errorCount/2];
        if (errorCount >= 3) filteredErrorMs += .25*(median-filteredErrorMs);
        const double dt = fmin(double(now-lastSyncMs), 60000);
        epochMs = current; anchorMs = lastSyncMs = now;
        if (errorCount < 3) { phasePpm = 0; return; }
        // Slow integral frequency estimate plus bounded phase correction.
        driftPpm = limit(driftPpm + filteredErrorMs*dt*1e6/(900000.0*900000.0), -MaxDriftPpm, MaxDriftPpm);
        const double total = limit(driftPpm + filteredErrorMs*1e6/SlewHorizonMs, -MaxRatePpm, MaxRatePpm);
        phasePpm = total-driftPpm;
    }
};
} // namespace antenna_controller
