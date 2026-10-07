#pragma once
#include <stdint.h>
#include <math.h>

namespace antenna_controller {
constexpr uint8_t SlewCoefficientCount = 6, SlewQueueSize = 4;
struct __attribute__((packed)) SlewCommand {
    uint8_t type = 10, node = 0, version = 6, count = 0;
    uint32_t request = 0;
    uint64_t startMs = 0;
    uint32_t durationMs = 30000;
    uint16_t steps = 3;
    double coefficients[SlewCoefficientCount]{};
};
static_assert(sizeof(SlewCommand) == 70, "Slew command layout changed");
enum class SlewPhase : uint8_t { Queued, Started, TargetsSent, Cancelled, Rejected };
enum class SlewError : uint8_t { Ok, Invalid, Unsynced, Past, Full, Overlap, Conflict, LegacyQueue };
struct __attribute__((packed)) SlewReply {
    uint8_t type = 11, node = 0, version = 6, phase = 0;
    uint32_t request = 0;
    uint8_t error = 0;
};
static_assert(sizeof(SlewReply) == 9, "Slew reply layout changed");
inline double slewBearing(const SlewCommand& command, uint16_t step) {
    const double t = double(command.durationMs)*step/(1000.0*command.steps);
    double result = 0;
    for (int i=command.count-1;i>=0;--i) result=result*t+command.coefficients[i];
    return isfinite(result) ? fmod(fmod(result,360.0)+360.0,360.0) : NAN;
}
inline bool validSlew(const SlewCommand& command) {
    if (command.type!=10 || command.version!=6 || command.node<1 || command.node>2 ||
        !command.request || !command.startMs || command.startMs>4102444800000ULL ||
        command.count<1 || command.count>SlewCoefficientCount || !command.durationMs ||
        command.durationMs>86400000 || !command.steps || command.steps>3600 ||
        command.durationMs<uint32_t(command.steps)*100) return false;
    for (uint8_t i=0;i<command.count;++i) if (!isfinite(command.coefficients[i])) return false;
    for (uint16_t i=0;i<=command.steps;++i) if (!isfinite(slewBearing(command,i))) return false;
    return true;
}
inline bool sameSlew(const SlewCommand& a, const SlewCommand& b) {
    if (a.startMs!=b.startMs || a.durationMs!=b.durationMs || a.steps!=b.steps || a.count!=b.count) return false;
    for (uint8_t i=0;i<a.count;++i) if (a.coefficients[i]!=b.coefficients[i]) return false;
    return true;
}
struct SlewQueue {
    struct Job { SlewCommand command; bool occupied=false, started=false; uint16_t next=0; };
    Job jobs[SlewQueueSize];
    SlewCommand recent[8]{};
    uint8_t recentNext=0;
    bool pending() const { for (const auto& j:jobs) if (j.occupied) return true; return false; }
    bool running() const { for (const auto& j:jobs) if (j.occupied && j.started) return true; return false; }
    SlewError add(const SlewCommand& command, uint64_t utc, bool legacyPending=false) {
        if (!validSlew(command)) return SlewError::Invalid;
        // Duplicate requests never restart a completed or cancelled plan.
        for (const auto& old:recent) if (old.request==command.request)
            return sameSlew(old,command) ? SlewError::Ok : SlewError::Conflict;
        if (!utc) return SlewError::Unsynced;
        if (command.startMs<=utc) return SlewError::Past;
        if (legacyPending) return SlewError::LegacyQueue;
        Job* free=nullptr;
        for (auto& job:jobs) {
            if (!job.occupied) { if (!free) free=&job; continue; }
            if (command.startMs<=job.command.startMs+job.command.durationMs &&
                job.command.startMs<=command.startMs+command.durationMs) return SlewError::Overlap;
        }
        if (!free) return SlewError::Full;
        free->command=command; free->occupied=true; free->started=false; free->next=0;
        recent[recentNext]=command; recentNext=(recentNext+1)%8;
        return SlewError::Ok;
    }
    template<class Notify> void cancel(Notify notify) {
        for (auto& job:jobs) if (job.occupied) {
            job.occupied=false; notify(job.command.request,SlewPhase::Cancelled);
        }
    }
    template<class Target,class Notify> void poll(uint64_t utc, Target target, Notify notify) {
        if (!utc) return;
        // Earliest plan first, even when commands were submitted out of order.
        for (uint8_t pass=0;pass<SlewQueueSize;++pass) {
            Job* selected=nullptr;
            for (auto& job:jobs) if (job.occupied && job.command.startMs<=utc &&
                (!selected || job.command.startMs<selected->command.startMs)) selected=&job;
            if (!selected) break;
            auto& job=*selected;
            const auto& c=job.command;
            if (!job.started) { job.started=true; notify(c.request,SlewPhase::Started); }
            const uint64_t elapsed=utc-c.startMs;
            const uint16_t due=elapsed>=c.durationMs ? c.steps : uint16_t(elapsed*c.steps/c.durationMs);
            // Forward jumps skip intermediate updates; backward steps never replay.
            if (due>=job.next) { target(float(slewBearing(c,due))); job.next=due+1; }
            if (due==c.steps) { job.occupied=false; notify(c.request,SlewPhase::TargetsSent); }
            else break;
        }
    }
};
} // namespace antenna_controller
