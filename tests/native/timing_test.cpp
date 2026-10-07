#include <cassert>
#include <cstdio>
#include <initializer_list>
#include "../../firmware/node_runtime.h"
using namespace antenna_controller;
constexpr uint64_t Epoch = 1791378000000ULL; // UTC minute boundary

void clockTests() {
    UtcClock clock;
    assert(clock.at(0) == 0 && clock.state(0) == uint8_t(ClockState::Unsynced));
    clock.synchronize(Epoch, 0);
    assert(clock.at(1234) == Epoch+1234 && clock.generation == 1);
    for (uint64_t t=3000; t<=30000; t+=3000) {
        const uint64_t before = clock.at(t);
        clock.synchronize(Epoch+t+100, t);
        assert(clock.at(t) == before); // ordinary sync never steps
        assert(fabs(clock.ratePpm(t)) <= 500);
    }
    assert(clock.ratePpm(30000) > 0 && clock.correctionMs == 0);
    uint64_t previous=clock.at(30000);
    for (uint64_t t=30001; t<=40000; ++t) {
        assert(clock.at(t) >= previous); previous=clock.at(t);
    }
    // Isolated delayed packet is rejected, not used as a clock reset.
    clock.synchronize(Epoch+41000-5000, 41000);
    assert(clock.state(41000) == uint8_t(ClockState::Checking));
    clock.synchronize(Epoch+44000+100, 44000);
    assert(clock.generation == 1 && clock.candidates == 0);
    // Three fresh consistent large errors permit recovery by step.
    for (uint64_t t=47000; t<=53000; t+=3000) clock.synchronize(Epoch+t+10000,t);
    assert(clock.generation == 2 && clock.at(53000) == Epoch+63000);
    assert(clock.correctionMs > 9000);
    clock.synchronize(Epoch+63000-20000,53000); // duplicate does not count
    assert(clock.candidates == 0);
    assert(clock.state(240000) == uint8_t(ClockState::Holdover));
    assert(clock.ratePpm(240000) == clock.driftPpm);
    assert(clock.at(uint64_t(UINT32_MAX)+10000) > Epoch+UINT32_MAX);

    // Startup delay remains an acceptable fixed bias. Filter variable delay and
    // learn a 100 ppm oscillator error with all three synchronization cadences.
    for (uint64_t period : {3000ULL,10000ULL,60000ULL}) {
        UtcClock simulated;
        simulated.synchronize(Epoch-20,0);
        for (uint64_t t=period; t<=21600000; t+=period) {
            const int delay = (t/period)%13 == 0 ? 400 : 20+int((t/period)%5)*3;
            const uint64_t source = Epoch+t+uint64_t(t*.0001)-delay;
            const uint64_t before=simulated.at(t);
            simulated.synchronize(source,t);
            assert(simulated.at(t) == before && simulated.generation == 1);
            assert(fabs(simulated.ratePpm(t)) <= 500);
        }
        const double error = double(simulated.at(21600000))-double(Epoch+21600000+2160-26);
        std::printf("sync %llu ms: final error %.2f ms drift %.2f ppm\n", (unsigned long long)period,error,simulated.driftPpm);
        assert(fabs(error)<100 && fabs(simulated.driftPpm-100)<15);
    }
    // Negative corrections also slew without making UTC run backward.
    UtcClock slow; slow.synchronize(Epoch+500,0);
    for (uint64_t t=3000; t<=30000; t+=3000) slow.synchronize(Epoch+t,t);
    assert(slow.ratePpm(30000)<0 && slow.ratePpm(30000)>=-500);
    const auto hold=slow.at(30000+UtcClock::SlewHorizonMs);
    const auto later=slow.at(30000+UtcClock::SlewHorizonMs+1000000);
    assert(fabs(double(later-hold)-(1000000+slow.driftPpm))<2);
}

void slotTests() {
    for (uint8_t node=1;node<=2;++node) {
        for (uint32_t period : {1000U,10000U,60000U}) {
            PeriodicSlot slot;
            int count=0;
            for (uint64_t t=0;t<period*3;t+=10) {
                if (slot.poll(t,Epoch+t,1,period,node)) {
                    assert((Epoch+t)%period == PeriodicSlot::phase(node,period));
                    ++count;
                    assert(!slot.poll(t,Epoch+t,1,period,node));
                }
            }
            assert(count==3);
        }
    }
    PeriodicSlot slot;
    assert(!slot.poll(0,0,0,1000,1));
    assert(slot.poll(100,100,0,1000,1)); // uptime before UTC
    assert(!slot.poll(200,Epoch+200,1,1000,1)); // initial sync: future slot only
    assert(slot.poll(1100,Epoch+1100,1,1000,1));
    assert(!slot.poll(1200,Epoch+101100,2,1000,1)); // forward step at slot
    assert(!slot.poll(1300,Epoch+1300,3,1000,1)); // backward step
    assert(slot.poll(2100,Epoch+2100,3,1000,1));
    assert(!slot.poll(9500,Epoch+9500,3,1000,1)); // missed slots skipped
    assert(slot.poll(10100,Epoch+10100,3,1000,1));

    ReportingPolicy policy(1);
    assert(!policy.poll(0,false,0,Epoch,1).heartbeat);
    auto report=policy.poll(1000,false,0,Epoch+1000,1);
    assert(report.heartbeat && report.sensor);
    report=policy.poll(1050,true,0,Epoch+1050,1); // moving now uses 1Hz sensors
    assert(!report.sensor);
    report=policy.poll(1100,true,0,Epoch+1100,1);
    assert(!report.sensor); // skip a slot only 100ms after the prior report
    report=policy.poll(2100,true,0,Epoch+2100,1);
    assert(report.sensor && !report.heartbeat);
    report=policy.poll(2200,false,0,Epoch+2200,1);
    assert(report.heartbeat && report.sensor); // completion remains immediate
    assert(policy.set(ReportingMode::Quiet,300,2300));
    policy.poll(2300,false,0,Epoch+2300,1);
    report=policy.poll(3000,false,10,Epoch+3000,1);
    assert(!report.sensor);
    report=policy.poll(3400,false,10.1F,Epoch+3400,1);
    assert(report.sensor && !report.heartbeat);
    report=policy.poll(61000,false,0,Epoch+61000,1);
    assert(report.heartbeat && !report.sensor);
    policy.poll(61500,true,0,Epoch+61500,1);
    report=policy.poll(71000,true,0,Epoch+71000,1);
    assert(report.heartbeat && report.sensor);
    report=policy.poll(72000,false,0,Epoch+72000,1);
    assert(report.heartbeat && report.sensor && policy.remaining(72000)==230300);
    report=policy.poll(302300,false,0,Epoch+302300,1);
    assert(report.modeChanged && report.heartbeat && report.sensor);
    assert(!policy.poll(302301,false,0,Epoch+302301,1).modeChanged);
    policy.set(ReportingMode::Quiet,2,UINT32_MAX-1000);
    assert(policy.remaining(998)==1);
    assert(policy.poll(uint64_t(UINT32_MAX)+1000,false,0).modeChanged);
    assert(!policy.set(ReportingMode::Quiet,0,0));
}
int main() { clockTests(); slotTests(); }
