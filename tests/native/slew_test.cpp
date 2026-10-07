#include <cassert>
#include <vector>
#include "../../firmware/slew_protocol.h"
using namespace antenna_controller;
int main() {
    SlewCommand c;
    assert(parseSlewCommand("{\"t\":\"sc\",\"n\":1,\"q\":42,\"at\":1791395489,\"coef\":[300,-0.5],\"dur\":30,\"steps\":3}",&c));
    assert(c.startMs==1791395489000ULL && c.count==2);
    assert(slewBearing(c,0)==300 && slewBearing(c,1)==295 && slewBearing(c,3)==285);
    SlewQueue queue;
    assert(queue.add(c,0)==SlewError::Unsynced);
    assert(queue.add(c,c.startMs)==SlewError::Past);
    assert(queue.add(c,c.startMs-1000)==SlewError::Ok);
    assert(queue.add(c,c.startMs-500)==SlewError::Ok); // idempotent
    auto conflict=c; conflict.coefficients[0]=200;
    assert(queue.add(conflict,c.startMs-100)==SlewError::Conflict);
    conflict=c; conflict.request=43; conflict.startMs+=1000;
    assert(queue.add(conflict,c.startMs-100)==SlewError::Overlap);
    std::vector<float> targets;
    std::vector<SlewPhase> phases;
    const auto target=[&](float bearing){ targets.push_back(bearing); };
    const auto notify=[&](uint32_t request,SlewPhase phase){ assert(request==42); phases.push_back(phase); };
    queue.poll(c.startMs-1,target,notify); assert(targets.empty());
    queue.poll(c.startMs,target,notify); assert(targets.back()==300 && queue.running());
    queue.poll(c.startMs+9999,target,notify); assert(targets.size()==1);
    queue.poll(c.startMs+10000,target,notify); assert(targets.back()==295);
    queue.poll(c.startMs+5000,target,notify); assert(targets.size()==2); // backward time: no replay
    queue.poll(c.startMs+30000,target,notify); assert(targets.back()==285 && targets.size()==3); // skip missed middle
    assert(phases.size()==2 && phases.back()==SlewPhase::TargetsSent && !queue.pending());
    assert(queue.add(c,c.startMs+50000)==SlewError::Ok && !queue.pending()); // late duplicate stays retired
    c.request=44; c.startMs+=100000; c.count=3;
    c.coefficients[0]=350; c.coefficients[1]=2; c.coefficients[2]=.1;
    assert(slewBearing(c,1)==20); // 350+20+10 wraps
    assert(queue.add(c,c.startMs-1000)==SlewError::Ok);
    queue.cancel([](uint32_t q,SlewPhase phase){ assert(q==44 && phase==SlewPhase::Cancelled); });
    assert(!queue.pending());
    assert(queue.add(c,c.startMs-500)==SlewError::Ok && !queue.pending());
    c.request=45; c.coefficients[0]=NAN;
    assert(!validSlew(c)); c.coefficients[0]=0; c.steps=0; assert(!validSlew(c));
    c.steps=3600; c.durationMs=1000; assert(!validSlew(c));
    SlewQueue full;
    c.steps=3; c.durationMs=30000;
    for (uint32_t i=0;i<4;++i) {
        c.request=100+i; c.startMs=1000000+i*40000;
        assert(full.add(c,1)==SlewError::Ok);
    }
    c.request=104; c.startMs=1200000;
    assert(full.add(c,1)==SlewError::Full);
    assert(!parseSlewCommand("{\"t\":\"sc\",\"n\":1,\"q\":42,\"at\":10,\"coef\":[0,2],\"dur\":-1,\"steps\":3}",&c));
    SlewReply reply; reply.node=1; reply.request=42; reply.phase=uint8_t(SlewPhase::Started);
    emitSlewReply(reply,"node_serial");
    JsonDocument doc; assert(!deserializeJson(doc,Serial.output));
    assert(doc["t"]=="sa" && doc["phase"]=="started" && doc["q"]==42);
}
