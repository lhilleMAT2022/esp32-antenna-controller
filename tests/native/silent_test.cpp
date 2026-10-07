#include <cassert>
#include "../../firmware/runtime_protocol.h"
#include "../../firmware/slew_protocol.h"
using namespace antenna_controller;

int main() {
    ReportingPolicy policy;
    assert(!policy.set(ReportingMode::Silent, 0, 0));
    assert(!policy.set(ReportingMode::Silent, 86401, 0));
    assert(policy.set(ReportingMode::Silent, 10, 100));
    assert(!policy.allowRadio() && policy.allowRadio(true));
    for (uint32_t now=100; now<10100; now+=50) {
        auto reports=policy.poll(now, now%200==0, 180, 1791390000000ULL+now, now/200);
        assert(!reports.sensor && !reports.heartbeat && !reports.modeChanged);
    }
    auto reports=policy.poll(10100, true, 180);
    assert(reports.modeChanged && reports.heartbeat && reports.sensor);
    assert(policy.mode==ReportingMode::Quiet && policy.remaining(10100)==10000);
    assert(policy.allowRadio());
    assert(!policy.poll(10101,true,0).modeChanged);
    assert(policy.poll(20100,false,0).modeChanged);
    assert(policy.mode==ReportingMode::Normal);
    assert(policy.set(ReportingMode::Silent,2,UINT32_MAX-1000));
    assert(policy.remaining(998)==1);
    policy.poll(uint64_t(UINT32_MAX)+1000,false,0);
    assert(policy.mode==ReportingMode::Quiet && policy.remaining(999)==2000);
    assert(policy.set(ReportingMode::Continuous,0,1000));
    assert(policy.remaining(1000)==0 && policy.allowRadio());

    ReportingCommand command{};
    assert(parseReportingCommand("{\"t\":\"rm\",\"n\":1,\"q\":42,\"mode\":\"silent\",\"duration_s\":10}",&command));
    ReportingRequests requests;
    bool changed=false;
    assert(requests.apply(command,policy,0,&changed)==0 && changed);
    assert(requests.apply(command,policy,5000,&changed)==0 && !changed);
    assert(policy.remaining(5000)==5000);
    policy.poll(10000,false,0);
    assert(requests.apply(command,policy,12000,&changed)==0 && !changed);
    assert(policy.mode==ReportingMode::Quiet && policy.remaining(12000)==8000);
    command.durationSeconds=11;
    assert(requests.apply(command,policy,12000,&changed)==2);

    // Only the addressed node may unwrap a broadcast; wrong protocol and
    // legacy unicast payloads cannot accidentally move both antennas.
    uint8_t frame[DownlinkHeaderSize+sizeof(SlewCommand)]{};
    frame[0]=DownlinkType; frame[1]=2; frame[2]=ProtocolVersion;
    const uint8_t* data=frame; int size=sizeof(frame);
    assert(!unwrapDownlink(data,size,1));
    assert(unwrapDownlink(data,size,2) && size==sizeof(SlewCommand));
    assert(data==frame+DownlinkHeaderSize);
    frame[2]=ProtocolVersion-1; data=frame; size=sizeof(frame);
    assert(!unwrapDownlink(data,size,2));
    frame[0]=uint8_t(PacketType::Command); frame[2]=ProtocolVersion;
    assert(!unwrapDownlink(data,size,2));
    data=frame; size=1;
    assert(!unwrapDownlink(data,size,2));
}
