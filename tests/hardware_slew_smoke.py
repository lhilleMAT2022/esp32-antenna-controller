"""Exercise scheduled targets on the NON-ACTUATING ESP32 model firmware.

Sends motion-model targets, duplicate/conflicting schedules and stop commands.
Leaves models stopped and reporting normal. Never applies or saves calibration.
"""
import argparse
from contextlib import ExitStack
import json
import secrets
import time

import serial
from hardware_runtime_smoke import Bench, messages


def reply(records, node, request, phase, error=0, route='gateway'):
    return [r for r in records if r['route']==route and r['message'].get('t')=='sa'
            and r['message'].get('n')==node and r['message'].get('q')==request
            and r['message'].get('phase')==phase and r['message'].get('e')==error]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gateway-port',required=True)
    parser.add_argument('--node1-port',required=True)
    parser.add_argument('--node2-port',required=True)
    parser.add_argument('--output',default='slew-smoke.log')
    args=parser.parse_args()
    with ExitStack() as stack:
        ports={}
        for name,portname in [('gateway',args.gateway_port),('node1',args.node1_port),('node2',args.node2_port)]:
            port=serial.Serial(baudrate=115200,timeout=0,write_timeout=2)
            port.dtr=port.rts=False; port.port=portname; port.open()
            stack.callback(port.close); ports[name]=port
        bench=Bench(ports,stack.enter_context(open(args.output,'w',encoding='utf-8')))
        initial=bench.collect(3)
        assert any(r['message'].get('t')=='gs' and r['message'].get('pv')==6 for r in initial)
        try:
            bench.mode('continuous')
            request=secrets.randbelow(0xffffff00)+1
            start=int(time.time())+7
            commands=[dict(t='sc',n=n,q=request,at=start,coef=[300,-.5],dur=6,steps=3) for n in (1,2)]
            first=len(bench.records)
            for command in commands: bench.send(command)
            accepted=bench.collect(1)
            for node in (1,2): assert reply(accepted,node,request,'accepted'), f'N{node}: no acceptance'
            for command in commands: bench.send(command)
            duplicate=bench.collect(1)
            for node in (1,2): assert reply(duplicate,node,request,'accepted'), f'N{node}: duplicate not acknowledged'
            for command in commands: bench.send(dict(command,q=request+1,at=start+1))
            rejected=bench.collect(1)
            for node in (1,2): assert reply(rejected,node,request+1,'rejected',5), f'N{node}: overlap accepted'
            print('Both nodes accepted plan, deduplicated retransmission and rejected overlap',flush=True)
            bench.collect(22)
            records=bench.records[first:]
            for node in (1,2):
                starts=reply(records,node,request,'started')
                ends=reply(records,node,request,'targets_sent')
                assert len(starts)==len(ends)==1, f'N{node}: lifecycle duplicates/missing'
                assert abs(starts[0]['received_utc_ms']-start*1000)<1000, f'N{node}: start late'
                assert abs(ends[0]['received_utc_ms']-(start+6)*1000)<1000, f'N{node}: endpoint late'
                status=messages(records,f'node{node}',node,'rp')
                targets={r['message'].get('tg') for r in status}
                assert {300,299,298}.issubset(targets), f'N{node}: missing intermediate targets {targets}'
                last=status[-1]['message']
                assert abs(last['h']-297)<1 and not last['mv'], f'N{node}: simulator did not settle {last}'
                print(f'N{node}: timed targets 300,299,298,297; model settled at {last["h"]}',flush=True)
            # Direct USB uses the same plan and acknowledges over both routes.
            future=dict(commands[1],q=request+2,at=int(time.time())+30)
            ports['node2'].write((json.dumps(future)+'\n').encode())
            accepted=bench.collect(1)
            assert reply(accepted,2,request+2,'accepted',route='node2')
            bench.send(dict(t='rc',n=2,q=request+3,c='stop'))
            cancelled=bench.collect(1)
            assert reply(cancelled,2,request+2,'cancelled')
            print('PASS: UTC slew targets, node lifecycle ACKs, overlap/replay checks, serial route and stop cancellation',flush=True)
        finally:
            for node in (1,2): bench.send(dict(t='rc',n=node,q=1,c='stop'))
            bench.collect(1)
            bench.mode('normal')


if __name__=='__main__': main()
