"""Exercise protocol-7 silent mode on non-actuating models; restore normal."""
import argparse
from contextlib import ExitStack
import json
import secrets
import time

import serial
from hardware_runtime_smoke import Bench, messages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gateway-port', required=True)
    parser.add_argument('--node1-port', required=True)
    parser.add_argument('--node2-port', required=True)
    parser.add_argument('--output', default='silent-smoke.log')
    args = parser.parse_args()
    with ExitStack() as stack:
        ports = {}
        for name, portname in [('gateway', args.gateway_port), ('node1', args.node1_port), ('node2', args.node2_port)]:
            port = serial.Serial(baudrate=115200, timeout=0, write_timeout=2)
            port.dtr = port.rts = False
            port.port = portname
            port.open()
            stack.callback(port.close)
            ports[name] = port
        bench = Bench(ports, stack.enter_context(open(args.output, 'w', encoding='utf-8')))
        initial = bench.collect(4)
        assert any(r['message'].get('pv') == 7 for r in initial if r['message'].get('t') == 'gs')
        try:
            bench.mode('continuous')
            request = secrets.randbelow(0xffffff00)+1
            start = int(time.time())+7
            for node in (1, 2):
                bench.send(dict(t='sc', n=node, q=request, at=start, coef=[120, 1], dur=4, steps=2))
            accepted = bench.collect(1)
            for node in (1, 2):
                assert any(r['message'].get('q') == request and r['message'].get('phase') == 'accepted'
                           for r in messages(accepted, 'gateway', node, 'sa'))
            bench.mode('silent', 20)
            first = len(bench.records)
            start2 = int(time.time())+12
            for node in (1, 2):
                bench.send(dict(t='sc', n=node, q=request+1, at=start2, coef=[140, 1], dur=2, steps=2))
                bench.send(dict(t='cc', n=node, q=request+2, op='status', v=[]))
            bench.collect(16)
            for node in (1, 2):
                bench.send(dict(t='rc', n=node, q=request+3, c='step', d=5))
            bench.collect(3)
            observation = bench.records[first:]
            for node in (1, 2):
                usb = messages(observation, f'node{node}', node, 'rp')
                silent = [r for r in usb if r['message'].get('mode') == 'silent']
                assert len(silent) >= 15, f'N{node}: missing serial reports during silent'
                assert len({r['message']['radio_tx'] for r in silent}) == 1, f'N{node}: radio transmit during silent'
                assert silent[-1]['message']['radio_suppressed'] > silent[0]['message']['radio_suppressed']
                for req in (request, request+1):
                    replies = messages(observation, f'node{node}', node, 'sa')
                    assert any(r['message'].get('q') == req and r['message'].get('phase') == 'targets_sent' for r in replies)
                # No node radio data after entry ACK until the first quiet report.
                radio = [r for r in observation if r['route'] == 'gateway' and r['message'].get('n') == node
                         and r['message'].get('src') == 'espnow']
                assert radio, f'N{node}: no silence-expiry report'
                earliest_quiet = next(r for r in radio if r['message'].get('mode') == 'quiet')
                assert not [r for r in radio if r['received_monotonic'] < earliest_quiet['received_monotonic'] - .05]
                assert 19500 <= earliest_quiet['message']['quiet_left_ms'] <= 20000
                assert any(r['message'].get('ack') == 2 for r in usb), f'N{node}: silent step not executed'
            print('PASS: both queued and newly sent slews ran silently; USB continued; radio TX counters unchanged', flush=True)
            remaining = bench.collect(22)
            for node in (1, 2):
                assert any(r['message'].get('mode') == 'normal' for r in messages(remaining, 'gateway', node, 'rp'))
            print('PASS: Silent 20s -> Quiet 20s -> Normal', flush=True)
            bench.mode('silent', 60)
            bench.mode('normal')
            print('PASS: explicit reporting-mode override acknowledged during silence', flush=True)
        finally:
            bench.mode('normal')
            for node in (1, 2):
                bench.send(dict(t='rc', n=node, q=1, c='stop'))
            bench.collect(2)


if __name__ == '__main__':
    main()
