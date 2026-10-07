"""Check protocol-5 disciplined UTC and staggered cadence on a stationary bench.

Temporarily sets both nodes continuous, normal, then quiet for 66 seconds;
leaves both normal. No reset, rotator, calibration apply/save/clear commands.
Requires exclusive access to CYD and both node serial ports. Keep sensors still.
Raw JSON evidence is written to --output (default runtime-smoke.log).
"""
import argparse
from contextlib import ExitStack
import json
import secrets
import statistics
import time

import serial


class Bench:
    def __init__(self, ports, evidence):
        self.ports = ports
        self.evidence = evidence
        self.buffers = {name: bytearray() for name in ports}
        self.records = []
        self.next_sync = 0.0

    def send(self, message):
        self.ports['gateway'].write((json.dumps(message)+'\n').encode())

    def collect(self, seconds):
        start = time.monotonic()
        next_progress = start+15
        first = len(self.records)
        while time.monotonic()-start < seconds:
            now = time.monotonic()
            if now >= self.next_sync:
                self.send(dict(t='gt', utc_ms=int(time.time()*1000)))
                self.next_sync = now+3
            for route, port in self.ports.items():
                self.buffers[route].extend(port.read(port.in_waiting))
                buffer = self.buffers[route]
                while b'\n' in buffer:
                    line, _, rest = buffer.partition(b'\n')
                    buffer[:] = rest
                    try:
                        message = json.loads(line)
                    except (ValueError, UnicodeDecodeError):
                        continue
                    if not isinstance(message, dict):
                        continue
                    record = dict(route=route, received_monotonic=time.monotonic(),
                                  received_utc_ms=int(time.time()*1000), message=message)
                    self.records.append(record)
                    self.evidence.write(json.dumps(record)+'\n')
            if now >= next_progress:
                print(f'  observing: {now-start:.0f}/{seconds:g} s', flush=True)
                next_progress = now+15
                self.evidence.flush()
            time.sleep(.01)
        return self.records[first:]

    def mode(self, mode, seconds=0):
        request = secrets.randbelow(0xffffffff)+1
        for node in (1, 2):
            self.send(dict(t='rm', n=node, q=request, mode=mode, duration_s=seconds))
        records = self.collect(2)
        for node in (1, 2):
            assert any(r['route'] == 'gateway' and r['message'].get('n') == node
                       and r['message'].get('mr') == request and r['message'].get('me') == 0
                       and r['message'].get('mode') == mode for r in records), f'N{node} did not acknowledge {mode}'
        print(f'Both nodes acknowledged {mode} {seconds or ""}', flush=True)


def messages(records, route, node, kind):
    return [r for r in records if r['route'] == route and r['message'].get('n') == node
            and r['message'].get('t') == kind]


def cadence(records, route, node, kind, period, minimum):
    samples = messages(records, route, node, kind)
    assert len(samples) >= minimum, f'{route} N{node} {kind}: only {len(samples)} samples'
    intervals = [b['received_monotonic']-a['received_monotonic'] for a, b in zip(samples, samples[1:])]
    if intervals:
        median = statistics.median(intervals)
        assert abs(median-period) < .4, f'{route} N{node} {kind}: median interval {median:.3f}s'
        assert max(intervals) < period*1.6+.4, f'{route} N{node} {kind}: missing reports {intervals}'
    print(f'{route} N{node} {kind}: {len(samples)} reports, expected {period:g}s', flush=True)


def phase_check(records, node, period_ms):
    samples = messages(records, 'gateway', node, 'rp')
    samples = [r['message'] for r in samples if not r['message'].get('mr')]
    assert samples, f'N{node}: no periodic status to check'
    offset = (node % 10)*100 if period_ms == 1000 else (node % (period_ms//1000))*1000
    errors = [(m['ts_ms']-offset) % period_ms for m in samples]
    assert max(errors) <= 120, f'N{node}: late/off-slot reports {errors}'
    assert all(m.get('sync_state') in ('tracking','slewing') for m in samples)
    assert all(abs(m['sync_rate_ppm']) <= 500 for m in samples)
    print(f'N{node} {period_ms}ms slots: lateness {min(errors)}..{max(errors)} ms', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gateway-port', required=True)
    parser.add_argument('--node1-port', required=True)
    parser.add_argument('--node2-port', required=True)
    parser.add_argument('--output', default='runtime-smoke.log')
    args = parser.parse_args()
    with ExitStack() as stack:
        ports = {}
        for route, name in [('gateway', args.gateway_port), ('node1', args.node1_port), ('node2', args.node2_port)]:
            port = serial.Serial(baudrate=115200, timeout=0, write_timeout=2)
            port.dtr = port.rts = False
            port.port = name
            port.open()
            stack.callback(port.close)
            ports[route] = port
        evidence = stack.enter_context(open(args.output, 'w', encoding='utf-8'))
        bench = Bench(ports, evidence)
        initial = bench.collect(2)
        assert any(r['message'].get('t') == 'gs' and r['message'].get('pv') == 5 for r in initial), 'CYD protocol 5 required'
        try:
            bench.mode('continuous')
            continuous = bench.collect(7)
            for node in (1, 2):
                phase_check(continuous, node, 1000)
                for kind in ('rp', 'rs'):
                    cadence(continuous, 'gateway', node, kind, 1, 5)
                sample = messages(continuous, 'gateway', node, 'rp')[-1]
                msg = sample['message']
                assert msg.get('tv') and abs(msg['ts_ms']-sample['received_utc_ms']) < 2000, f'N{node} UTC invalid'
                assert msg.get('gw_rx_ms'), 'Gateway receive UTC absent'
                print(f'N{node} UTC offsets: CYD {msg["ts_ms"]-msg["gw_rx_ms"]} ms, PC {msg["ts_ms"]-sample["received_utc_ms"]} ms', flush=True)
            bench.mode('normal')
            normal = bench.collect(22)
            for node in (1, 2):
                phase_check(normal, node, 10000)
                for kind in ('rp', 'rs'):
                    cadence(normal, 'gateway', node, kind, 10, 2)
                    cadence(normal, f'node{node}', node, kind, 1, 19)
            bench.mode('quiet', 66)
            quiet = bench.collect(62)  # t=2..64: includes at least one UTC minute slot.
            for node in (1, 2):
                statuses = messages(quiet, 'gateway', node, 'rp')
                assert 1 <= len(statuses) <= 2 and all(r['message']['mode'] == 'quiet' for r in statuses), f'N{node}: expected minute slots'
                phase_check(quiet, node, 60000)
                if len(statuses) == 2:
                    cadence(quiet, 'gateway', node, 'rp', 60, 2)
                for kind in ('rs', 'cs'):
                    assert not messages(quiet, 'gateway', node, kind), f'N{node}: unexpected {kind}; keep sensors stationary'
                for kind in ('rp', 'rs'):
                    cadence(quiet, f'node{node}', node, kind, 1, 58)
            expired = bench.collect(5)
            for node in (1, 2):
                assert any(r['message'].get('mode') == 'normal' for r in messages(expired, 'gateway', node, 'rp')), f'N{node}: quiet did not expire'
            for node in (1, 2):
                samples = [r['message'] for r in messages(bench.records, f'node{node}', node, 'rp') if r['message'].get('tv')]
                for previous, current in zip(samples, samples[1:]):
                    elapsed = (current['up_ms']-previous['up_ms']) & 0xffffffff
                    delta = current['ts_ms']-previous['ts_ms']
                    assert abs(delta-elapsed) <= 3+elapsed*.0005, f'N{node}: unexpected clock step'
            print('PASS: staggered slots, bounded continuous clocks, quiet expiry and continuous USB', flush=True)
        finally:
            bench.mode('normal')


if __name__ == '__main__':
    main()
