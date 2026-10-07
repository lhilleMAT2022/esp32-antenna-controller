"""Exercise CYD link expiry/recovery by holding one ESP32 in reset via RTS.

This restarts the specified node and discards its unsaved RAM settings. Use
only on an idle bench node with the standard ESP32 USB auto-reset circuit.
No calibration settings are written and no rotator commands are sent.
"""

import argparse
import json
import time

import serial


def open_port(port):
    connection = serial.Serial(baudrate=115200, timeout=.2, write_timeout=2)
    connection.dtr = False
    connection.rts = False
    connection.port = port
    connection.open()
    return connection


def wait_line(connection, predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = connection.readline().decode('utf-8', errors='replace').strip()
        if predicate(line):
            print(line, flush=True)
            return line
    raise RuntimeError(f'Timed out on {connection.port}')


def sensor_line(line, node, require_valid=True):
    try:
        message = json.loads(line)
    except ValueError:
        return False
    return (isinstance(message, dict) and message.get('t') == 'rs'
            and message.get('n') == node
            and (not require_valid or message.get('sf', 0) & 12 == 12))


def check_status(connection, expected):
    connection.write(b'STATUS\n')
    pending = set(expected)

    def match(line):
        fields = line.split()
        if len(fields) >= 2:
            pending.discard(' '.join(fields[:2]))
        return not pending

    wait_line(connection, match, timeout=4)
    print('Confirmed ' + ', '.join(expected), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gateway-port', required=True)
    parser.add_argument('--reset-node-port', required=True)
    parser.add_argument('--node', type=int, choices=(1, 2), required=True)
    args = parser.parse_args()
    node, other = args.node, 3 - args.node
    with open_port(args.gateway_port) as gateway:
        gateway.write(b'PANEL HOME\n')
        wait_line(gateway, lambda line: sensor_line(line, node))
        wait_line(gateway, lambda line: sensor_line(line, other, require_valid=False))
        check_status(gateway, [f'N{node} ONLINE', f'N{other} ONLINE'])
        with open_port(args.reset_node_port) as target:
            try:
                gateway.reset_input_buffer()
                started = time.monotonic()
                target.rts = True
                wait_line(gateway, lambda line: line == f'LINK N{node} OFFLINE', timeout=9)
                print(f'OFFLINE after {time.monotonic() - started:.2f} s in reset', flush=True)
                check_status(gateway, [f'N{node} OFFLINE', f'N{other} ONLINE'])
            finally:
                target.rts = False
            wait_line(gateway, lambda line: line == f'LINK N{node} ONLINE')
            wait_line(gateway, lambda line: sensor_line(line, node))
            check_status(gateway, [f'N{node} ONLINE', f'N{other} ONLINE'])
        print('PASS: link loss, timeout redraw event, unaffected peer, recovery and valid target sensors', flush=True)


if __name__ == '__main__':
    main()
