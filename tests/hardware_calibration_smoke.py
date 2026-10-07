"""Verify calibration replies and invalid-fit rejection; never save or actuate."""

import argparse
import json
import secrets
import time

import serial


def receive(connection, predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        raw = connection.readline()
        try:
            message = json.loads(raw)
        except (ValueError, UnicodeDecodeError):
            continue
        if isinstance(message, dict) and predicate(message):
            return message
    raise RuntimeError(f"Timed out waiting for calibration response on {connection.port}")


def check_route(port, node):
    with serial.Serial(port, 115200, timeout=.5) as connection:
        initial = receive(connection, lambda m: m.get('t') == 'cs' and m.get('n') == node)
        for operation, values, expected_error in [('status', [], 0), ('mag', [0.0]*12, 2)]:
            request = secrets.randbelow(0xffffffff)+1
            command = {'t': 'cc', 'n': node, 'q': request, 'op': operation, 'v': values}
            connection.write((json.dumps(command, separators=(',', ':'))+'\n').encode())
            response = receive(connection, lambda m: m.get('t') == 'cs' and m.get('n') == node and m.get('req') == request)
            if response.get('e') != expected_error:
                raise RuntimeError(f"Unexpected {operation} response: {response}")
            if response.get('cf', 0) & 31 != initial.get('cf', 0) & 31:
                raise RuntimeError('Calibration state changed during read-only/rejection check')
            print(port, operation, json.dumps(response, separators=(',', ':')))
        print(f'PASS {port}: node status and invalid matrix rejection; calibration unchanged')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node-port', required=True)
    parser.add_argument('--gateway-port', required=True)
    parser.add_argument('--node', type=int, choices=(1, 2), default=2)
    args = parser.parse_args()
    check_route(args.node_port, args.node)
    check_route(args.gateway_port, args.node)


if __name__ == '__main__':
    main()
