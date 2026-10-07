import time
import socket
import threading
import unittest
from unittest.mock import patch

from antenna_controller.bridge import AntennaController, PiSerialRelay, parse_board_line, json_line
from antenna_controller.slew import SlewPlan, parse_bearing


class SlewTests(unittest.TestCase):
    def test_relative_start_resolves_to_absolute_utc_once(self):
        for now, expected in ((1791395489.0, 1791395509), (1791395489.75, 1791395510)):
            with self.subTest(now=now), patch('antenna_controller.slew.time.time', return_value=now) as clock:
                plan = parse_bearing('+20 300 -0.5 dur 60 step 30'.split())
                self.assertEqual(plan.start_utc_s, expected)
                self.assertEqual((plan.duration_s, plan.steps), (60, 30))
                self.assertEqual(plan.message(1, 42)['at'], expected)
                self.assertEqual(plan.message(2, 42)['at'], expected)
                clock.assert_called_once_with()
        self.assertEqual(parse_bearing(['+20']), 20)

    def test_invalid_relative_starts(self):
        for start in ('+', '+0', '+-1', '++20', '+1.5', '+nan', '+inf', '+1e2', '+99999999999'):
            with self.subTest(start=start), self.assertRaises(ValueError):
                parse_bearing([start, '300'])

    def test_relay_forwards_slew_reporting_and_clock_commands(self):
        relay = PiSerialRelay('unused', listen_host='127.0.0.1', listen_port=0)
        received = []
        ready = threading.Event()
        def send(message):
            received.append(message)
            if len(received) == 3: ready.set()
            return True
        relay.serial.send = send
        worker = threading.Thread(target=relay.server.serve_forever, daemon=True)
        worker.start()
        commands = [dict(t='sc', n=2, q=42, at=1791395489, coef=[300,-.5], dur=30, steps=3),
                    dict(t='rm', n=2, q=43, mode='normal', duration_s=0),
                    dict(t='gt', utc_ms=1791395489000)]
        try:
            with socket.create_connection(relay.server.server_address, timeout=2) as client:
                client.sendall(b''.join(json_line(command) for command in commands))
                self.assertTrue(ready.wait(2))
                self.assertEqual(received, commands)
        finally:
            relay.server.shutdown()
            relay.server.server_close()
            worker.join(2)

    def test_examples_and_defaults(self):
        self.assertEqual(parse_bearing(['300']), 300)
        plan = parse_bearing('1791395489 300 -0.5'.split())
        self.assertEqual((plan.duration_s, plan.steps), (30, 3))
        self.assertEqual([plan.bearing(k) for k in range(4)], [300, 295, 290, 285])
        plan = parse_bearing('1791395489 0 2 dur 60 step 30'.split())
        self.assertEqual([plan.bearing(k) for k in (0, 1, 30)], [0, 4, 120])
        plan = parse_bearing('1791395489 350 2 0.1 step 3 dur 30'.split())
        self.assertEqual(plan.bearing(1), 20)
        self.assertIsInstance(parse_bearing('1791395489 100'.split()), SlewPlan)

    def test_invalid_input(self):
        for command in ('', '360', 'nan', '1.5 30', '0 30', '10 nan', '10 2 dur',
                        '10 2 dur -1', '10 2 step 0', '10 2 step 3.5',
                        '10 2 dur 30 dur 60', '10 2 step 3601', '10 2 dur 1 step 11',
                        '10 0 1 2 3 4 5 6', '10 0 1e308 dur 30'):
            with self.subTest(command=command), self.assertRaises(ValueError):
                parse_bearing(command.split())

    def test_route_preflight_and_node_acknowledgments(self):
        c = AntennaController('unused', command_port=0, state_port=0)
        c.primary.connected.set()
        sent = []
        c.primary.send = lambda message: sent.append(message) or True
        plan = parse_bearing(f'{int(time.time())+60} 300 -0.5'.split())
        try:
            result = c.schedule_slew((1, 2), plan)
            self.assertIn('wait for each node', result)
            self.assertEqual([m['n'] for m in sent], [1, 2])
            self.assertEqual(sent[0]['q'], sent[1]['q'])
            request = sent[0]['q']
            c._handle_board_message(dict(t='sa', n=1, q=request, phase='accepted', e=0), 'gateway')
            self.assertNotIn((1, request), c.slew_pending)
            self.assertIn((2, request), c.slew_pending)
            c._handle_board_message(dict(t='sa', n=2, q=request, phase='rejected', e=5), 'gateway')
            self.assertFalse(c.slew_pending)
            self.assertIn('rejected', c.recent_events()[-1]['detail'])
            with self.assertRaises(ValueError):
                c.schedule_slew((1,), parse_bearing('10 200'.split()))
            sent.clear()
            with patch.object(c, '_select_route', side_effect=[('gateway', c.primary), RuntimeError('offline')]):
                with self.assertRaises(RuntimeError): c.schedule_slew((1, 2), plan)
            self.assertEqual(sent, [])
            self.assertEqual(parse_board_line('{"t":"sc","n":2}')['t'], 'sc')
            self.assertEqual(parse_board_line('{"t":"sa","n":2}')['t'], 'sa')
        finally:
            c._server.server_close()
            c.state_socket.close()
