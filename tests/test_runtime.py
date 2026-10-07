import math
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from antenna_controller.bridge import AntennaController, NodeStateStore
from antenna_controller.runtime import ReportingControl, vector_mae, vector_text, utc_text


def heartbeat(node=1, q=1, mode="normal", request=0, boot=123):
    return dict(t="rp", n=node, q=q, boot=boot, mode=mode, cf=96, h=90, mv=False,
                mr=request, me=0, quiet_left_ms=30000 if mode == 'quiet' else 0,
                ts_ms=1791378000123, tv=True, sync_age_ms=2000, sync_step_ms=4)


class RuntimeTests(unittest.TestCase):
    def test_silent_reports_and_ack_expectations(self):
        from antenna_controller.slew import SlewPlan
        controller = AntennaController('unused', command_port=0, state_port=0)
        controller.primary.connected.set()
        sent = []
        controller.primary.send = lambda message: sent.append(message) or True
        try:
            with patch('antenna_controller.bridge.time.monotonic', return_value=100):
                for node in (1, 2):
                    controller._handle_board_message(dict(heartbeat(node, mode='silent'), quiet_left_ms=300000), 'gateway')
            with patch('antenna_controller.bridge.time.monotonic', return_value=200), patch('antenna_controller.bridge.time.time', return_value=1791390000):
                with patch.object(controller, 'state_socket') as udp:
                    controller._publish_state(controller.states.snapshot()[0])
                    published = json.loads(udp.sendto.call_args.args[0])
                    self.assertEqual(published['reporting_mode'], 'silent')
                    self.assertEqual(published['quiet_remaining_s'], 200)
                self.assertTrue(controller.states.route_fresh(1, 'gateway'))
                self.assertTrue(controller.radio_ack_suppressed(1, 'gateway'))
                self.assertFalse(controller.radio_ack_suppressed(1, 'backup'))
                result = controller.schedule_slew((1, 2), SlewPlan(1791390020, (300, -.5)))
                self.assertIn('acceptance unconfirmed', result)
                self.assertFalse(controller.slew_pending)
                self.assertIn('acceptance unconfirmed', controller.calibration_command(['1', 'status']))
                self.assertIsNone(controller.calibration.nodes[1].pending)
            with patch('antenna_controller.bridge.time.monotonic', return_value=491):
                self.assertFalse(controller.states.route_fresh(1, 'gateway'))
            controller.set_reporting_mode('silent', 30)
            self.assertEqual(sent[-1]['duration_s'], 30)
        finally:
            controller._server.server_close()
            controller.state_socket.close()

    def test_mae_conventions_and_undefined_angles(self):
        magnitude, az, el = vector_mae([0, -1, 1])
        self.assertAlmostEqual(magnitude, math.sqrt(2))
        self.assertEqual((az, el), (270, 45))
        self.assertEqual(vector_mae([0, 0, 1]), (1, None, 90))
        self.assertEqual(vector_mae([0, 0, 0]), (0, None, None))
        self.assertIsNone(vector_mae([float('nan'), 0, 1]))
        self.assertIn('270.0°,45.0°', vector_text([0, -1, 1], 3, 'g'))
        self.assertIn('.123 UTC', utc_text(1791378000123))

    def test_global_modes_require_each_matching_node_ack(self):
        sent = []
        control = ReportingControl(lambda n, m: sent.append(m), lambda *_: None)
        control.command('quiet', 30)
        self.assertEqual([m['n'] for m in sent], [1, 2])
        self.assertEqual(sent[0]['q'], sent[1]['q'])
        request = sent[0]['q']
        control.ingest(heartbeat(1, mode='quiet', request=request))
        control.ingest(heartbeat(2, mode='quiet', request=request+1))
        self.assertFalse(control.confirmed())
        self.assertIn(2, control.pending)
        control.ingest(heartbeat(2, mode='quiet', request=request))
        self.assertTrue(control.confirmed())
        control.command('normal')
        control.ingest(dict(heartbeat(1, request=sent[-1]['q']), me=2))
        self.assertIn('rejected', control.results[1])

    def test_reporting_timeout_and_invalid_duration(self):
        control = ReportingControl(lambda *_: None, lambda *_: None)
        for mode, seconds in [('quiet', 0), ('quiet', -1), ('quiet', 86401), ('normal', 1), ('quiet', 1.5), ('silent', 0), ('silent', -1), ('silent', 86401)]:
            with self.subTest(mode=mode, seconds=seconds), self.assertRaises(ValueError):
                control.command(mode, seconds)
        with patch('antenna_controller.runtime.time.monotonic', return_value=1):
            control.command('normal')
        with patch('antenna_controller.runtime.time.monotonic', return_value=7):
            control.expire()
        self.assertFalse(control.confirmed())
        self.assertIn('outcome unknown', control.results[1])

    def test_mode_aware_route_freshness_and_clock_measurements(self):
        store = NodeStateStore()
        msg = dict(heartbeat(mode='quiet'), gw_rx_ms=1791378000126)
        with patch('antenna_controller.bridge.time.monotonic', return_value=100), patch('antenna_controller.bridge.time.time', return_value=1791378000.143):
            state = store.update(msg, 'gateway')
        self.assertEqual(state.node_utc_ms, 1791378000123)
        self.assertEqual(state.node_cyd_offset_ms, -3)
        self.assertAlmostEqual(state.node_pc_offset_ms, -20, delta=1)
        with patch('antenna_controller.bridge.time.monotonic', return_value=180):
            self.assertTrue(store.route_fresh(1, 'gateway'))
            self.assertFalse(store.route_fresh(1, 'backup'))
        with patch('antenna_controller.bridge.time.monotonic', return_value=191):
            self.assertFalse(store.route_fresh(1, 'gateway'))
        state = store.update(dict(msg, q=2, gw_rx_ms=1791378000128), 'gateway')
        self.assertEqual(state.cyd_jitter_ms, 1)
        store.update(heartbeat(q=3, mode='continuous'), 'gateway')
        self.assertIsNone(store.update(heartbeat(q=2, mode='quiet'), 'gateway'))
        self.assertEqual(store.route_modes[(1, 'gateway')], 'continuous')

    def test_rejected_status_does_not_refresh_route(self):
        store = NodeStateStore()
        with patch('antenna_controller.bridge.time.monotonic', return_value=100):
            store.update(heartbeat(q=3, mode='continuous'), 'gateway')
        with patch('antenna_controller.bridge.time.monotonic', return_value=110):
            self.assertIsNone(store.update(heartbeat(q=2), 'gateway'))
            self.assertIsNone(store.update(dict(t='unknown', n=1), 'gateway'))
            self.assertFalse(store.route_fresh(1, 'gateway'))

    def test_calibration_waits_for_both_continuous_acknowledgments(self):
        controller = AntennaController('unused', command_port=0, state_port=0)
        temporary = tempfile.TemporaryDirectory()
        controller.calibration.capture_dir = Path(temporary.name)
        sent = []
        controller.primary.connected.set()
        controller.primary.send = lambda msg: sent.append(msg) or True
        try:
            for node in (1, 2):
                controller._handle_board_message(heartbeat(node=node), 'gateway')
            result = controller.calibration_command(['1', 'start', 'mag'])
            self.assertIn('both nodes confirm', result)
            self.assertEqual(controller.calibration.nodes[1].mode, '')
            request = sent[0]['q']
            controller._handle_board_message(heartbeat(1, q=2, mode='continuous', request=request), 'gateway')
            self.assertEqual(controller.calibration.nodes[1].mode, '')
            controller._handle_board_message(heartbeat(2, q=2, mode='continuous', request=request), 'gateway')
            self.assertEqual(controller.calibration.nodes[1].mode, 'mag')
            self.assertIsNone(controller.pending_capture)
            with self.assertRaisesRegex(ValueError, 'Cancel active'):
                controller.set_reporting_mode('quiet', 30)
            self.assertEqual(controller.calibration.nodes[2].mode, '')
            controller.calibration_command(['1', 'cancel'])
            controller.set_reporting_mode('quiet', 30)
            with self.assertRaisesRegex(ValueError, 'pending reporting-mode'):
                controller.calibration_command(['1', 'start', 'mag'])
            self.assertEqual(controller.calibration.nodes[1].mode, '')
        finally:
            controller._server.server_close()
            controller.state_socket.close()
            temporary.cleanup()

    def test_quiet_heartbeat_keeps_calibration_flags_fresh(self):
        controller = AntennaController('unused', command_port=0, state_port=0)
        try:
            with patch('antenna_controller.calibration.time.monotonic', return_value=100):
                controller._handle_board_message(dict(heartbeat(mode='quiet'), cf=111), 'gateway')
            with patch('antenna_controller.calibration.time.monotonic', return_value=160):
                self.assertIn('CALIBRATED / SAVED', controller.calibration.describe(1))
                self.assertNotIn('STALE', controller.calibration.describe(1))
        finally:
            controller._server.server_close()
            controller.state_socket.close()
