import math
import os
import shutil
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from antenna_controller.calibration import CalibrationManager, FACES, fit_accelerometer, fit_magnetometer
from antenna_controller.bridge import AntennaController, parse_board_line


def magnetic_capture(count=400):
    rng = np.random.default_rng(42)
    directions = rng.normal(size=(count, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    distortion = np.array([[1.3, .12, -.06], [.12, .8, .04], [-.06, .04, 1.1]])
    offset = np.array([18, -26, 9])
    raw = (directions * 48) @ distortion.T + offset + rng.normal(0, .1, (count, 3))
    return raw, directions, offset


class FittingTests(unittest.TestCase):
    def test_full_ellipsoid_recovers_bias_and_direction(self):
        raw, directions, offset = magnetic_capture()
        fit = fit_magnetometer(raw.tolist())
        np.testing.assert_allclose(fit.offset, offset, atol=.1)
        corrected = (raw - fit.offset) @ np.array(fit.matrix).reshape(3, 3).T
        corrected /= np.linalg.norm(corrected, axis=1)[:, None]
        errors = np.degrees(np.arccos(np.clip(np.sum(corrected*directions, axis=1), -1, 1)))
        self.assertLess(max(errors), .6)
        self.assertLess(fit.residual, .005)

    def test_rejects_level_circle_and_insufficient_samples(self):
        angle = np.linspace(0, 2*math.pi, 200)
        circle = np.column_stack((40*np.cos(angle), 40*np.sin(angle), np.full(200, 10)))
        with self.assertRaisesRegex(ValueError, "planar"):
            fit_magnetometer(circle.tolist())
        with self.assertRaisesRegex(ValueError, "120"):
            fit_magnetometer(circle[:30].tolist())

    def test_rejects_interference_and_nonfinite_samples(self):
        raw, _, _ = magnetic_capture()
        raw[::6] += 30
        with self.assertRaises(ValueError):
            fit_magnetometer(raw.tolist())
        raw[0, 0] = np.nan
        with self.assertRaises(ValueError):
            fit_magnetometer(raw.tolist())

    def test_six_faces_recover_bias_and_scale(self):
        bias = np.array([.03, -.02, .01])
        gain = np.array([1.02, .97, 1.04])
        faces = {}
        for j, face in enumerate(FACES):
            vector = np.zeros(3)
            vector[j//2] = 1 if face[0] == '+' else -1
            faces[face] = np.tile(vector/gain+bias, (12, 1)).tolist()
        fit = fit_accelerometer(faces)
        np.testing.assert_allclose(fit.offset, bias, atol=1e-6)
        np.testing.assert_allclose(np.diag(np.array(fit.matrix).reshape(3, 3)), gain)
        faces['+x'][0][1] += .5
        with self.assertRaisesRegex(ValueError, "Movement"):
            fit_accelerometer(faces)


def status(q=1, boot=99, request=0, flags=96, error=0):
    return {"t": "cs", "n": 2, "q": q, "boot": boot, "req": request, "cf": flags, "e": error}


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.sent, self.events = [], []
        self.manager = CalibrationManager(lambda n, m: self.sent.append(m) or "gateway", lambda n, m: self.events.append(m))
        self.manager.ingest(status())

    def test_save_waits_for_matching_node_ack_and_times_out(self):
        self.manager.command(['2', 'save'])
        request = self.sent[-1]['q']
        self.manager.ingest(status(2, request=request+1))
        self.assertIsNotNone(self.manager.nodes[2].pending)
        deadline = self.manager.nodes[2].pending[2]
        with patch('antenna_controller.calibration.time.monotonic', return_value=deadline+1):
            self.assertIn('outcome unknown', self.manager.describe(2))
        self.assertIsNone(self.manager.nodes[2].pending)

    def test_matching_ack_reports_saved_and_rejects_stale_copy(self):
        self.manager.command(['2', 'save'])
        self.manager.ingest(status(3, request=self.sent[-1]['q'], flags=111))
        self.manager.ingest(status(2, flags=96))
        self.assertIn('CALIBRATED / SAVED', self.manager.describe(2))
        self.assertIsNone(self.manager.nodes[2].pending)
        self.manager.command(['2', 'clear'])
        self.manager.ingest(status(4, request=self.sent[-1]['q'], flags=111, error=3))
        self.assertIn('rejected clear', self.manager.describe(2))

    def test_capture_deduplicates_routes_and_cancels_on_reboot(self):
        self.manager.command(['2', 'start', 'mag'])
        raw = {'t': 'rs', 'n': 2, 'q': 10, 'sf': 15, 'm': [20, 30, 40]}
        self.manager.ingest(raw)
        self.manager.ingest(raw)
        self.assertEqual(len(self.manager.nodes[2].samples), 1)
        self.manager.ingest(status(1, boot=100))
        self.assertEqual(self.manager.nodes[2].mode, '')
        self.assertEqual(self.manager.nodes[2].samples, [])
        self.manager.ingest(status(99, boot=99))
        self.assertEqual(self.manager.nodes[2].boot, 100)

    def test_face_capture_stops_after_twelve_samples(self):
        self.manager.command(['2', 'face', '+x'])
        for seq in range(20):
            self.manager.ingest({'t': 'rs', 'n': 2, 'q': seq, 'sf': 15, 'a': [1, 0, 0]})
        self.assertEqual(len(self.manager.nodes[2].faces['+x']), 12)
        self.assertEqual(self.manager.nodes[2].mode, '')

    def test_full_mag_workflow_sends_validated_fit_without_saving(self):
        self.manager.command(['2', 'start', 'mag'])
        raw, _, _ = magnetic_capture(150)
        for seq, vector in enumerate(raw):
            self.manager.ingest({'t': 'rs', 'n': 2, 'q': seq, 'sf': 15, 'm': vector.tolist()})
        self.manager.command(['2', 'fit', 'mag'])
        self.manager.command(['2', 'apply', 'mag'])
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.sent[0]['op'], 'mag')
        self.assertEqual(len(self.sent[0]['v']), 12)
        self.assertIn('NODE confirmation', self.manager.describe(2))

    def test_calibration_protocol_and_backup_routing(self):
        self.assertEqual(parse_board_line('{"t":"cc","n":2}')['t'], 'cc')
        controller = AntennaController('unused', command_port=0, state_port=0, backup_host='127.0.0.1')
        sent = []
        controller.backup.connected.set()
        controller.backup.send = lambda m: sent.append(m) or True
        try:
            controller._handle_board_message(status(), 'backup')
            controller.calibration.command(['2', 'status'])
            self.assertEqual(sent[0]['t'], 'cc')
            self.assertEqual(sent[0]['op'], 'status')
        finally:
            controller._server.server_close()
            controller.state_socket.close()


class NativeFirmwareTests(unittest.TestCase):
    def test_shared_firmware_math_storage_and_protocol(self):
        root = Path(__file__).resolve().parents[1]
        compiler = shutil.which('g++')
        if not compiler and Path('C:/msys64/mingw64/bin/g++.exe').is_file():
            compiler = 'C:/msys64/mingw64/bin/g++.exe'
        library = root/'.pio/libdeps/cyd_gateway/ArduinoJson/src'
        if not compiler or not library.is_dir():
            self.skipTest('Requires g++ and ArduinoJson installed by the PlatformIO build')
        with tempfile.TemporaryDirectory() as temp:
            binary = Path(temp)/'calibration_test.exe'
            environment = dict(os.environ)
            environment['PATH'] = str(Path(compiler).parent) + os.pathsep + environment.get('PATH', '')
            result = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-static',
                                     '-I'+str(root/'tests/native'), '-I'+str(library),
                                     str(root/'tests/native/calibration_test.cpp'), '-o', str(binary)],
                                    capture_output=True, text=True, env=environment)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
