import unittest
import tempfile
from pathlib import Path

from textual.widgets import Select

from antenna_controller.bridge import AntennaController
from antenna_controller.tui import AntennaControllerApp, CalibrationHelp, ReportingDialog


class TextualTuiTests(unittest.IsolatedAsyncioTestCase):
    async def test_calibration_help_and_node_reported_status(self):
        controller = AntennaController("unused", command_port=0, state_port=0)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        controller.calibration.capture_dir = Path(temporary.name)
        app = AntennaControllerApp(controller)
        try:
            async with app.run_test(size=(120, 45)) as pilot:
                await pilot.pause()
                self.assertGreaterEqual(app.query_one("#node-health").size.height, 10)
                await pilot.press("c")
                await pilot.pause(0.7)
                self.assertIsInstance(app.screen, CalibrationHelp)
                await pilot.press("escape")
                await pilot.pause()
                controller.calibration.ingest({"t": "cs", "n": 2, "q": 1, "boot": 1,
                                               "req": 0, "cf": 111, "e": 0})
                for node in (1, 2):
                    controller._handle_board_message({"t": "rp", "n": node, "q": 2, "boot": 1 if node == 2 else 2,
                                                      "mode": "continuous", "cf": 111 if node == 2 else 96}, 'gateway')
                app.refresh_dashboard()
                self.assertIn("CALIBRATED / SAVED", str(app.query_one("#calibration").render()))
                self.assertIn("ALL", app._execute_manual_command("cal 2 start mag"))
                self.assertEqual(controller.calibration.nodes[2].mode, "mag")
                controller.calibration.ingest({"t": "cs", "n": 1, "q": 1, "boot": 2,
                                               "req": 0, "cf": 96, "e": 0})
                self.assertIn("ALL", app._execute_manual_command("cal 1 start mag"))
                app.refresh_dashboard()
                panel = str(app.query_one("#calibration").render())
                self.assertIn("N1 SENSOR / CALIBRATION", panel)
                self.assertIn("UNCALIBRATED", panel)
                self.assertNotIn("CALIBRATED / SAVED", panel.split('N2 SENSOR')[0])
                self.assertIn("CALIBRATED / SAVED", panel)
                self.assertIn("MAE:", panel)
                self.assertEqual(controller.calibration.nodes[1].mode, "mag")
                self.assertEqual(controller.calibration.nodes[2].mode, "mag")
                await pilot.press("c")
                await pilot.pause()
                self.assertEqual(app.screen.node_id, 1)
                await pilot.press("escape")
                app.query_one("#node-select", Select).value = "node 2"
                await pilot.pause()
                app.refresh_dashboard()
                self.assertIn("N2 SENSOR / CALIBRATION", str(app.query_one("#calibration").render()))
                self.assertLessEqual(app.query_one("#calibration").parent.region.bottom,
                                     app.query_one("#command-bar").region.y)
        finally:
            controller._server.server_close()
            controller.state_socket.close()

    async def test_dashboard_mounts_and_cycles_sensor_mode(self):
        controller = AntennaController(
            "unused", command_port=0, state_port=0
        )
        app = AntennaControllerApp(controller)
        try:
            async with app.run_test(size=(120, 45)) as pilot:
                await pilot.pause()
                self.assertIsNotNone(app.query_one("#history"))
                self.assertIsNotNone(app.query_one("#node-health"))
                await pilot.press("m")
                await pilot.pause()
                self.assertEqual(app.mode, "magnetometer")
        finally:
            controller._server.server_close()
            controller.state_socket.close()

    async def test_reporting_button_and_manual_command_control_both_nodes(self):
        controller = AntennaController('unused', command_port=0, state_port=0)
        controller.primary.connected.set()
        sent = []
        controller.primary.send = lambda message: sent.append(message) or True
        app = AntennaControllerApp(controller)
        try:
            async with app.run_test(size=(120, 45)) as pilot:
                await pilot.click('#reporting-button')
                await pilot.pause()
                self.assertIsInstance(app.screen, ReportingDialog)
                await pilot.click('#mode-quiet')
                await pilot.pause()
                self.assertEqual([m['n'] for m in sent], [1, 2])
                self.assertTrue(all(m['mode'] == 'quiet' and m['duration_s'] == 300 for m in sent))
                for node in (1, 2):
                    controller._handle_board_message({'t': 'rp', 'n': node, 'q': 1, 'boot': node,
                        'cf': 96, 'mode': 'quiet', 'quiet_left_ms': 300000, 'mr': sent[0]['q'], 'me': 0}, 'gateway')
                app.refresh_dashboard()
                self.assertIn('quiet', str(app.query_one('#node-health').render()))
                self.assertIn('left', str(app.query_one('#node-health').render()))
                self.assertIn('Requested normal', app._execute_manual_command('report normal'))
                self.assertTrue(all(m['mode'] == 'normal' for m in sent[-2:]))
        finally:
            controller._server.server_close()
            controller.state_socket.close()


if __name__ == "__main__":
    unittest.main()
