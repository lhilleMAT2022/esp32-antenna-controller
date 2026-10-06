import unittest

from antenna_controller.bridge import AntennaController
from antenna_controller.tui import AntennaControllerApp, CalibrationHelp


class TextualTuiTests(unittest.IsolatedAsyncioTestCase):
    async def test_calibration_help_and_node_reported_status(self):
        controller = AntennaController("unused", command_port=0, state_port=0)
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
                app.refresh_dashboard()
                self.assertIn("CALIBRATED / SAVED", str(app.query_one("#calibration").render()))
                self.assertIn("ALL", app._execute_manual_command("cal 2 start mag"))
                self.assertEqual(controller.calibration.nodes[2].mode, "mag")
                await pilot.pause()
                self.assertLessEqual(app.query_one("#calibration").region.bottom,
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


if __name__ == "__main__":
    unittest.main()
