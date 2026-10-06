import unittest

from antenna_controller.bridge import AntennaController
from antenna_controller.tui import AntennaControllerApp


class TextualTuiTests(unittest.IsolatedAsyncioTestCase):
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
