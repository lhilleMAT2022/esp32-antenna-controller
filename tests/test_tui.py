import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch

from textual.command import CommandPalette
from textual.widgets import HelpPanel, RichLog, Select, TabbedContent

from antenna_controller.bridge import AntennaController
from antenna_controller.tui import AntennaControllerApp, CalibrationHelp, CommandInput, ReportingDialog
from antenna_controller.command_history import CommandEntry, load_history


class TextualTuiTests(unittest.IsolatedAsyncioTestCase):
    async def test_recall_includes_hotkeys_reporting_and_external_commands_once(self):
        controller = AntennaController('unused', command_port=0, state_port=0)
        controller.primary.connected.set()
        controller.primary.send = lambda message: True
        app = AntennaControllerApp(controller)
        try:
            async with app.run_test(size=(140, 50)) as pilot:
                app.screen.set_focus(None)
                await pilot.press('right', 'right')
                field = app.query_one('#command-input', CommandInput)
                field.focus()
                field.value = 'my draft'
                await pilot.press('up')
                self.assertEqual(field.value, 'step 1 5.0')
                await pilot.press('up')
                self.assertEqual(field.value, 'step 1 5.0')
                self.assertEqual(field.position, 0)
                await pilot.press('down', 'down')
                self.assertEqual(field.value, 'my draft')
                field.value = 'point 90'
                await pilot.press('enter')
                await pilot.press('up', 'up')
                self.assertEqual(field.value, 'step 1 5.0')  # point recorded once
                await pilot.click('#reporting-button')
                app.screen.query_one('#quiet-seconds').value = '20'
                await pilot.click('#mode-silent')
                field.reset_history_position()
                field.focus()
                await pilot.press('up')
                self.assertEqual(field.value, 'report silent 20')
                controller._send_rotator_command(2, 'stop')
                field.reset_history_position()
                await pilot.press('up')
                self.assertEqual(field.value, 'stop 2')
                controller._handle_board_message(dict(t='rp', n=1, q=1, boot=1, mode='silent', quiet_left_ms=20000), 'gateway')
                app.refresh_dashboard()
                self.assertIn('RADIO SILENT', str(app.query_one('#node-health').render()))
        finally:
            controller._server.server_close()
            controller.state_socket.close()

    async def test_command_history_csv_hotkeys_and_input_recall(self):
        controller = AntennaController('unused', command_port=0, state_port=0)
        controller.primary.connected.set()
        sent = []
        controller.primary.send = lambda message: sent.append(message) or True
        app = AntennaControllerApp(controller)
        try:
            with tempfile.TemporaryDirectory() as directory:
                controller.commands.start_csv(directory)
                async with app.run_test(size=(140, 50)) as pilot:
                    field = app.query_one('#command-input', CommandInput)
                    field.focus()
                    field.value = 'point 90'
                    await pilot.press('enter')
                    field.value = 'cal 1 status'
                    await pilot.press('enter')
                    field.value = 'unfinished draft'
                    await pilot.press('up')
                    self.assertEqual(field.value, 'cal 1 status')
                    await pilot.press('up')
                    self.assertEqual(field.value, 'point 90')
                    await pilot.press('down', 'down')
                    self.assertEqual(field.value, 'unfinished draft')
                    before = len(sent)
                    await pilot.press('left', 'right', 'a', 'd')
                    self.assertEqual(len(sent), before)  # editing never slews a node
                    app.screen.set_focus(None)
                    await pilot.press('left', 'right', 'a', 'd')
                    self.assertEqual([m['n'] for m in sent[-4:]], [1, 1, 2, 2])
                    self.assertEqual([m['d'] for m in sent[-4:]], [-5, 5, -5, 5])
                    commands = [e.command for e in controller.commands.entries()]
                    self.assertEqual(commands[:2], ['point 90', 'cal 1 status'])
                    self.assertEqual(len(commands), 6)
                    app.query_one('#health-tabs', TabbedContent).active = 'command-history-tab'
                    app.refresh_dashboard()
                    await pilot.pause()
                    log = app.query_one('#command-history-log', RichLog)
                    self.assertIn('UTC  point 90', log.lines[0].text)
                    for _ in range(40):
                        app.action_step_node(1, 5)
                    app.refresh_dashboard()
                    await pilot.pause()
                    self.assertGreater(log.max_scroll_y, 0)
                    log.scroll_home(animate=False)
                    await pilot.pause()
                    app.action_step_node(1, 5)
                    app.refresh_dashboard()
                    await pilot.pause()
                    self.assertEqual(log.scroll_y, 0)  # new commands preserve review position
                    await pilot.click('#save-command-history')
                    self.assertEqual(len(load_history(controller.commands.path)), 47)
                controller.commands.close()
        finally:
            controller.commands.close()
            controller._server.server_close()
            controller.state_socket.close()

    async def test_palette_and_key_guide_close_with_escape_or_button(self):
        controller = AntennaController('unused', command_port=0, state_port=0)
        app = AntennaControllerApp(controller)
        try:
            async with app.run_test(size=(140, 50)) as pilot:
                await pilot.press('ctrl+p')
                self.assertIsInstance(app.screen, CommandPalette)
                await pilot.press('escape')
                self.assertNotIsInstance(app.screen, CommandPalette)
                # Use the actual Ctrl+P system command callback for Keys.
                keys = next(c for c in app.get_system_commands(app.screen) if c.title == 'Keys')
                keys.callback()
                await pilot.pause()
                self.assertTrue(app.screen.query(HelpPanel))
                app.query_one('#command-input', CommandInput).focus()
                await pilot.press('escape')
                self.assertFalse(app.screen.query(HelpPanel))
                app.action_show_help_panel()
                await pilot.pause()
                await pilot.click('#close-key-help')
                self.assertFalse(app.screen.query(HelpPanel))
        finally:
            controller._server.server_close()
            controller.state_socket.close()

    async def test_csv_replay_dispatches_then_quits_without_stopping_nodes(self):
        controller = AntennaController('unused', command_port=0, state_port=0,
                                      replay_entries=[CommandEntry(None, 0, 'point 90'), CommandEntry(None, 0, 'quit')])
        controller.primary.connected.set()
        sent = []
        controller.primary.send = lambda message: sent.append(message) or True
        app = AntennaControllerApp(controller)
        try:
            async with app.run_test(size=(120, 45)) as pilot:
                await pilot.pause(0.2)
            self.assertEqual([m['c'] for m in sent], ['goto', 'goto'])
            self.assertEqual([e.command for e in controller.commands.entries()], ['point 90', 'quit'])
            self.assertEqual(app.replay.index, 2)
        finally:
            controller._server.server_close()
            controller.state_socket.close()

    async def test_relative_slew_manual_commands(self):
        controller = AntennaController('unused', command_port=0, state_port=0)
        controller.primary.connected.set()
        sent = []
        controller.primary.send = lambda message: sent.append(message) or True
        app = AntennaControllerApp(controller)
        try:
            with patch('antenna_controller.slew.time.time', return_value=1791395489.25):
                app._execute_manual_command('point +20 300 -0.5')
                self.assertEqual([m['n'] for m in sent], [1, 2])
                self.assertTrue(all(m['at'] == 1791395510 for m in sent))
                self.assertTrue(all(m['coef'] == [300, -0.5] for m in sent))
                self.assertEqual(sent[0]['q'], sent[1]['q'])
                sent.clear()
                app._execute_manual_command('goto 2 +20 0 2 dur 60 step 30')
                self.assertEqual(len(sent), 1)
                self.assertEqual({k: sent[0][k] for k in ('t', 'n', 'at', 'coef', 'dur', 'steps')},
                                 dict(t='sc', n=2, at=1791395510, coef=[0, 2], dur=60, steps=30))
        finally:
            controller._server.server_close()
            controller.state_socket.close()

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
