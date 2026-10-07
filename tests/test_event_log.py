import csv
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from textual.widgets import RichLog, Select

from antenna_controller.bridge import AntennaController
from antenna_controller.command_history import CSV_COLUMNS, CommandEntry, load_history, replay_command, validate_command
from antenna_controller.event_log import EVENT_HISTORY_LIMIT, parse_dump_path
from antenna_controller.tui import AntennaControllerApp, CommandInput


class EventLogTests(unittest.IsolatedAsyncioTestCase):
    def controller(self):
        controller = AntennaController('unused', command_port=0, state_port=0)
        self.addCleanup(controller._server.server_close)
        self.addCleanup(controller.state_socket.close)
        return controller

    def test_event_retention_and_monotonic_elapsed(self):
        controller = self.controller()
        for index in range(EVENT_HISTORY_LIMIT + 3):
            with patch.object(controller.commands, 'elapsed', return_value=index * 0.1):
                controller._record_event('SYSTEM', str(index), source='test', route='local')
        entries = controller.recent_events()
        self.assertEqual(len(entries), 10_000)
        self.assertEqual(entries[0]['detail'], '3')
        self.assertAlmostEqual(entries[0]['elapsed_seconds'], 0.3)
        self.assertEqual(entries[-1]['sequence'], EVENT_HISTORY_LIMIT + 3)

    def test_dump_paths_preserve_windows_backslashes_case_and_spaces(self):
        for command, expected in [
            ('dump', None), ('DuMp Test.CSV', 'Test.CSV'),
            (r'dump C:\Temp\MyFile.csv', r'C:\Temp\MyFile.csv'),
            (r'dump "C:\Temp\My Logs.CSV"', r'C:\Temp\My Logs.CSV'),
        ]:
            with self.subTest(command=command):
                validate_command(command)
                self.assertEqual(parse_dump_path(command), expected)
                self.assertEqual(replay_command(CommandEntry(None, 1, command), 1000), command)
        for command in ('dump "', 'dump ""', 'dump "unclosed', 'dump a\nb.csv'):
            with self.subTest(command=command), self.assertRaises(ValueError):
                validate_command(command)

    async def test_dump_default_custom_filtered_and_csv_quoting(self):
        controller = self.controller()
        app = AntennaControllerApp(controller)
        with tempfile.TemporaryDirectory() as directory:
            async with app.run_test(size=(120, 45)) as pilot:
                controller._record_event('SYSTEM', 'Message, "quoted"\nsecond line µT', source='test', route='local')
                controller._record_event('SENSOR', 'node reading', source='node1', route='gateway')
                with patch('antenna_controller.tui.Path', side_effect=lambda p: Path(directory) / p):
                    result = app._execute_manual_command('dump')
                default = Path(directory) / f'antenna_events_{int(controller.commands.started_utc)}.csv'
                self.assertIn(str(default), result)
                with default.open(newline='', encoding='utf-8') as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(list(rows[0]), list(CSV_COLUMNS))
                self.assertEqual(len(rows), 2)
                self.assertIn('Message, "quoted" | second line µT', rows[0]['command'])
                self.assertTrue(rows[0]['utc_time'].endswith('Z'))
                self.assertGreaterEqual(float(rows[0]['elapsed_seconds']), 0)
                app.query_one('#log-select', Select).value = 'sensor'
                await pilot.pause()
                target = Path(directory) / 'Filtered Events.CSV'
                app._execute_manual_command(f'dump "{target}"')
                with target.open(newline='', encoding='utf-8') as stream:
                    filtered = list(csv.DictReader(stream))
                self.assertEqual(len(filtered), 1)
                self.assertIn('SENSOR', filtered[0]['command'])
                self.assertIn('node reading', filtered[0]['command'])
                self.assertEqual(controller.commands.entries()[-1].command, f'dump "{target}"')
                # Recording dump actions also remains valid for batch replay.
                recording = controller.commands.start_csv(directory)
                self.assertEqual(len(load_history(recording)), 2)
                controller.commands.close()

    async def test_widget_and_export_are_bounded_and_file_errors_are_handled(self):
        controller = self.controller()
        app = AntennaControllerApp(controller)
        with tempfile.TemporaryDirectory() as directory:
            async with app.run_test(size=(120, 45)) as pilot:
                for index in range(EVENT_HISTORY_LIMIT + 7):
                    controller._record_event('SYSTEM', f'event {index}', source='test', route='local')
                app._update_events()
                log = app.query_one('#raw-log', RichLog)
                self.assertEqual(log.max_lines, 10_000)
                self.assertEqual(len(log.lines), 10_000)
                self.assertEqual(len(app._displayed_events), 10_000)
                target = Path(directory) / 'last.csv'
                app._execute_manual_command(f'dump "{target}"')
                with target.open(newline='', encoding='utf-8') as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), 10_000)
                self.assertTrue(rows[0]['command'].endswith('event 7'))
                self.assertTrue(rows[-1]['command'].endswith('event 10006'))
                field = app.query_one('#command-input', CommandInput)
                field.value = f'dump "{directory}"'  # a directory is not a file
                app._submit_manual_command()
                self.assertEqual(controller.recent_events()[-1]['type'], 'ERROR')
                self.assertEqual(len(controller.commands.entries()), 1)

    async def test_dump_cannot_overwrite_active_command_recording(self):
        controller = self.controller()
        app = AntennaControllerApp(controller)
        with tempfile.TemporaryDirectory() as directory:
            recording = controller.commands.start_csv(directory)
            try:
                async with app.run_test(size=(120, 45)):
                    with self.assertRaisesRegex(ValueError, 'active command-history'):
                        app._execute_manual_command(f'dump "{recording}"')
                self.assertEqual(load_history(recording), [])
            finally:
                controller.commands.close()
