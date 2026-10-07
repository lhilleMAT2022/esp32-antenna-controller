import csv
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from antenna_controller.bridge import AntennaController, build_parser, main
from antenna_controller.command_history import (
    CSV_COLUMNS, CommandEntry, CommandHistory, CommandReplay, load_history, replay_command,
)


class CommandHistoryTests(unittest.TestCase):
    def controller(self):
        c = AntennaController('unused', command_port=0, state_port=0)
        self.addCleanup(c._server.server_close)
        self.addCleanup(c.state_socket.close)
        self.addCleanup(c.commands.close)
        c.primary.connected.set()
        c.primary.send = lambda message: True
        return c

    def test_nested_actions_record_once_and_keep_partial_sends(self):
        c = self.controller()
        with c.commands.action('point 180'):
            c._send_rotator_command(1, 'goto', azimuth_deg=180)
            c._send_rotator_command(2, 'goto', azimuth_deg=180)
        c.primary.send = lambda message: message['n'] == 1
        with self.assertRaises(RuntimeError), c.commands.action('point 90'):
            c._send_rotator_command(1, 'goto', azimuth_deg=90)
            c._send_rotator_command(2, 'goto', azimuth_deg=90)
        with self.assertRaises(RuntimeError):
            c._send_rotator_command(2, 'stop')
        self.assertEqual([e.command for e in c.commands.entries()], ['point 180', 'point 90'])

    def test_external_calibration_and_reporting_are_recorded(self):
        c = self.controller()
        c.handle_antenna_command(dict(message_type='antenna_command', antenna='REF', command='go_to', target_az_deg=123))
        c.calibration_command(['1', 'status'])
        c.set_reporting_mode('quiet', 60)
        c.set_online(False)
        self.assertEqual([e.command for e in c.commands.entries()],
                         ['goto 2 123.0', 'cal 1 status', 'report quiet 60', 'offline'])

    def test_calibration_automatic_reporting_is_not_replayed_twice(self):
        c = self.controller()
        c.calibration_command(['2', 'start', 'mag'])
        self.assertEqual([e.command for e in c.commands.entries()], ['cal 2 start mag'])
        self.assertEqual(set(c.reporting.pending), {1, 2})

    def test_concurrent_command_contexts_stay_independent(self):
        history = CommandHistory()
        barrier = threading.Barrier(2)
        def record(command):
            with history.action(command):
                barrier.wait(timeout=2)
                history.sent()
        workers = [threading.Thread(target=record, args=(f'stop {n}',)) for n in (1, 2)]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join(3)
            self.assertFalse(worker.is_alive())
        self.assertEqual(sorted(e.command for e in history.entries()), ['stop 1', 'stop 2'])

    def test_csv_round_trip_is_flushed_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            history = CommandHistory()
            self.addCleanup(history.close)
            path = history.start_csv(directory)
            self.assertEqual(path.name, f'antenna_commands_{int(history.started_utc)}.csv')
            with history.action('point "300"', local=True):
                pass
            entries = load_history(path)  # readable before Save or close
            self.assertEqual(entries[0].command, 'point "300"')
            self.assertAlmostEqual(entries[0].utc, history.entries()[0].utc, places=2)
            self.assertAlmostEqual(entries[0].elapsed, history.entries()[0].elapsed, places=5)
            history.save()
            self.assertEqual(load_history(path), entries)
            other = CommandHistory()
            other.filename = history.filename
            with self.assertRaises(FileExistsError):
                other.start_csv(directory)
            history.close()

    def test_disk_failure_preserves_memory_without_failing_sent_command(self):
        history = CommandHistory()
        with tempfile.TemporaryDirectory() as directory:
            history.start_csv(directory)
            writer = history._writer
            try:
                with patch.object(history, '_writer') as broken:
                    broken.writerow.side_effect = OSError('disk full')
                    with history.action('stop 1'):
                        history.sent()
                self.assertIn('disk full', history.save_error)
                self.assertEqual(history.entries()[0].command, 'stop 1')
                self.assertIs(history._writer, writer)
                history.save()
                self.assertEqual(load_history(history.path)[0].command, 'stop 1')
                self.assertFalse(history.save_error)
            finally:
                history.close()

    def write_csv(self, directory, rows):
        path = Path(directory) / 'commands.csv'
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(CSV_COLUMNS)
            writer.writerows(rows)
        return path

    def test_loader_rejects_bad_sequences_before_execution(self):
        cases = [
            [('', -1, 'stop 1')], [('', 'nan', 'stop 1')],
            [('', 2, 'stop 1'), ('', 1, 'stop 2')],
            [('', 0, 'goto 3 90')], [('', 0, 'point 90 dur 30')],
            [('', 0, 'quit'), ('', 1, 'point 90')],
            [('2026-10-07T12:00:00', 0, 'stop 1')],
            [('', 0, 'cal 1 align +x -x 0')],
            [('', 0, 'stop 1', 'extra')],
        ]
        with tempfile.TemporaryDirectory() as directory:
            for rows in cases:
                with self.subTest(rows=rows), self.assertRaisesRegex(ValueError, 'CSV row'):
                    load_history(self.write_csv(directory, rows))
            path = self.write_csv(directory, [('', 2, 'point +20 300 -.5'), ('', 40, 'quit')])
            self.assertEqual(len(load_history(path)), 2)

    def test_replay_timing_equal_times_and_quit(self):
        history = CommandHistory()
        replay = CommandReplay([CommandEntry(None, t, cmd) for t, cmd in
                                [(2, 'point 90'), (2, 'stop 1'), (3, 'quit')]], history)
        sent = []
        for elapsed, count in ((1.99, 0), (2, 2), (2.5, 2), (3, 3), (4, 3)):
            with patch.object(history, 'elapsed', return_value=elapsed):
                replay.tick(sent.append)
            self.assertEqual(len(sent), count)
        self.assertEqual(sent, ['point 90', 'stop 1', 'quit'])

    def test_replay_stops_on_error_or_cancel_without_retry(self):
        history = CommandHistory()
        replay = CommandReplay([CommandEntry(None, 0, 'stop 1'), CommandEntry(None, 0, 'quit')], history)
        with patch.object(history, 'elapsed', return_value=5):
            def fail(command):
                raise RuntimeError('no route')
            replay.tick(fail)
            self.assertIn('row 2', replay.error)
            self.assertIn('no route', replay.error)
            sent = []
            replay.tick(sent.append)
            self.assertFalse(sent)
            replay.error = ''
            replay.cancelled = True
            replay.tick(sent.append)
            self.assertFalse(sent)

    def test_replay_rebases_absolute_slew_but_preserves_relative_and_immediate(self):
        entry = CommandEntry(1791395489.25, 5, 'point 1791395510 300 -0.5')
        self.assertEqual(replay_command(entry, 1800000000.25), 'point 1800000026 300 -0.5')
        for command in ('point +20 300 -0.5', 'goto 2 +20 90', 'point 300', 'goto 2 20', 'quit'):
            self.assertEqual(replay_command(CommandEntry(entry.utc, 5, command), 1800000000), command)

    def test_cli_requires_tui_and_valid_file_before_opening_hardware(self):
        args = build_parser().parse_args(['ac', '--serial', 'COM10', '--tui', '--command-file', 'plan.csv'])
        self.assertEqual(args.command_file, 'plan.csv')
        with patch('antenna_controller.bridge.AntennaController') as controller:
            with self.assertRaises(SystemExit):
                main(['ac', '--serial', 'unused', '--command-file', 'missing.csv'])
            with self.assertRaises(SystemExit):
                main(['ac', '--serial', 'unused', '--tui', '--command-file', 'missing.csv'])
            controller.assert_not_called()
