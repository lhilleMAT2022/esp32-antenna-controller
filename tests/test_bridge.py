import json
import unittest
from types import SimpleNamespace

from antenna_controller.bridge import (
    AntennaController,
    NodeStateStore,
    SerialDeviceIdentity,
    SerialJsonLink,
    _serial_settings_from_args,
    build_parser,
    compact_rotator_command,
    json_line,
    parse_board_line,
)


class ProtocolTests(unittest.TestCase):
    def test_compact_goto_command(self):
        command = compact_rotator_command(2, 17, "goto", azimuth_deg=123.45)
        self.assertEqual(
            command,
            {"t": "rc", "n": 2, "q": 17, "c": "goto", "az": 123.5},
        )

    def test_json_line_is_compact_and_finite(self):
        command = compact_rotator_command(1, 1, "stop")
        encoded = json_line(command)
        self.assertEqual(json.loads(encoded), command)
        self.assertTrue(encoded.endswith(b"\n"))

    def test_human_log_line_is_ignored(self):
        self.assertIsNone(parse_board_line("SENSOR heading=251.0M"))

    def test_status_and_sensor_merge(self):
        store = NodeStateStore()
        state = store.update(
            {
                "t": "rp",
                "n": 2,
                "q": 1,
                "h": 99.5,
                "tg": None,
                "mv": 0,
                "rg": -58,
                "rn": -61,
            },
            "gateway",
        )
        self.assertEqual(state.heading_deg, 99.5)
        self.assertEqual(state.rssi_at_gateway_dbm, -58)
        self.assertEqual(state.rssi_at_node_dbm, -61)
        state = store.update(
            {
                "t": "rs",
                "n": 2,
                "q": 2,
                "mh": 250.1,
                "m": [-17.0, -45.0, 5.0],
                "a": [-0.04, 0.60, -0.79],
                "f": 48.0,
                "r": 143.0,
                "p": 2.5,
                "sf": 15,
            },
            "backup",
        )
        self.assertEqual(state.heading_deg, 99.5)
        self.assertEqual(state.field_strength_ut, 48.0)
        self.assertEqual(state.route, "backup")

    def test_invalid_azimuth_rejected(self):
        with self.assertRaises(ValueError):
            compact_rotator_command(2, 1, "goto", azimuth_deg=360.0)

    def test_icd_command_maps_to_gateway(self):
        controller = AntennaController(
            "unused", command_port=0, state_port=0
        )
        sent = []
        controller.primary.connected.set()
        controller.primary.send = lambda message: sent.append(message) or True
        try:
            response = controller.handle_antenna_command(
                {
                    "message_type": "antenna_command",
                    "antenna": "SURV",
                    "command": "go_to",
                    "target_az_deg": 83.0,
                }
            )
        finally:
            controller._server.server_close()
            controller.state_socket.close()
        self.assertTrue(response["ok"])
        self.assertEqual(response["route"], "gateway")
        self.assertEqual(
            sent[0],
            {"t": "rc", "n": 1, "q": 1, "c": "goto", "az": 83.0},
        )

    def test_offline_mode_rejects_only_external_commands(self):
        controller = AntennaController(
            "unused", command_port=0, state_port=0
        )
        controller.set_online(False)
        try:
            with self.assertRaisesRegex(ValueError, "OFFLINE"):
                controller.handle_antenna_command(
                    {
                        "message_type": "antenna_command",
                        "antenna": "SURV",
                        "command": "stop",
                    },
                    external=True,
                )
        finally:
            controller._server.server_close()
            controller.state_socket.close()

    def test_serial_link_follows_same_usb_device_to_new_com_port(self):
        original = SimpleNamespace(
            device="COM6",
            vid=0x1A86,
            pid=0x7523,
            serial_number=None,
            location="1-5.4.2.2",
        )
        moved = SimpleNamespace(
            device="COM11",
            vid=0x1A86,
            pid=0x7523,
            serial_number=None,
            location="1-5.4.2.2",
        )
        other = SimpleNamespace(
            device="COM6",
            vid=0x10C4,
            pid=0xEA60,
            serial_number="0001",
            location="1-5.4.2.3",
        )
        link = SerialJsonLink("gateway", "COM6", lambda *_: None)
        link._device_identity = SerialDeviceIdentity.from_port_info(original)
        self.assertIs(link._resolve_port([other, moved]), moved)

    def test_serial_link_refuses_reused_com_port(self):
        original = SimpleNamespace(
            device="COM7",
            vid=0x10C4,
            pid=0xEA60,
            serial_number="0001",
            location="1-5.4.2.3",
        )
        replacement = SimpleNamespace(
            device="COM7",
            vid=0x1A86,
            pid=0x7523,
            serial_number=None,
            location="1-5.4.2.2",
        )
        link = SerialJsonLink("node", "COM7", lambda *_: None)
        link._device_identity = SerialDeviceIdentity.from_port_info(original)
        self.assertIsNone(link._resolve_port([replacement]))

    def test_serial_link_follows_unique_vid_pid_without_serial_number(self):
        original = SimpleNamespace(
            device="COM6",
            vid=0x1A86,
            pid=0x7523,
            serial_number=None,
            location="1-5.4.2.2",
        )
        moved = SimpleNamespace(
            device="COM10",
            vid=0x1A86,
            pid=0x7523,
            serial_number=None,
            location="1-5.4.2.1",
        )
        link = SerialJsonLink("gateway", "COM6", lambda *_: None)
        link._device_identity = SerialDeviceIdentity.from_port_info(original)
        self.assertIs(link._resolve_port([moved]), moved)

    def test_serial_link_refuses_ambiguous_vid_pid_fallback(self):
        original = SimpleNamespace(
            device="COM6",
            vid=0x1A86,
            pid=0x7523,
            serial_number=None,
            location="1-5.4.2.2",
        )
        candidates = [
            SimpleNamespace(
                device=f"COM{port}",
                vid=0x1A86,
                pid=0x7523,
                serial_number=None,
                location=f"1-5.4.2.{port}",
            )
            for port in (10, 11)
        ]
        link = SerialJsonLink("gateway", "COM6", lambda *_: None)
        link._device_identity = SerialDeviceIdentity.from_port_info(original)
        self.assertIsNone(link._resolve_port(candidates))

    def test_posix_port_and_serial_settings_parse(self):
        args = build_parser().parse_args(
            [
                "ac",
                "--serial",
                "/dev/ttyACM0",
                "--baud",
                "230400",
                "--data-bits",
                "7",
                "--parity",
                "E",
                "--stop-bits",
                "2",
                "--rtscts",
            ]
        )
        settings = _serial_settings_from_args(args)
        self.assertEqual(args.serial, "/dev/ttyACM0")
        self.assertEqual(settings.baud, 230400)
        self.assertEqual(settings.data_bits, 7)
        self.assertEqual(settings.parity, "E")
        self.assertEqual(settings.stop_bits, 2.0)
        self.assertTrue(settings.rtscts)


if __name__ == "__main__":
    unittest.main()
