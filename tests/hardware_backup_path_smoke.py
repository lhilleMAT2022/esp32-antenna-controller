"""Bench test AC -> TCP relay -> node 2 and node/CYD telemetry return paths."""

import argparse
import json
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from antenna_controller.bridge import AntennaController, PiSerialRelay, json_line


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gateway-port", required=True)
    parser.add_argument("--node-port", required=True)
    parser.add_argument("--target", type=float, default=90.0)
    args = parser.parse_args()

    state_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state_receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    state_receiver.bind(("127.0.0.1", 31989))
    state_receiver.settimeout(1.0)

    relay = PiSerialRelay(
        args.node_port, listen_host="127.0.0.1", listen_port=31995
    )
    relay.serial.start()
    relay_thread = threading.Thread(
        target=relay.server.serve_forever, daemon=True
    )
    relay_thread.start()

    ac = AntennaController(
        args.gateway_port,
        backup_host="127.0.0.1",
        prefer_backup_node2=True,
    )
    ac.start()
    try:
        deadline = time.monotonic() + 12.0
        saw_ref_state = False
        while time.monotonic() < deadline and not (
            saw_ref_state
            and ac.states.route_fresh(2, "gateway")
            and ac.states.route_fresh(2, "backup")
        ):
            try:
                raw, _ = state_receiver.recvfrom(8192)
            except socket.timeout:
                continue
            state = json.loads(raw)
            if state.get("message_type") == "antenna_state" and state.get(
                "antenna"
            ) == "REF":
                saw_ref_state = True
                print("STATE", json.dumps(state, separators=(",", ":")))
        if not saw_ref_state:
            raise RuntimeError("no REF antenna_state received")
        if not ac.states.route_fresh(2, "gateway"):
            raise RuntimeError("no node-2 telemetry received through CYD")
        if not ac.states.route_fresh(2, "backup"):
            raise RuntimeError("no node-2 telemetry received through Pi relay")

        with socket.create_connection(("127.0.0.1", 31988), timeout=3) as client:
            request = {
                "message_type": "antenna_command",
                "antenna": "REF",
                "command": "go_to",
                "target_az_deg": args.target,
            }
            client.sendall(json_line(request))
            response = json.loads(client.makefile("rb").readline())
        print("COMMAND", json.dumps(response, separators=(",", ":")))
        if not response.get("ok") or response.get("route") != "backup":
            raise RuntimeError("command did not use the backup route")
        print("PASS: CYD telemetry, Pi relay telemetry, and backup command path")
        return 0
    finally:
        ac.close()
        relay.server.shutdown()
        relay.server.server_close()
        relay.serial.close()
        state_receiver.close()


if __name__ == "__main__":
    raise SystemExit(main())
