"""Read compact JSON telemetry from an attached antenna-controller board."""

import argparse
import json
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--require", nargs="*", default=["rp"])
    args = parser.parse_args()

    import serial

    found = set()
    messages = []
    with serial.Serial(args.port, 115200, timeout=0.5) as connection:
        time.sleep(1.0)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            line = connection.readline().decode("utf-8", errors="replace").strip()
            if not line.startswith("{"):
                continue
            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                continue
            message_type = message.get("t")
            if message_type in {"rp", "rs", "ra"}:
                found.add(message_type)
                messages.append(message)
                print(json.dumps(message, separators=(",", ":")))
            if set(args.require).issubset(found):
                break

    missing = set(args.require) - found
    if missing:
        print(f"Missing required message types: {sorted(missing)}")
        return 1
    print(f"PASS: received {sorted(found)} ({len(messages)} messages)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
