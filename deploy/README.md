# Linux service installation

These units are templates for the deployed Ubuntu host and Raspberry Pi.

1. Copy this repository to `/opt/antenna-controller`.
2. Create a virtual environment and install `requirements.txt`.
3. Copy the applicable `.env.example` to `/etc/default/` without the
   `.env.example` suffix and set the persistent `/dev/serial/by-id/...` path.
4. Copy the applicable `.service` file to `/etc/systemd/system/`.
5. Run `sudo systemctl daemon-reload`, then `sudo systemctl enable --now
   <unit-name>`.

For the node relay, bind TCP 31995 only to the rooftop data-network address and
restrict it at the host firewall to the RF Collection Desktop. Do not deploy
the unauthenticated relay on an Internet-accessible interface.
