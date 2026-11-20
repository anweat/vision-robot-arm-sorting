#!/usr/bin/env python3
"""Announce the board on every IPv4 link so the phone relay can find it (doc 31 §2.1).

The phone shares its network over USB (RNDIS); the board gets an address from the phone's DHCP in a subnet
Android picks at random. The relay app listens on UDP 37020 and takes the sender address of this beacon as
the board host. Runs on the board with the stock Python 3.9, standard library only.

    python3 scripts/board_beacon.py                 # every 2 s, all interfaces except lo
    python3 scripts/board_beacon.py --once -v       # one round, print where it went

Sends to each interface's directed broadcast (`ip -4 -o addr` "brd"), not 255.255.255.255: a limited
broadcast leaves through one interface only, which may be eth1 instead of the phone's usb0.

The `gateway` field is the port of the gateway that is actually up (doc 36 §4): every round it reads /proc for
`web/serve.py` processes (board_check.processes()) and keeps the ones whose `--port` has a TCP listener. Among
those it prefers one with `--lan` whose `--console` is the real console on :8765, then `--lan`, then the real
console; among equals the `--gateway` value, then the oldest process. With no gateway found it sends `--gateway`.
`gateway_from` says which ("proc" or "arg"). The phone relay forwards to this port unless the App has one typed in.
"""

import argparse
import json
import socket
import subprocess
import time
from urllib.parse import urlsplit

try:  # same directory on the board (/root/arm-app/scripts); without it the beacon still runs on --gateway
    import board_check
except ImportError:  # pragma: no cover
    board_check = None

PORT = 37020
CONSOLE_PORT = 8765  # src/teleop/server.py, the real-machine console


def broadcasts():
    """[(ifname, addr, brd)] for every IPv4 address with a broadcast address, loopback excluded."""
    out = subprocess.run(["ip", "-4", "-o", "addr", "show"], capture_output=True, text=True, check=False).stdout
    found = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 4 or parts[1] == "lo" or "brd" not in parts:
            continue
        found.append((parts[1], parts[3].split("/")[0], parts[parts.index("brd") + 1]))
    return found


def _console_port(url):
    try:
        return urlsplit(url).port if url else None
    except ValueError:
        return None


def pick_gateway(procs, listening, fallback):
    """(port, "proc") of the gateway the relay should forward to, or (fallback, "arg") if none is up.

    procs: board_check.processes() rows; listening: ports with a TCP listener, or None when unknown (no filter).
    """
    gws = [p for p in procs if p.get("role") == "gateway" and p.get("port")]
    if listening is not None:
        gws = [p for p in gws if p["port"] in listening]
    if not gws:
        return fallback, "arg"

    def rank(p):
        real = _console_port(p.get("console")) == CONSOLE_PORT
        lan = bool(p.get("lan"))
        return (not (real and lan), not lan, not real, p["port"] != fallback,
                p.get("started") or float("inf"), p.get("pid", 0))

    return min(gws, key=rank)["port"], "proc"


def detect_gateway(fallback):
    """pick_gateway() on this machine's /proc; (fallback, "arg") if /proc cannot be read."""
    if board_check is None:
        return fallback, "arg"
    try:
        procs = board_check.processes()
    except Exception:  # not Linux, /proc unreadable: keep beaconing
        return fallback, "arg"
    try:
        listening = board_check.listening() or None
    except Exception:
        listening = None
    return pick_gateway(procs, listening, fallback)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--interval", type=float, default=2.0)
    p.add_argument("--gateway", type=int, default=8090, help="gateway port to announce when no running src/web/serve.py is found")
    p.add_argument("--ssh", type=int, default=22)
    p.add_argument("--once", action="store_true")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        host = socket.gethostname()
        last = None
        while True:
            gateway, source = detect_gateway(args.gateway)
            if args.verbose and (gateway, source) != last:
                print(f"gateway :{gateway} ({source})", flush=True)
            last = (gateway, source)
            payload = json.dumps(dict(kind="arm-board", host=host, gateway=gateway, gateway_from=source, ssh=args.ssh,
                                      t=round(time.time(), 1))).encode()
            for ifname, addr, brd in broadcasts():
                try:
                    sock.sendto(payload, (brd, PORT))
                    if args.verbose:
                        print(f"{ifname} {addr} -> {brd}:{PORT}", flush=True)
                except OSError as exc:
                    if args.verbose:
                        print(f"{ifname} {brd}: {exc}", flush=True)
            if args.once:
                return 0
            time.sleep(args.interval)
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
