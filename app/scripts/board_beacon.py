#!/usr/bin/env python3
"""Announce the board on every IPv4 link so the phone relay can find it (doc 31 §2.1).

The phone shares its network over USB (RNDIS); the board gets an address from the phone's DHCP in a subnet
Android picks at random. The relay app listens on UDP 37020 and takes the sender address of this beacon as
the board host. Runs on the board with the stock Python 3.9, standard library only.

    python3 scripts/board_beacon.py                 # every 2 s, all interfaces except lo
    python3 scripts/board_beacon.py --once -v       # one round, print where it went

Sends to each interface's directed broadcast (`ip -4 -o addr` "brd"), not 255.255.255.255: a limited
broadcast leaves through one interface only, which may be eth1 instead of the phone's usb0.
"""

import argparse
import json
import socket
import subprocess
import time

PORT = 37020


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


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--interval", type=float, default=2.0)
    p.add_argument("--gateway", type=int, default=8090, help="board gateway port (src/web/serve.py --port)")
    p.add_argument("--ssh", type=int, default=22)
    p.add_argument("--once", action="store_true")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    host = socket.gethostname()
    while True:
        payload = json.dumps(dict(kind="arm-board", host=host, gateway=args.gateway, ssh=args.ssh,
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


if __name__ == "__main__":
    raise SystemExit(main())
