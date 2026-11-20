"""scripts/board_beacon.py: the `gateway` port it announces follows the gateway that is actually up (doc 36 §4).

Standard library only; /proc is replaced by hand-written process rows, so it runs on the Mac as well as the board.
"""
import json
import os
import socket
import subprocess
import sys
import time
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import board_beacon as B  # noqa: E402


def gw(port, pid, console=None, lan=False, started=None):
    return dict(role='gateway', port=port, pid=pid, console=console, lan=lan, started=started)


REAL = 'http://127.0.0.1:8765'
SIM = 'http://127.0.0.1:8770'


class PickGatewayTest(unittest.TestCase):
    def test_no_gateway_falls_back_to_arg(self):
        procs = [dict(role='console', port=8765, pid=5), dict(role='camera', port=8080, pid=6)]
        self.assertEqual(B.pick_gateway(procs, {8765, 8080}, 8090), (8090, 'arg'))
        self.assertEqual(B.pick_gateway([], None, 8090), (8090, 'arg'))

    def test_single_hand_started_gateway_on_other_port(self):
        # 2026-11-20: someone started the gateway by hand on :8081 while the relay forwarded to 8090
        procs = [gw(8081, 100, REAL, lan=True)]
        self.assertEqual(B.pick_gateway(procs, {8081, 8765}, 8090), (8081, 'proc'))

    def test_prefers_real_console_with_lan(self):
        procs = [gw(8091, 10, SIM, lan=True), gw(8092, 11, REAL, lan=False), gw(8093, 12, REAL, lan=True)]
        self.assertEqual(B.pick_gateway(procs, None, 8090), (8093, 'proc'))

    def test_lan_beats_real_console_without_lan(self):
        procs = [gw(8092, 11, REAL, lan=False), gw(8091, 10, SIM, lan=True)]
        self.assertEqual(B.pick_gateway(procs, None, 8090), (8091, 'proc'))

    def test_real_console_beats_plain(self):
        procs = [gw(8094, 13), gw(8092, 11, REAL)]
        self.assertEqual(B.pick_gateway(procs, None, 8090), (8092, 'proc'))

    def test_tie_prefers_arg_port_then_oldest(self):
        procs = [gw(8081, 100, REAL, lan=True, started=50.0), gw(8090, 200, REAL, lan=True, started=90.0)]
        self.assertEqual(B.pick_gateway(procs, None, 8090), (8090, 'proc'))
        procs = [gw(8082, 100, REAL, lan=True, started=90.0), gw(8081, 200, REAL, lan=True, started=50.0)]
        self.assertEqual(B.pick_gateway(procs, None, 8090), (8081, 'proc'))

    def test_skips_gateway_that_is_not_listening(self):
        procs = [gw(8090, 10, REAL, lan=True), gw(8081, 11, SIM)]
        self.assertEqual(B.pick_gateway(procs, {8081}, 8090), (8081, 'proc'))
        self.assertEqual(B.pick_gateway(procs[:1], {22}, 8090), (8090, 'arg'))

    def test_console_url_variants(self):
        for url in ('http://127.0.0.1:8765', 'http://127.0.0.1:8765/', 'http://localhost:8765/api'):
            self.assertEqual(B.pick_gateway([gw(8095, 1), gw(8096, 2, url)], None, 8090), (8096, 'proc'), url)
        self.assertIsNone(B._console_port('http://[bad'))
        self.assertIsNone(B._console_port(None))


class DetectGatewayTest(unittest.TestCase):
    def test_uses_board_check(self):
        fake = mock.Mock()
        fake.processes.return_value = [gw(8081, 100, REAL, lan=True)]
        fake.listening.return_value = {8081, 8765}
        with mock.patch.object(B, 'board_check', fake):
            self.assertEqual(B.detect_gateway(8090), (8081, 'proc'))

    def test_unreadable_proc_falls_back(self):
        fake = mock.Mock()
        fake.processes.side_effect = FileNotFoundError('/proc/uptime')
        with mock.patch.object(B, 'board_check', fake):
            self.assertEqual(B.detect_gateway(8090), (8090, 'arg'))
        with mock.patch.object(B, 'board_check', None):
            self.assertEqual(B.detect_gateway(8091), (8091, 'arg'))

    def test_empty_listener_table_means_no_filter(self):
        fake = mock.Mock()
        fake.processes.return_value = [gw(8081, 100, REAL, lan=True)]
        fake.listening.return_value = set()  # /proc/net/tcp unreadable
        with mock.patch.object(B, 'board_check', fake):
            self.assertEqual(B.detect_gateway(8090), (8081, 'proc'))


class PayloadTest(unittest.TestCase):
    def test_once_sends_detected_port(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(('127.0.0.1', 0))
        rx.settimeout(2)
        try:
            with mock.patch.object(B, 'PORT', rx.getsockname()[1]), \
                    mock.patch.object(B, 'broadcasts', return_value=[('lo', '127.0.0.1', '127.0.0.1')]), \
                    mock.patch.object(B, 'detect_gateway', return_value=(8081, 'proc')):
                self.assertEqual(B.main(['--once']), 0)
            body = json.loads(rx.recv(1024).decode())
        finally:
            rx.close()
        self.assertEqual(body['kind'], 'arm-board')
        self.assertEqual(body['gateway'], 8081)
        self.assertEqual(body['gateway_from'], 'proc')
        self.assertEqual(body['ssh'], 22)


# A stand-in whose command line looks like the gateway: `python3 -c <listen> web/serve.py --port N --lan --console ...`
FAKE_GATEWAY = """import socket, sys, time
s = socket.socket(); s.bind(('127.0.0.1', 0)); s.listen(1)
print(s.getsockname()[1], flush=True); time.sleep(30)"""


@unittest.skipUnless(sys.platform.startswith('linux') and os.path.exists('/proc/uptime'), 'reads /proc (board, Linux)')
class ProcTest(unittest.TestCase):
    def test_finds_running_gateway_in_proc(self):
        # the stand-in binds an ephemeral port first; a second one carries that port as --port and listens on nothing
        first = subprocess.Popen([sys.executable, '-c', FAKE_GATEWAY], stdout=subprocess.PIPE, text=True)
        try:
            port = int(first.stdout.readline())
            fake = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)', 'web/serve.py',
                                     '--port', str(port), '--lan', '--console', REAL])
            try:
                deadline = time.time() + 5
                while time.time() < deadline and B.detect_gateway(1)[0] != port:
                    time.sleep(0.1)
                self.assertEqual(B.detect_gateway(1), (port, 'proc'))
            finally:
                fake.kill()
                fake.wait()
        finally:
            first.kill()
            first.wait()
            first.stdout.close()


if __name__ == '__main__':
    unittest.main()
