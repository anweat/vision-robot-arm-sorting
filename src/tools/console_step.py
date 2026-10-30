"""One supervised action through the running teleop console (Mac forward 127.0.0.1:8765).

Each call takes the lease ("on site" confirmed by the operator who asked for it), sends heartbeats while
the action runs, then stops, so nothing moves between calls. All console checks (limits, keep-out,
5 deg segments with readback, hold at readback on resistance) apply. Only use with the operator watching.

    python3 src/tools/console_step.py status
    python3 src/tools/console_step.py jog 2 -5            # joint, delta (1/2/5/10)
    python3 src/tools/console_step.py cart z -1           # x/y/z cm or pitch deg
    python3 src/tools/console_step.py grip open           # wide/open/grip
    python3 src/tools/console_step.py pose 90,105,5,5,90  # J1-J5 via preview + execute
"""
import json
import sys
import threading
import time
import urllib.error
import urllib.request

BASE = 'http://127.0.0.1:8765'
OWNER = 'claude-console-step'


class ConsoleError(RuntimeError):
    pass


def call(path, data=None, timeout=60, token='', owner=OWNER):
    body = None if data is None else json.dumps({'owner': owner, **data}).encode()
    req = urllib.request.Request(BASE + path, data=body,
                                 headers={'Content-Type': 'application/json', 'X-Console-Token': token})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return {'error': json.loads(e.read()).get('error')}


def status():
    return call('/api/status')


class Session:
    """Lease + heartbeat for one or more actions; always stops on exit."""

    def __init__(self, owner=OWNER):
        self.owner = owner
        boot = call('/api/bootstrap')
        self.token = boot['token']
        if boot['status'].get('busy'):
            raise ConsoleError('console busy, nothing sent')
        self.done = threading.Event()

    def __enter__(self):
        self.rearm()
        threading.Thread(target=self._beat, daemon=True).start()
        return self

    def rearm(self):
        """Enable again (the console disarms when it holds on resistance); the operator is still on site."""
        for i in range(8):     # right after a hold the readback is stale for a moment
            try:
                r = self._post('/api/enable', {'onsite': True})
                break
            except ConsoleError as e:
                if '回读过期' not in str(e) or i == 7:
                    raise
                time.sleep(0.4)
        self.revision = r['revision']

    def __exit__(self, *exc):
        self.done.set()
        call('/api/stop', {}, token=self.token, owner=self.owner)

    def _beat(self):
        """A beat that fails (a dropped connection, a slow answer) is tried again at the next one: the console's lease
        lapses by itself 2 s after the last good beat, and a lapsed lease is not revived, so retrying cannot keep
        the arm enabled any longer than it was meant to be -- but dying on the first error ended the lease for good."""
        while not self.done.wait(.5):
            try:
                r = call('/api/heartbeat', {}, token=self.token, owner=self.owner, timeout=5)
                if isinstance(r, dict) and r.get('error'):
                    print(f"heartbeat refused: {r['error']}", file=sys.stderr, flush=True)
            except Exception as e:      # noqa: BLE001
                print(f"heartbeat failed: {e!r}", file=sys.stderr, flush=True)

    def _post(self, path, data, timeout=60):
        r = call(path, data, timeout=timeout, token=self.token, owner=self.owner)
        if r.get('error'):
            raise ConsoleError(r['error'])
        if 'revision' in r:
            self.revision = r['revision']
        return r

    def jog(self, joint, delta):
        return self._post('/api/jog', {'joint': joint, 'delta': delta, 'revision': self.revision})

    def cart(self, axis, amount):
        return self._post('/api/cartesian', {'axis': axis, 'amount': amount, 'revision': self.revision})

    def grip(self, action):
        return self._post('/api/gripper', {'action': action, 'revision': self.revision})

    def pose(self, target):
        plan = self._post('/api/pose/plan', {'target': list(target)})
        return self._post('/api/pose/execute', {'plan_id': plan['id'], 'revision': plan['revision']}, timeout=120)


def show(s):
    fk = s.get('fk') or {}
    print(json.dumps(dict(pose=s.get('pose'), command=s.get('command'),
                          tip_cm=[round(v * 100, 1) for v in fk.get('tip', [])], pitch=fk.get('tip_pitch_deg'),
                          busy=s.get('busy'), armed=s.get('armed'), profile=s.get('profile'), error=s.get('error'),
                          margin_cm=s.get('clearance_cm')), ensure_ascii=False))


def main(argv):
    if argv[0] == 'status':
        return show(status())
    try:
        with Session() as s:
            if argv[0] == 'jog':
                s.jog(int(argv[1]), int(argv[2]))
            elif argv[0] == 'cart':
                s.cart(argv[1], float(argv[2]))
            elif argv[0] == 'grip':
                s.grip(argv[1])
            elif argv[0] == 'pose':
                s.pose([int(v) for v in argv[1].split(',')])
            else:
                sys.exit(__doc__)
    except ConsoleError as e:
        print('ERROR:', e)
    show(status())


if __name__ == '__main__':
    main(sys.argv[1:] or ['status'])
