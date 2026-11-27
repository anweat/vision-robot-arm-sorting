"""Non-motion gateway fixture shared by browser and Android emulator acceptance."""
import json
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "app/src/web"))
import serve as S


def sample_events(count=1000):
    now = time.time() - count
    events = [dict(t=now - 1, event="pipeline_start", run="offline-1000", cubes=[], registration={})]
    for i in range(count):
        e = dict(t=now + i, event=["pipeline_sorted", "pipeline_missed", "pipeline_skip", "pipeline_approach_failed", "pipeline_cleared"][i % 5],
                 cube=[round(i / 100, 2), 10], category=S.L.CATEGORIES[i % 4], seconds=1.25,
                 reason='中文, "引号"\n换行 & <内容>' if i == 2 else '=SUM(1,2)' if i == 7 else 'offline fixture')
        if i % 5 in (0, 4): e.update(bin="bin2", source="flow")
        events.append(e)
    events.append(dict(t=now + count, event="pipeline_end", run="offline-1000", sorted=count // 5, missed=count // 5,
                       skipped=count // 5, cleared=count // 5, seconds=100))
    for e in events:
        assert not S.L.validate(e), S.L.validate(e)
    return events


class QuietHandler(S.Handler):
    def log_message(self, *args): pass


class FixtureGateway:
    def __init__(self, root, count=1000):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.state = S.State(self.root / "log.jsonl", flow_root=self.root / "flow")
        self.state.scripts = S.FS.Store(self.root / "builtin", self.root / "scripts")
        self.state.log_path.write_text("".join(json.dumps(e, ensure_ascii=False) + "\n" for e in sample_events(count)), encoding="utf-8")
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), QuietHandler)
        self.server.daemon_threads = True
        self.server.state = self.state
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.state.base_url = self.url
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.state.stop_flow()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)
