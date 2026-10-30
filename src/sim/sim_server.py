#!/usr/bin/env python3
"""Simulation back end for front-end development: the console API with a simulated arm, table and camera.

    uv run --with 'opencv-python<5' --with numpy --with onnxruntime src/sim/sim_server.py [--lan]

It never imports Arm_Lib and cannot reach hardware. The arm is driven by the same Controller as the real
console (src/teleop/core.py), so limits, keep-out checks, the enable/heartbeat/revision rules and the
status object are the real ones; every /api/* path of docs/atlas-200i-dk-a2/15-接口文档.md §4 answers
here too. On top of that:

  * the wrist camera is rendered from the arm's pose and the blocks on the table (src/sim/render.py)
  * closing the gripper on a block picks it up, opening drops it (src/sim/world.py)
  * /api/sim/* reads and edits the scene (blocks, the printed map, bins and other props) and checks
    the block detector against it
  * /snapshot.jpg, /stream.mjpg, /health mimic the board camera service (§6)
  * /api/log, /api/log/stream, /api/log/records, /api/log/summary and /api/scene serve the event log and
    the last scan in the form of docs/atlas-200i-dk-a2/18-日志契约.md; POST /api/sim/sort runs the real
    sorting pipeline against this simulation (src/sim/run_pipeline.py) to fill that log
  * any origin may call it (CORS), and --lan listens on all interfaces, so a front end on another port
    or a phone can use it directly. That is acceptable only because nothing real moves.

/sim/ is the shell page (src/sim/web): the arm and the blocks in 3D, the camera view, the detector check
and every call listed and runnable. The old console page is still served at / as a reference.
"""
from __future__ import annotations

import argparse
import json
import secrets
import sys
import threading
import time
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("vision", "sim", "teleop", "tools", "interact"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import server as console  # noqa: E402  (src/teleop/server.py)
from camera import Camera  # noqa: E402
from core import Controller  # noqa: E402
from localization import Locator, annotate as annotate_location, locate_board  # noqa: E402
import evaluate as E  # noqa: E402
import intents as I  # noqa: E402  (src/interact)
import log_contract as L  # noqa: E402
import render as R  # noqa: E402
import world as W  # noqa: E402


SIM_DATA = ROOT / "data" / "sim"        # where src/sim/run_pipeline.py writes: log.jsonl, pipeline/<run>/scene.json
WEB = Path(__file__).parent / "web"
SHELL = {"/sim/": ("index.html", "text/html; charset=utf-8"), "/sim/app.js": ("app.js", "text/javascript"),
         "/sim/style.css": ("style.css", "text/css")}


class SimCamera(Camera):
    """The rendered wrist camera. Frames carry clock 'sim', which never satisfies the hardware gate."""
    simulated = True

    def __init__(self, world: W.World, seed: int = 0):
        super().__init__()
        self.source = "sim"                 # reported as kind 'live'
        self.world = world
        self.renderer = R.Renderer(world.items, seed=seed)
        self.noise = np.random.default_rng(seed)
        self.map_revision = None
        self.view = None                    # (world revision, servos, clean frame, truth, latest noisy frame)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def _info(self):
        return dict(super()._info(), simulated=True)

    def run(self):
        while not self.closed.is_set():
            try:
                with self.world.lock:
                    servos, blocks, revision = self.world.render_list()
                    solids, laid = self.world.solids(), (self.world.map_revision, self.world.table, self.world.map_data)
                if laid[0] != self.map_revision:
                    self.renderer.set_map(laid[1], laid[2])
                    self.map_revision = laid[0]
                if self.view is None or self.view[0] != revision:
                    frame, truth = self.renderer.render(servos, blocks, solids)
                else:
                    frame, truth = self.view[2], self.view[3]
                image = R.sensor(frame, self.noise)
                ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 88])
                with self.lock:
                    self.view = (revision, servos, frame, truth, image)
                self.publish(buf.tobytes(), time.time(), "sim")
            except Exception as exc:
                with self.lock:
                    self.error = f"仿真渲染失败：{exc}"
            self.closed.wait(0.2)

    def current(self):
        """(world revision, servos, image, truth, seq) of the latest frame, or None before the first."""
        with self.lock:
            if self.view is None:
                return None
            revision, servos, _, truth, image = self.view
            return revision, servos, image, truth, self.seq


class SimLocator(Locator):
    """The console's map localization (colour blobs on the registered map) run on the simulated frames,
    so status.localization has the same shape as on the board, plus the 4-class detector's boxes under
    `blocks4`. The official 3-class model needs the Ascend runtime and is absent here."""

    def __init__(self, control, camera, directory, detect):
        super().__init__(control, camera, None, directory)
        self.detect = detect

    def run(self):
        while not self.closed.is_set():
            try:
                state = self.control.status()
                view = self.camera.current()
                jpeg, meta = self.camera.snapshot()
                if view is None or not jpeg:
                    raise ValueError("画面不可用或过期")
                if state["busy"]:
                    raise ValueError("需要静止的回读")
                if not self.control.arm.world.matches_default_map():
                    raise ValueError("地图定位按 map_a4_v2 的码位求解；当前地图的码位不同或没有地图，未运行")
                result = locate_board(jpeg, state["pose"])
                try:
                    result["blocks4"] = self.detect(view[2])
                except Exception as exc:
                    result["blocks4"] = [dict(error=str(exc))]
                after = self.control.status()
                if after["busy"] or after["revision"] != state["revision"]:
                    raise ValueError("计算期间姿态发生变化，结果已丢弃")
                result.update(t=time.time(), frame=meta, servos=state["pose"], simulated=True)
                ok, buf = cv2.imencode(".jpg", annotate_location(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR), result))
                with self.lock:
                    self.result = dict(board=result)
                    self.images = dict(board=buf.tobytes() if ok else jpeg)
            except Exception as exc:
                with self.lock:
                    # the pose rides along so a scan does not wait out its timeout on a view without codes
                    self.result = dict(board=dict(error=str(exc), t=time.time(), objects=[], simulated=True,
                                                  servos=self.control.status()["pose"]))
                    self.images = {}
            self.closed.wait(1)


class SimApp(console.App):
    def __init__(self, controller, camera, world, model=None):
        self.world, self.model = world, model      # scene() is called by the console's constructor
        super().__init__(controller, camera)
        self.detector = None
        self.detector_lock = threading.Lock()
        self.check = None                   # last detector check, cached per camera frame
        self.log_path = SIM_DATA / "log.jsonl"
        self.sorting = None                 # the run_pipeline.py child process, when one was started here
        self.port = None                    # set once the HTTP server is listening
        self.sorting_tail = []
        self.interaction = I.Interaction(self.run_command)
        self.locator = SimLocator(controller, camera, controller.directory, self.detect)
        threading.Thread(target=self.locator.run, daemon=True).start()

    def detect(self, image, conf=0.25):
        with self.detector_lock:
            if self.detector is None:
                try:
                    from blocks_detector import BlocksDetector
                    self.detector = BlocksDetector(self.model, "onnx")
                except Exception as exc:
                    raise ValueError(f"检测模型不可用（需要 onnxruntime 和 data/models/blocks4 下的 best.onnx）：{exc}") from exc
            return self.detector.detect(image, conf=conf)

    def scene(self):
        """As the console's, with the map that is on the simulated table (an empty layout if none)."""
        data = self.world.map_data or dict(layout_m={}, side_m=0)
        off = self.world.table.offset
        placement = dict(base_from_map=dict(dx=float(off[0]), dy=float(off[1]), dz=float(off[2])))
        return dict(super().scene(), map=dict(data, placement=dict(data.get("placement", {}), **placement), name=self.world.map_name))

    def status(self):
        snap = self.world.snapshot()
        return dict(super().status(), sim=dict(revision=snap["revision"], blocks=len(snap["blocks"]), holding=snap["holding"]))

    def detector_check(self, conf=0.25):
        """The detector's boxes on the current frame against the blocks that are really in it."""
        view = self.camera.current()
        if view is None:
            raise ValueError("仿真画面尚未就绪")
        revision, servos, image, truth, seq = view
        with self.lock:
            if self.check and self.check["key"] == (seq, conf):
                return self.check
        dets = self.detect(image, conf)
        result = E.compare(truth, dets)
        ok, buf = cv2.imencode(".jpg", E.annotate(image, result))
        result.update(key=(seq, conf), frame_seq=seq, world_revision=revision, servos=servos, conf=conf, match_iou=E.MATCH_IOU,
                      model=str(self.detector.model.relative_to(ROOT)) if self.detector.model.is_relative_to(ROOT) else self.detector.model.name,
                      detections=dets, image=buf.tobytes(),
                      evidence="simulated frame; the faces are the artwork the detector was partly trained on")
        with self.lock:
            self.check = result
        return result

    def latest_run(self) -> Path | None:
        runs = sorted(p for p in (SIM_DATA / "pipeline").glob("*") if (p / "scene.json").exists())
        return runs[-1] if runs else None

    def sort_state(self) -> dict:
        p = self.sorting
        return dict(running=bool(p and p.poll() is None), exit_code=None if p is None else p.poll(),
                    output=self.sorting_tail[-12:], log_lines=L.read(self.log_path)[1])

    def start_sort(self, body) -> dict:
        """Run the real sorting pipeline against this simulation, as a child process (run_pipeline.py)."""
        import subprocess
        if self.sorting and self.sorting.poll() is None:
            raise ValueError("分拣任务已在运行")
        if self.control.status()["armed"]:
            raise ValueError("有页面持有操作权；先停止（/api/stop）再启动分拣任务")
        rounds, blocks = body.get("rounds", 1), body.get("blocks", 4)
        if type(rounds) is not int or not 1 <= rounds <= 50 or type(blocks) is not int or not 1 <= blocks <= 6:
            raise ValueError("rounds 须为 1–50，blocks 须为 1–6")
        if not self.world.matches_default_map() or self.world.props:
            raise ValueError("分拣任务按 map_a4_v2 的码位投放：先换回 v2 布局的地图并移走桌上的物体")
        if self.port is None:
            raise ValueError("服务端口未知，无法启动分拣任务")
        port = self.port
        args = [sys.executable, str(Path(__file__).with_name("run_pipeline.py")), "--base", f"http://127.0.0.1:{port}",
                "--rounds", str(rounds), "--blocks", str(blocks)]
        if type(body.get("seed")) is int:
            args += ["--seed", str(body["seed"])]
        if body.get("keep_scene") is True:
            args.append("--keep-scene")
        self.sorting_tail = []
        self.sorting = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=ROOT)

        def drain(p):
            for line in p.stdout:
                if not line.startswith('{"t"'):           # the events are in the log already
                    self.sorting_tail.append(line.rstrip()[:300])
                    del self.sorting_tail[:-50]
        threading.Thread(target=drain, args=(self.sorting,), daemon=True).start()
        return self.sort_state()

    def stop_sort(self) -> dict:
        if self.sorting and self.sorting.poll() is None:
            self.sorting.terminate()
            self.control.stop()
        return self.sort_state()

    def run_command(self, command, preset):
        """What an accepted voice/gesture command does here (src/interact/intents.py calls this)."""
        if command == "stop":
            self.stop_sort()
            self.control.stop()
            return "已停止后续指令"
        if command == "start":
            self.start_sort({})
            return "分拣任务已启动"
        if command == "run_preset":
            raise NotImplementedError(f"预设动作“{preset}”尚未接入，待硬件组提供")
        raise ValueError(f"未知指令 {command}")

    def interact(self, path, body):
        """/api/interact/*: the voice and gesture side. Accepted commands also land in the log as `interaction`."""
        it = self.interaction
        if path == "/api/interact/arm":
            result = it.arm(body.get("armed") is True)
        elif path == "/api/interact/voice":
            result = it.voice(body.get("text"))
        elif path == "/api/interact/gesture":
            result = it.gesture(body.get("label"), body.get("confidence"))
        else:
            raise ValueError("未知操作")
        event = result.pop("event", None)
        if event:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(event, ensure_ascii=False) + "\n")
        return result

    def action(self, path, body):
        if path.startswith("/api/interact/"):
            return self.interact(path, body)
        if path == "/api/sim/sort":
            return self.start_sort(body)
        if path == "/api/sim/sort/stop":
            return self.stop_sort()
        if not path.startswith("/api/sim/"):
            return super().action(path, body)
        world, c = self.world, self.control
        if c.busy:
            raise ValueError("机械臂动作中，不能改动场景")
        if path == "/api/sim/reset":
            world.reset(body.get("count", 4), body.get("seed"))
        elif path == "/api/sim/block":
            block_id = body.get("id")
            if block_id is not None and not isinstance(block_id, str):
                raise ValueError("物块 id 无效")
            category = body.get("category")
            if category is None and block_id in world.blocks:
                category = world.blocks[block_id]["category"]
            world.place(category, body.get("map_cm"), body.get("yaw_deg", 0), body.get("up", W.TOP), block_id)
        elif path == "/api/sim/remove":
            world.remove(body.get("id"))
        elif path == "/api/sim/faults":
            world.set_faults(body.get("grip_miss"))
        elif path == "/api/sim/map":
            world.set_map(body.get("name"))
        elif path == "/api/sim/prop":
            keys = ("yaw_deg", "name", "category", "colour", "inner_cm", "height_cm", "wall_cm", "size_cm")
            world.add_prop(body.get("kind"), body.get("map_cm"), **{k: body[k] for k in keys if k in body})
        elif path == "/api/sim/prop/remove":
            world.remove_prop(body.get("id"))
        elif path == "/api/sim/bins":
            world.load_bins(body.get("name"))
        else:
            raise ValueError("未知操作")
        c.log("sim_scene", path=path, request={k: v for k, v in body.items() if k != "owner"})
        return world.snapshot()


def handler(app: SimApp):
    class Handler(console.handler(app)):
        def local(self):
            pass                            # simulation only: any host, any origin

        def send(self, body, ctype="application/json", status=200, extra=()):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            for name, value in (("Content-Type", ctype), ("Content-Length", str(len(body))), ("Cache-Control", "no-store"),
                                ("X-Content-Type-Options", "nosniff"), *self.cors(), *extra):
                self.send_header(name, value)
            self.end_headers()
            self.wfile.write(body)

        @staticmethod
        def cors():
            return (("Access-Control-Allow-Origin", "*"), ("Access-Control-Allow-Headers", "Content-Type, X-Console-Token"),
                    ("Access-Control-Allow-Methods", "GET, POST, OPTIONS"),
                    ("Access-Control-Expose-Headers", "X-Frame-Time, X-Frame-Seq"), ("Access-Control-Max-Age", "600"))

        def do_OPTIONS(self):
            self.send_response(204)
            for name, value in self.cors():
                self.send_header(name, value)
            self.send_header("Content-Length", "0")
            self.end_headers()

        def do_GET(self):
            url = urlsplit(self.path)
            path, query = url.path, parse_qs(url.query)
            try:
                if path in SHELL:
                    name, mime = SHELL[path]
                    self.send((WEB / name).read_bytes(), mime)
                elif path == "/sim":
                    self.send(b"", status=302, extra=(("Location", "/sim/"),))
                elif path == "/api/sim/world":
                    self.send(app.world.snapshot())
                elif path == "/api/sim/options":
                    self.send(dict(maps=sorted(f.stem for f in W.CONFIG.glob("map_*.json")),
                                   bins=sorted(f.stem for f in W.CONFIG.glob("bins_*.json")),
                                   categories=list(W.CATEGORIES), faces=app.world.items))
                elif path in ("/api/sim/detect", "/api/sim/annotated"):
                    conf = float(query.get("conf", ["0.25"])[0])
                    if not 0.01 <= conf <= 0.99:
                        raise ValueError("conf 必须在 0.01–0.99")
                    result = app.detector_check(conf)
                    if path.endswith("annotated"):
                        self.send(result["image"], "image/jpeg")
                    else:
                        self.send({k: v for k, v in result.items() if k not in ("image", "key")})
                elif path == "/api/sim/sort":
                    self.send(app.sort_state())
                elif path == "/api/interact":
                    self.send(app.interaction.describe())
                elif path == "/api/log":
                    after = int(query.get("after", ["0"])[0])
                    limit = min(int(query.get("limit", ["500"])[0]), 2000)
                    events, total = L.read(app.log_path, after, limit)
                    self.send(dict(contract_version=L.VERSION, events=events, total=total,
                                   next=events[-1]["seq"] if events else min(after, total)))
                elif path == "/api/log/records":
                    self.send(dict(contract_version=L.VERSION, records=L.records(L.read(app.log_path)[0])))
                elif path == "/api/log/summary":
                    self.send(L.summary(L.read(app.log_path)[0]))
                elif path == "/api/log/stream":
                    self.log_stream(int(query.get("after", [self.headers.get("Last-Event-ID") or "0"])[0]))
                elif path in ("/api/scene", "/api/scene/image"):
                    run = app.latest_run()
                    if run is None:
                        return self.send({"error": "还没有扫描结果"}, status=404)
                    if path == "/api/scene":
                        self.send((run / "scene.json").read_bytes())
                    else:
                        self.send((run / "scene.png").read_bytes(), "image/png")
                elif path == "/snapshot.jpg":
                    jpeg, meta = app.camera.snapshot()
                    if not jpeg:
                        return self.send({"error": "no frame"}, status=503)
                    self.send(jpeg, "image/jpeg", extra=(("X-Frame-Time", f"{meta['stamp']:.6f}"), ("X-Frame-Seq", str(meta["seq"]))))
                elif path == "/stream.mjpg":
                    self.stream()
                elif path == "/health":
                    meta = app.camera.info()
                    self.send(dict(device="simulation", frames=meta["seq"], latest_age_s=meta["age_s"], size=[R.IMAGE_W, R.IMAGE_H]))
                else:
                    super().do_GET()
            except ValueError as exc:
                self.send({"error": str(exc)}, status=409 if path.startswith("/api/sim/") else 400)

        def log_stream(self, after: int):
            """Server-sent events: one message per log line, id = seq, so a reconnecting EventSource
            resumes where it stopped (Last-Event-ID)."""
            self.send_response(200)
            for name, value in (("Content-Type", "text/event-stream; charset=utf-8"), ("Cache-Control", "no-store"),
                                ("X-Accel-Buffering", "no"), *self.cors()):
                self.send_header(name, value)
            self.end_headers()
            quiet = time.time()
            try:
                while not app.closed.is_set():
                    events, _ = L.read(app.log_path, after, 500)
                    for e in events:
                        self.wfile.write(f"id: {e['seq']}\ndata: {json.dumps(e, ensure_ascii=False)}\n\n".encode())
                        after = e["seq"]
                    if events:
                        self.wfile.flush()
                        quiet = time.time()
                    elif time.time() - quiet > 10:
                        self.wfile.write(b": keep-alive\n\n")
                        self.wfile.flush()
                        quiet = time.time()
                    time.sleep(0.2)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            for name, value in self.cors():
                self.send_header(name, value)
            self.end_headers()
            seq = -1
            try:
                while not app.closed.is_set():
                    jpeg, meta = app.camera.snapshot()
                    if jpeg and meta["seq"] != seq:
                        seq = meta["seq"]
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nX-Frame-Time: %.6f\r\nContent-Length: %d\r\n\r\n"
                                         % (meta["stamp"], len(jpeg)) + jpeg + b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def build(folder: Path, blocks: int = 4, seed: int = 0, default_profile: str = "explore", model: Path | None = None,
          map_name: str | None = W.DEFAULT_MAP.stem, bins: str | None = None, speed: float = 1.0):
    """(app, camera) ready to be served; the caller closes camera and sets app.closed / app.locator.closed."""
    world = W.World(seed, map_name)
    if bins:
        world.load_bins(bins)
    world.reset(blocks)
    camera = SimCamera(world, seed)
    control = Controller(folder, arm=W.WorldArm(world), hardware=False, camera=camera, default_profile=default_profile)
    if speed != 1.0:                    # the simulated servos arrive at once; only the waiting is shortened
        from motion_policy import profile_config
        for key in ("motion", "motion_explore"):
            for name in ("segment_ms", "settle_ms"):
                if name in control.base_cfg.get(key, {}):
                    control.base_cfg[key][name] = control.base_cfg[key][name] / speed
        control.cfg = profile_config(default_profile, control.base_cfg)
    return SimApp(control, camera, world, model), camera


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=8770)
    p.add_argument("--lan", action="store_true", help="listen on all interfaces instead of 127.0.0.1")
    p.add_argument("--blocks", type=int, default=4, help="blocks placed at random at start")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--speed", type=float, default=1.0, help="divide the per-segment waiting time (1 = as the real arm)")
    p.add_argument("--map", default=W.DEFAULT_MAP.stem, help="config/map_*.json name, or 'none' for a bare table")
    p.add_argument("--bins", help="config/bins_*.json name: stand its bins at their planned places")
    p.add_argument("--model", type=Path, help="ONNX block detector (default: src/vision/blocks_detector.py's own)")
    p.add_argument("--default-profile", choices=("explore", "normal"), default="explore")
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    folder = args.output or ROOT / "data" / "sim" / (time.strftime("%Y%m%d_%H%M%S") + "_" + secrets.token_hex(2))
    app, camera = build(folder, args.blocks, args.seed, args.default_profile, args.model, args.map, args.bins, args.speed)
    threading.Thread(target=app.poll, daemon=True).start()
    host = "0.0.0.0" if args.lan else "127.0.0.1"
    httpd = console.ThreadingHTTPServer((host, args.port), handler(app))
    httpd.daemon_threads = True
    app.port = args.port
    print(f"Arm simulation [no hardware]: http://{host}:{args.port}/sim/   (old console page: /)", flush=True)
    print(f"Session: {folder}", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop_sort()
        app.closed.set()
        app.locator.closed.set()
        app.control.stop()
        camera.close()
        httpd.server_close()


if __name__ == "__main__":
    main()
