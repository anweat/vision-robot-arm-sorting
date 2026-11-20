#!/usr/bin/env python3
"""Board self-check: which services are up, on which ports, and whether the phone link is there (doc 36).

Read-only: it reads /proc, the network interfaces and the services' own status endpoints. It starts, stops
and moves nothing. Runs on the board with the stock Python 3 (3.9/3.10), standard library only.

    python3 scripts/board_check.py                  # in /root/arm-app; human-readable
    python3 scripts/board_check.py --json           # for dev/tools/check_links.py on the Mac
    python3 scripts/board_check.py --show-token     # print gateway tokens in full (default: masked)

Exit code: 0 when nothing failed, 1 otherwise (warnings do not count).

What it looks for:
  camera   src/adapters/board_camera_server.py   /health
  console  src/teleop/server.py                  /api/status   (127.0.0.1 only)
  gateway  src/web/serve.py (any number, any port) /api/health with its own --token
  sim      dev/sim/sim_server.py                 /api/status   (a simulator someone runs on the board)
  beacon   systemd arm-beacon                    (UDP 37020 broadcast for the phone relay)
  links    eth1 (cable), usb0 (board Type-C gadget), the phone's RNDIS link (driver rndis_host)
The phone relay forwards to the board gateway port set in the App (default 8090); a gateway on any other
port is reported with the setting the App needs.
"""

import argparse
import json
import os
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path

APP = Path(__file__).resolve().parents[1]
RELAY_DEFAULT_PORT = 8090
TOKEN_FILE = APP / "data" / "gateway_token"

ROLES = (  # (role, marker in the command line, default port)
    ("gateway", "web/serve.py", 8080),
    ("console", "teleop/server.py", 8765),
    ("camera", "board_camera_server.py", 8080),
    ("sim", "sim_server.py", 8770),
    ("beacon", "board_beacon.py", None),
)

OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def http_json(url, token=None, timeout=2.0):
    """(status, body or None, ms). status 0 = no HTTP answer (refused, reset, timeout)."""
    req = urllib.request.Request(url)
    if token:
        req.add_header("X-Token", token)
    t0 = time.monotonic()
    try:
        with OPENER.open(req, timeout=timeout) as r:
            raw = r.read()
            status = r.status
    except urllib.error.HTTPError as e:
        raw, status = e.read(), e.code
    except Exception:
        return 0, None, round((time.monotonic() - t0) * 1000)
    ms = round((time.monotonic() - t0) * 1000)
    try:
        return status, json.loads(raw.decode("utf-8")), ms
    except ValueError:
        return status, None, ms


def arg_value(argv, name):
    for i, a in enumerate(argv):
        if a == name and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(name + "="):
            return a.split("=", 1)[1]
    return None


def processes():
    """[{pid, role, argv, cwd, started}] for the services this script knows about."""
    found = []
    boot = time.time() - float(Path("/proc/uptime").read_text().split()[0])
    hz = os.sysconf("SC_CLK_TCK")
    for d in Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            argv = (d / "cmdline").read_bytes().decode("utf-8", "replace").split("\0")
        except OSError:
            continue
        argv = [a for a in argv if a]
        line = " ".join(argv)
        for role, marker, default_port in ROLES:
            if marker in line and "board_check" not in line:
                try:
                    cwd = os.readlink(d / "cwd")
                except OSError:
                    cwd = None
                try:
                    stat = (d / "stat").read_text().rsplit(")", 1)[1].split()
                    started = boot + int(stat[19]) / hz
                except (OSError, IndexError, ValueError):
                    started = None
                port = arg_value(argv, "--port")
                found.append(dict(pid=int(d.name), role=role, argv=argv, cwd=cwd, started=started,
                                  port=int(port) if port and port.isdigit() else default_port,
                                  token=arg_value(argv, "--token"), console=arg_value(argv, "--console"),
                                  lan="--lan" in argv))
                break
    return sorted(found, key=lambda p: (p["role"], p["port"] or 0, p["pid"]))


def listening():
    """Ports with an IPv4/IPv6 TCP listener."""
    ports = set()
    for name in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            rows = Path(name).read_text().splitlines()[1:]
        except OSError:
            continue
        for row in rows:
            f = row.split()
            if len(f) > 3 and f[3] == "0A":  # TCP_LISTEN
                ports.add(int(f[1].rsplit(":", 1)[1], 16))
    return ports


def interfaces():
    """[{name, state, ipv4, driver}] except lo."""
    out = subprocess.run(["ip", "-j", "addr", "show"], capture_output=True, text=True, check=False).stdout
    try:
        items = json.loads(out or "[]")
    except ValueError:
        items = []
    links = []
    for it in items:
        name = it.get("ifname")
        if name == "lo":
            continue
        try:
            driver = os.path.basename(os.readlink(f"/sys/class/net/{name}/device/driver"))
        except OSError:
            driver = None
        v4 = [f"{a['local']}/{a['prefixlen']}" for a in it.get("addr_info", []) if a.get("family") == "inet"]
        links.append(dict(name=name, state=it.get("operstate"), ipv4=v4, driver=driver))
    return links


def systemd_active(unit):
    r = subprocess.run(["systemctl", "is-active", unit], capture_output=True, text=True, check=False)
    return r.stdout.strip() or "unknown"


def mask(token, show):
    if not token:
        return None
    return token if show else token[:2] + "…"


def check(show_token=False):
    procs = processes()
    ports = listening()
    links = interfaces()
    saved_token = TOKEN_FILE.read_text().strip() if TOKEN_FILE.exists() else None
    items = []  # {level: ok|warn|fail|info, key, text, hint}

    def add(level, key, text, hint=None):
        items.append(dict(level=level, key=key, text=text, hint=hint))

    by_role = {}
    for p in procs:
        by_role.setdefault(p["role"], []).append(p)

    # network links
    phone = [l for l in links if l["driver"] == "rndis_host"]
    eth1 = next((l for l in links if l["name"] == "eth1"), None)
    if eth1 and eth1["ipv4"] and eth1["state"] == "UP":
        add("info", "link.eth1", f"网线 eth1 {', '.join(eth1['ipv4'])}")
    if phone:
        for l in phone:
            if l["ipv4"]:
                add("ok", "link.phone", f"手机 USB 共享 {l['name']} {', '.join(l['ipv4'])}")
            else:
                add("fail", "link.phone", f"手机 USB 共享 {l['name']} 在但没有 IPv4（{l['state']}）",
                    "手机上重开 USB 网络共享，或 Mac 上 dev/tools/phone.sh rndis；板上 networkctl status " + l["name"])
    else:
        add("warn", "link.phone", "没有手机 USB 共享网卡（rndis_host）",
            "手机插板卡 Type-A 口并开 USB 网络共享（31 §2.1d）；经网线检查时可忽略")

    beacon = systemd_active("arm-beacon")
    add("ok" if beacon == "active" else "warn", "beacon", f"广播 arm-beacon: {beacon}",
        None if beacon == "active" else "systemctl restart arm-beacon（App 板卡地址留空时靠它找板卡）")

    # camera
    cams = by_role.get("camera", [])
    status, body, ms = http_json("http://127.0.0.1:8080/health")
    if status == 200 and body:
        age = body.get("latest_age_s")
        stale = age is None or age > 2
        add("warn" if stale else "ok", "camera",
            f"相机 :8080 {body.get('fps_since_open')} fps，最新帧 {age} s 前，重开 {body.get('reopen_count')} 次（{ms} ms）",
            "帧不更新：看 camera_server.log，USB 相机可能掉了" if stale else None)
    else:
        add("fail", "camera", "相机 :8080 无响应" + ("（进程在）" if cams else "（没有进程）"),
            "sh scripts/board_up.sh")

    # console
    status, body, ms = http_json("http://127.0.0.1:8765/api/status")
    if status == 200 and body:
        age = body.get("age_s")
        text = (f"控制台 :8765 {body.get('mode')}，回读 {age if age is None else round(age, 2)} s 前，"
                f"{'有任务在动' if body.get('busy') else '空闲'}，revision {body.get('revision')}")
        bad = bool(body.get("error")) or age is None or age > 3
        add("warn" if bad else "ok", "console", text + (f"，error={body.get('error')}" if body.get("error") else ""),
            "回读不更新或报错：看 /tmp/teleop.log（舵机总线、供电）" if bad else None)
    else:
        add("fail", "console", "控制台 :8765 无响应" + ("（进程在）" if by_role.get("console") else "（没有进程）"),
            "sh scripts/board_up.sh；日志 /tmp/teleop.log")

    # gateways (any number, any port)
    gws = []
    for p in by_role.get("gateway", []):
        status, body, ms = http_json(f"http://127.0.0.1:{p['port']}/api/health", token=p["token"])
        gw = dict(pid=p["pid"], port=p["port"], console=p["console"], lan=p["lan"], cwd=p["cwd"],
                  started=p["started"], token=mask(p["token"], False),
                  token_is_saved=bool(p["token"]) and p["token"] == saved_token,
                  status=status, health=body, ms=ms)
        gws.append(gw)
        where = "所有网卡" if p["lan"] else "仅本机"
        tok = "口令=data/gateway_token" if gw["token_is_saved"] else (f"口令={gw['token']}" if p["token"] else "无口令")
        started = time.strftime("%m-%d %H:%M", time.localtime(p["started"])) if p["started"] else "?"
        desc = f"网关 :{p['port']}（{where}，{tok}，控制台 {p['console']}，{started} 起，pid {p['pid']}）"
        if status == 200 and body:
            sim = "仿真" if body.get("console_sim") else "实机"
            ready = body.get("console_ready")
            add("ok" if ready else "warn", f"gateway.{p['port']}",
                f"{desc} console_ready={ready} {sim}（{ms} ms）",
                None if ready else f"网关连不上控制台：{body.get('console_error')}")
        else:
            add("fail", f"gateway.{p['port']}", f"{desc} /api/health 返回 {status or '无响应'}")
    if not gws:
        add("fail", "gateway", "没有网关进程", "sh scripts/board_up.sh（起在 :8090，口令 data/gateway_token）")
    elif not any(g["port"] == RELAY_DEFAULT_PORT for g in gws):
        lan = [g for g in gws if g["lan"] and g["health"]]
        pick = [g for g in lan if not g["health"].get("console_sim")] or lan
        others = " 或 ".join(str(g["port"]) for g in pick)
        add("warn", "gateway.relay",
            f"板卡 :{RELAY_DEFAULT_PORT} 上没有网关，App 跳板默认转发到这里，手机会连不上（连接被重置）",
            f"App「连接」页「板卡网关端口」改成 {others or '网关端口'} 并重开跳板，口令填该网关的；"
            f"或 sh scripts/board_up.sh 在 :{RELAY_DEFAULT_PORT} 再起一个")
    real = [g for g in gws if g["health"] and not g["health"].get("console_sim")]
    if len({g["console"] for g in real}) < len(real):
        add("warn", "gateway.dup", "多个网关接着同一个实机控制台：各自抢操作权，会互相 409",
            "只留一个；停掉手动起的那个（kill <pid>）")

    # simulator on the board
    for p in by_role.get("sim", []):
        status, body, ms = http_json(f"http://127.0.0.1:{p['port']}/api/status")
        add("info" if status == 200 else "warn", f"sim.{p['port']}",
            f"板上仿真控制台 :{p['port']} pid {p['pid']}（{status or '无响应'}）")

    # resources
    load = os.getloadavg()
    mem = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        k, v = line.split(":", 1)
        mem[k] = int(v.split()[0]) // 1024
    cores = os.cpu_count() or 1
    add("warn" if load[0] > cores * 4 else "info", "load",
        f"负载 {load[0]:.1f}/{load[1]:.1f}/{load[2]:.1f}（{cores} 核），可用内存 {mem.get('MemAvailable')} MB")

    return dict(t=round(time.time(), 1), host=os.uname().nodename, items=items, gateways=gws,
                processes=[dict(pid=p["pid"], role=p["role"], port=p["port"], cwd=p["cwd"],
                                cmd=" ".join(p["argv"])[:200]) for p in procs],
                listening=sorted(ports), links=links, relay_default_port=RELAY_DEFAULT_PORT,
                tokens={str(g["port"]): next((p["token"] for p in procs if p["pid"] == g["pid"]), None)
                        for g in gws} if show_token else None)


MARK = {"ok": "✓", "warn": "!", "fail": "✗", "info": "·"}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--json", action="store_true")
    p.add_argument("--show-token", action="store_true")
    args = p.parse_args(argv)
    result = check(show_token=args.show_token)
    failed = any(i["level"] == "fail" for i in result["items"])
    if args.json:
        print(json.dumps(result, ensure_ascii=False))
        return 1 if failed else 0
    print(f"板卡 {result['host']}  {time.strftime('%Y-%m-%d %H:%M:%S')}")
    for i in result["items"]:
        print(f"  {MARK[i['level']]} {i['text']}")
        if i["hint"] and i["level"] in ("warn", "fail"):
            print(f"      → {i['hint']}")
    for port, tok in (result["tokens"] or {}).items():
        print(f"  口令 :{port} = {tok}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
