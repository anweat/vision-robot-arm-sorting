#!/usr/bin/env python3
"""Check every hop from this computer to the board, plus the local simulator and gateway (doc 36).

Read-only: it opens TCP connections, sends GET requests, runs `tailscale status/ping`, `adb` queries and
`scripts/board_check.py` on the board over SSH. Nothing is started, stopped or moved.

    python3 dev/tools/check_links.py                   # local + Tailscale + phone relay + board + phone (adb)
    python3 dev/tools/check_links.py --only local      # just the simulator / gateway on this computer
    python3 dev/tools/check_links.py --board root@192.168.137.100      # board over the cable, no relay
    python3 dev/tools/check_links.py --json            # machine-readable

Sections (each can be picked with --only, comma-separated):
  local   simulator consoles and gateways on 127.0.0.1 (ports --local-ports)
  ts      Tailscale on this computer and the relay phone (--phone, default $ARM_PHONE or v2339a)
  relay   the phone's forwarded ports: gateway :8090, SSH :2222, wireless adb :5555
  board   scripts/board_check.py over SSH through the relay (or --board), then the gateway end to end
  phone   adb: USB function (rndis), the App and Tailscale processes, whether Android froze the App

Exit code: 0 when nothing failed, 1 otherwise (warnings do not count). HTTP goes direct: the proxy
environment (Clash and the like) is ignored, it breaks requests to Tailscale addresses (doc 31 §2.1a).
"""

import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

APP_PACKAGE = "com.example.myapplication00001"
TS_PACKAGE = "com.tailscale.ipn"
GATEWAY_LOCAL = 8090  # the App relay listens here on the phone
SSH_LOCAL = 2222
ADB_PORT = 5555
SECTIONS = ("local", "ts", "relay", "board", "phone")

OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))
NO_PROXY_ENV = {k: v for k, v in os.environ.items() if "proxy" not in k.lower()}


class Report:
    MARK = {"ok": "✓", "warn": "!", "fail": "✗", "info": "·"}

    def __init__(self):
        self.items = []
        self.section = None

    def head(self, title):
        self.section = title
        if not ARGS.json:
            print(f"\n[{title}]", flush=True)

    def add(self, level, text, hint=None):
        self.items.append(dict(section=self.section, level=level, text=text, hint=hint))
        if not ARGS.json:
            print(f"  {self.MARK[level]} {text}", flush=True)
            if hint and level in ("warn", "fail"):
                print(f"      → {hint}", flush=True)

    @property
    def failed(self):
        return any(i["level"] == "fail" for i in self.items)


def http(url, token=None, timeout=4.0):
    """(status, json or None, ms, error). status 0 = no HTTP answer."""
    req = urllib.request.Request(url)
    if token:
        req.add_header("X-Token", token)
    t0 = time.monotonic()
    try:
        with OPENER.open(req, timeout=timeout) as r:
            raw, status = r.read(), r.status
    except urllib.error.HTTPError as e:
        raw, status = e.read(), e.code
    except Exception as e:  # refused, reset, timeout
        reason = getattr(e, "reason", e)
        return 0, None, round((time.monotonic() - t0) * 1000), str(reason)
    ms = round((time.monotonic() - t0) * 1000)
    try:
        return status, json.loads(raw.decode("utf-8")), ms, None
    except ValueError:
        return status, None, ms, None


def tcp(host, port, timeout=3.0):
    """ms to connect, or None."""
    t0 = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return round((time.monotonic() - t0) * 1000)
    except OSError:
        return None


def run(cmd, timeout=15, stdin=None):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=NO_PROXY_ENV, input=stdin)
        return r.returncode, r.stdout, r.stderr
    except (OSError, subprocess.TimeoutExpired) as e:
        return -1, "", str(e)


# ---------------------------------------------------------------- local

def check_local(rep):
    rep.head("本机：仿真与网关")
    found = 0
    for port in ARGS.local_ports:
        if tcp("127.0.0.1", port, timeout=0.5) is None:
            continue
        found += 1
        status, body, ms, _ = http(f"http://127.0.0.1:{port}/api/health", token=ARGS.local_token)
        if status == 200 and body and "console" in body:
            sim = "仿真" if body.get("console_sim") else ("实机" if body.get("console_sim") is False else "?")
            ready = body.get("console_ready")
            rep.add("ok" if ready else "warn",
                    f"网关 :{port} → 控制台 {body.get('console')}（{sim}）console_ready={ready}  http://127.0.0.1:{port}/",
                    None if ready else f"网关连不上控制台：{body.get('console_error')}；先起仿真")
            continue
        if status == 401:
            rep.add("info", f"网关 :{port} 要口令（--local-token 传入可查控制台）")
            continue
        status, body, ms, _ = http(f"http://127.0.0.1:{port}/api/status")
        if status == 200 and body:
            mode = body.get("mode", "?")
            rep.add("ok", f"控制台 :{port} mode={mode}（{ms} ms）  http://127.0.0.1:{port}/sim/")
        else:
            rep.add("info", f":{port} 有程序在听，但不是网关/控制台（{status or '无响应'}）")
    if not found:
        rep.add("info", f"本机 {', '.join(map(str, ARGS.local_ports))} 都没开",
                "仿真 + 网关：sh dev/sim/start_local.sh（Windows 双击 dev/sim/start_windows.cmd）")


# ---------------------------------------------------------------- tailscale

def tailscale_bin():
    for c in ("/Applications/Tailscale.app/Contents/MacOS/Tailscale", shutil.which("tailscale")):
        if c and os.path.exists(c):
            return c
    return None


def check_tailscale(rep, ctx):
    rep.head("Tailscale")
    ts = tailscale_bin()
    if not ts:
        rep.add("fail", "本机没有 tailscale 命令", "装 Tailscale 并登录同一账号")
        return
    code, out, err = run([ts, "status", "--json"])
    try:
        st = json.loads(out)
    except ValueError:
        rep.add("fail", f"tailscale status 失败：{(err or out).strip()[:120]}", "打开 Tailscale 并登录")
        return
    me = st.get("Self", {})
    if st.get("BackendState") != "Running":
        rep.add("fail", f"本机 Tailscale 状态 {st.get('BackendState')}", "打开 Tailscale 并连接")
        return
    rep.add("ok", f"本机 {me.get('HostName')} {', '.join(me.get('TailscaleIPs', [])[:1])}")
    want = ARGS.phone.lower()
    phone = None
    for peer in (st.get("Peer") or {}).values():
        names = [peer.get("HostName", "").lower(), peer.get("DNSName", "").split(".")[0].lower()]
        if want in names or want in peer.get("TailscaleIPs", []):
            phone = peer
            break
    if not phone:
        rep.add("fail", f"tailnet 里没有叫 {ARGS.phone} 的设备", "--phone <名字或 100.x 地址>；或手机上登录 Tailscale")
        return
    ip = phone["TailscaleIPs"][0]
    ctx["phone_ip"] = ip
    if not phone.get("Online"):
        rep.add("fail", f"手机 {phone.get('HostName')} {ip} 离线（最后在线 {phone.get('LastSeen', '?')[:16]}）",
                "手机上打开 Tailscale；vivo 要在「后台耗电管理」允许 Tailscale 后台高耗电（31 §2.1b）")
        ctx["phone_offline"] = True
        return
    code, out, err = run([ts, "ping", "-c", "2", "--timeout", "3s", ip], timeout=12)
    m = re.findall(r"pong from \S+ \([\d.]+\) via (\S+(?: \S+)?) in (\d+)ms", out)
    if m:
        via, ms = m[-1]
        direct = not via.startswith("DERP")
        rep.add("ok" if direct else "warn", f"手机 {phone.get('HostName')} {ip} 在线，{'直连' if direct else '中转 ' + via} {ms} ms",
                None if direct else "没打通直连，延迟会高一些；两端换个网络或等它自己打洞")
    else:
        rep.add("fail", f"手机 {ip} 显示在线但 tailscale ping 不通：{(out + err).strip()[-120:]}",
                "手机可能刚被省电冻结：点亮屏幕打开 Tailscale 再试")


# ---------------------------------------------------------------- relay

def check_relay(rep, ctx):
    rep.head("手机跳板（App 前台服务）")
    ip = ctx.get("phone_ip")
    if not ip:
        rep.add("fail", "不知道手机地址，跳过", "先让 [Tailscale] 一节通过，或 --phone 100.x.x.x")
        return
    ssh_ms = tcp(ip, SSH_LOCAL)
    gw_ms = tcp(ip, GATEWAY_LOCAL)
    ctx["relay_ssh"] = ssh_ms is not None
    if ssh_ms is None and gw_ms is None:
        rep.add("fail", f"手机 :{GATEWAY_LOCAL}、:{SSH_LOCAL} 都连不上：跳板没开",
                "手机打开 App →「连接」页打开「跳板」；vivo 要允许 App 后台高耗电，否则退后台即被冻结")
        return
    rep.add("ok" if ssh_ms is not None else "fail", f"SSH 转发 :{SSH_LOCAL} " + (f"建连 {ssh_ms} ms" if ssh_ms is not None else "连不上"))
    if gw_ms is None:
        rep.add("fail", f"网关转发 :{GATEWAY_LOCAL} 连不上")
        return
    status, body, ms, err = http(f"http://{ip}:{GATEWAY_LOCAL}/api/health")
    ctx["relay_gw_status"] = status
    if status in (200, 401):
        rep.add("ok", f"网关转发 :{GATEWAY_LOCAL} 有网关应答（{status}，{ms} ms）")
    else:
        rep.add("fail", f"网关转发 :{GATEWAY_LOCAL} 建连成功但没有 HTTP 应答（{err or status}）",
                "跳板转发到的板卡端口上没有网关：看下面 [板卡] 一节的网关端口，改 App「板卡网关端口」或在板上起 :8090")


# ---------------------------------------------------------------- board

def check_board(rep, ctx):
    rep.head("板卡")
    if ARGS.board:
        target, port = ARGS.board, ARGS.board_port
    elif ctx.get("relay_ssh"):
        target, port = f"root@{ctx['phone_ip']}", SSH_LOCAL
    else:
        rep.add("fail", "没有通往板卡的 SSH（跳板 :2222 不通）", "--board root@192.168.137.100 走网线")
        return
    ssh = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-p", str(port), target]
    t0 = time.monotonic()
    code, out, err = run(ssh + ["cd /root/arm-app && test -f scripts/board_check.py && "
                                "python3 scripts/board_check.py --json --show-token || echo NO_CHECK"], timeout=40)
    ms = round((time.monotonic() - t0) * 1000)
    if "NO_CHECK" in out:
        rep.add("fail", "板上没有 scripts/board_check.py",
                f"scp -P {port} app/scripts/board_check.py {target}:/root/arm-app/scripts/（或 app/scripts/deploy.sh）")
        return
    try:
        res = json.loads(out.strip().splitlines()[-1])
    except (ValueError, IndexError):
        rep.add("fail", f"SSH {target}:{port} 失败：{(err or out).strip()[-160:]}",
                "密钥未装（ssh-copy-id）或板卡没开机；跳板 SSH 通但这里失败时看手机 App 是否被冻结")
        return
    rep.add("ok", f"SSH {target} -p {port}，自检 {ms} ms（{res.get('host')}）")
    ctx["board"] = res
    end_to_end = relay_target(ctx, res)  # before listing, so a stale "no gateway on :8090" can be relabelled
    for i in res["items"]:
        if i["key"] == "gateway.relay" and ctx.get("relay_target"):
            rep.add("info", f"板卡 :8090 没有网关；App 跳板已改为转发到板卡 :{ctx['relay_target']}")
            continue
        rep.add(i["level"], i["text"], i.get("hint"))
    if end_to_end:
        rep.add(*end_to_end)


def relay_target(ctx, res):
    """Which board gateway answers behind the phone's :8090? Tries each gateway's token (never printed)."""
    if ARGS.board or not ctx.get("phone_ip") or ctx.get("relay_gw_status") not in (200, 401):
        return None
    tokens = res.get("tokens") or {}
    for gw in res.get("gateways", []):
        status, body, ms, _ = http(f"http://{ctx['phone_ip']}:{GATEWAY_LOCAL}/api/health",
                                   token=tokens.get(str(gw["port"])))
        if status == 200 and body and body.get("console") == gw["console"]:
            ctx["relay_target"] = gw["port"]
            sim = "仿真" if body.get("console_sim") else "实机"
            return ("ok", f"端到端：本机 → 手机 :{GATEWAY_LOCAL} → 板卡网关 :{gw['port']}（{sim}，口令 {gw['token']}）{ms} ms")
    return ("warn", "端到端：跳板后面有网关应答，但板上各网关的口令都对不上",
            "App 连接页「板卡网关端口」与口令要和上面列出的某个网关一致")


# ---------------------------------------------------------------- phone (adb)

def check_phone(rep, ctx):
    rep.head("手机（adb）")
    if not shutil.which("adb"):
        rep.add("info", "本机没有 adb，跳过")
        return
    code, out, _ = run(["adb", "devices"])
    devices = [l.split()[0] for l in out.splitlines()[1:] if l.strip().endswith("device")]
    ip = ctx.get("phone_ip")
    serial = next((d for d in devices if ip and d.startswith(ip + ":")), None) or (devices[0] if len(devices) == 1 else None)
    if not serial and ip and tcp(ip, ADB_PORT, timeout=2) is not None:
        run(["adb", "connect", f"{ip}:{ADB_PORT}"])
        serial = f"{ip}:{ADB_PORT}"
    if not serial:
        rep.add("info", "没有连着的手机（adb）",
                "手机插 USB：adb tcpip 5555 后经 Tailscale 连；插着板卡时用「无线调试」配对：sh dev/tools/phone.sh connect <IP:端口>")
        return
    sh = lambda c: run(["adb", "-s", serial, "shell", c], timeout=15)[1].strip()
    rep.add("ok", f"adb {serial}（{sh('getprop ro.product.model')}，Android {sh('getprop ro.build.version.release')}）")
    usb = sh("getprop sys.usb.state")
    rep.add("ok" if "rndis" in usb else "warn", f"USB 功能 {usb or '?'}",
            None if "rndis" in usb else "插着板卡时要是 rndis：sh dev/tools/phone.sh rndis")
    for pkg, name in ((APP_PACKAGE, "App"), (TS_PACKAGE, "Tailscale")):
        pid = sh(f"pidof {pkg}")
        if not pid:
            rep.add("fail" if name == "App" else "warn", f"{name} 没在运行",
                    "sh dev/tools/phone.sh app" if name == "App" else "手机上打开 Tailscale")
            continue
        procs = sh(f"dumpsys activity processes {pkg}")
        frozen = "isFrozen=true" in procs
        fg = "fgs" in procs.lower() or "foreground" in procs.lower()
        rep.add("warn" if frozen else "ok", f"{name} pid {pid}" + ("，已被系统冻结" if frozen else ""),
                "vivo「电池 → 后台耗电管理」给它「允许后台高耗电」，或把它切回前台" if frozen else None)
        if name == "App":
            svc = sh(f"dumpsys activity services {pkg}/.relay.RelayService")
            on = "ServiceRecord" in svc
            rep.add("ok" if on else "warn", "跳板服务 " + ("在跑" if on else "没在跑"),
                    None if on else "App「连接」页打开「跳板」")


def main(argv=None):
    global ARGS
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--only", default=",".join(SECTIONS), help="comma-separated: " + ",".join(SECTIONS))
    p.add_argument("--phone", default=os.environ.get("ARM_PHONE", "v2339a"), help="relay phone: Tailscale name or IP")
    p.add_argument("--board", help="SSH target for the board instead of the relay, e.g. root@192.168.137.100")
    p.add_argument("--board-port", type=int, default=22)
    p.add_argument("--local-ports", default="8090,8091,8092,8093,8770,8771",
                   type=lambda s: [int(x) for x in s.split(",") if x])
    p.add_argument("--local-token", help="token of a local gateway started with --token")
    p.add_argument("--json", action="store_true")
    ARGS = p.parse_args(argv)
    want = {s.strip() for s in ARGS.only.split(",")}
    rep, ctx = Report(), {}
    if not ARGS.json:
        print(time.strftime("链路检查 %Y-%m-%d %H:%M:%S"))
    if "local" in want:
        check_local(rep)
    if want & {"ts", "relay", "board", "phone"} and not (ARGS.board and want <= {"local", "board"}):
        check_tailscale(rep, ctx)
    if "relay" in want and not ARGS.board:
        check_relay(rep, ctx)
    if "board" in want:
        check_board(rep, ctx)
    if "phone" in want:
        check_phone(rep, ctx)
    if ARGS.json:
        print(json.dumps(dict(items=rep.items, phone_ip=ctx.get("phone_ip"), relay_target=ctx.get("relay_target")),
                         ensure_ascii=False))
    else:
        bad = [i for i in rep.items if i["level"] == "fail"]
        warn = [i for i in rep.items if i["level"] == "warn"]
        print(f"\n结果：{len(bad)} 项失败，{len(warn)} 项警告")
    return 1 if rep.failed else 0


ARGS = None

if __name__ == "__main__":
    sys.exit(main())
