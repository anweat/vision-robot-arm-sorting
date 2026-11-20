#!/bin/sh
# Bring the teleop console back after a board reboot. Nothing here is started at boot on the board.
#   1. board camera server on :8080 (same as deploy_to_board.sh, only if not already healthy)
#   2. phone IP-camera reverse tunnel: board 127.0.0.1:18081 -> phone (the board cannot reach the phone's Wi-Fi)
#   3. teleop console on the board :8765 (hardware, execute; motion still needs "on site" + enable in the page)
#   4. Mac 127.0.0.1:8765 -> board console
# Tunnels ride on the SSH ControlMaster, so they end with it. Usage:
#   archive/scripts/start_console.sh [phone-ip:port]     (optional third-person phone, default 192.168.110.185:8080)
set -eu

HOST="${BOARD_HOST:-root@192.168.137.100}"
PHONE="${1:-${PHONE:-192.168.110.185:8080}}"
CP="-o ControlPath=$HOME/.ssh/cm-%C"
PY=/root/miniconda3/envs/ros-arm-env/bin/python
VPY=venv-vision/bin/python

if ! ssh $CP -O check "$HOST" >/dev/null 2>&1; then
  echo "No SSH master. Open one first (asks for the board password once):"
  echo "  ssh -o ControlMaster=yes $CP -o ControlPersist=12h -fN $HOST"
  exit 1
fi

# the phone is optional (the camera-only pipeline does not use it); forward it only when it answers
if curl -sf -m 3 -o /dev/null "http://$PHONE/shot.jpg"; then USE_PHONE=1; else USE_PHONE=0; echo "phone: not in use"; fi

ssh $CP "$HOST" "cd /root/arm-app
  if ! curl -sf -m 2 http://127.0.0.1:8080/health >/dev/null; then
    nohup $PY app/src/adapters/board_camera_server.py --port 8080 > camera_server.log 2>&1 &
    echo \$! > camera_server.pid
    for i in 1 2 3 4 5 6 7 8 9 10; do sleep 1; curl -sf -m 1 http://127.0.0.1:8080/health >/dev/null && break; done  # conda imports take a few seconds
  fi
  printf 'camera: '; curl -s -m 2 http://127.0.0.1:8080/health; echo"

# forwards through the master; 'already' errors are harmless when re-run
[ "$USE_PHONE" = 1 ] && { ssh $CP -O forward -R "18081:$PHONE" "$HOST" 2>/dev/null || true; }
ssh $CP -O forward -L 8765:127.0.0.1:8765 "$HOST" 2>/dev/null || true

# a pid file, not pgrep -f: the pattern would also match this ssh command's own shell
ssh $CP "$HOST" "cd /root/arm-app
  if ! { [ -f teleop.pid ] && grep -q teleop /proc/\$(cat teleop.pid)/cmdline 2>/dev/null; }; then
    . /usr/local/Ascend/ascend-toolkit/set_env.sh >/dev/null 2>&1   # NPU runtime for the official garbage model
    setsid nohup $VPY app/src/teleop/server.py --hardware --execute \
      --camera-url http://127.0.0.1:8080/snapshot.jpg --phone-url http://127.0.0.1:18081/shot.jpg \
      > /tmp/teleop.log 2>&1 < /dev/null &
    echo \$! > teleop.pid; sleep 5
  fi
  echo console pid \$(cat teleop.pid); tail -2 /tmp/teleop.log"

printf 'mac -> console: '; curl -s -m 5 -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8765/
echo "open http://127.0.0.1:8765/ (only one console page at a time)"
