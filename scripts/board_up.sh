#!/bin/sh
# Run ON the board (docs 31 §2.1c): bring up camera server, console and gateway; safe to re-run.
#   ssh -p 2222 root@<phone Tailscale IP> 'sh /root/arm-app/scripts/board_up.sh'
#   ... 'sh /root/arm-app/scripts/board_up.sh stop'      stop the three (beacon keeps running)
#
# Starting the console does not move the arm: it only reads the servos until a client sends a motion command
# (doc 32 §1.2: the first motion command takes control). Check the arm's surroundings before driving it.
set -u
cd /root/arm-app || exit 1
CAMPY=/root/miniconda3/envs/ros-arm-env/bin/python
VPY=/root/arm-app/venv-vision/bin/python
TOKEN_FILE=data/gateway_token
mkdir -p data/web
# The table sheet (AGENTS.md: the two-bin A4 table is map_a4_v4); the gateway hands it to the flow it starts.
export ARM_MAP="${ARM_MAP:-map_a4_v4}"
# NPU runtime for every child: the console's official model and the flow's .om block detector
set +u   # set_env.sh reads variables that may be unset (LD_LIBRARY_PATH, …)
. /usr/local/Ascend/ascend-toolkit/set_env.sh >/dev/null 2>&1
set -u

alive() { [ -f "$1.pid" ] && kill -0 "$(cat "$1.pid")" 2>/dev/null; }

stop_one() {
  if alive "$1"; then kill "$(cat "$1.pid")" && echo "stopped $1"; fi
  rm -f "$1.pid"
}

if [ "${1:-}" = stop ]; then
  stop_one gateway; stop_one teleop; stop_one camera_server
  exit 0
fi

# 1. camera server :8080 (sole owner of /dev/video0)
if ! curl -sf -m 2 http://127.0.0.1:8080/health >/dev/null; then
  setsid nohup $CAMPY src/adapters/board_camera_server.py --port 8080 > camera_server.log 2>&1 < /dev/null &
  echo $! > camera_server.pid
  for i in 1 2 3 4 5 6 7 8 9 10; do sleep 1; curl -sf -m 1 http://127.0.0.1:8080/health >/dev/null && break; done
fi
printf 'camera:  '; curl -s -m 2 http://127.0.0.1:8080/health; echo

# 2. console 127.0.0.1:8765 (hardware)
if ! alive teleop; then
  setsid nohup $VPY src/teleop/server.py --hardware --execute \
    --camera-url http://127.0.0.1:8080/snapshot.jpg > /tmp/teleop.log 2>&1 < /dev/null &
  echo $! > teleop.pid
  for i in 1 2 3 4 5 6 7 8 9 10 11 12; do sleep 1; curl -sf -m 1 -o /dev/null http://127.0.0.1:8765/api/status && break; done
fi
printf 'console: '; curl -s -m 2 -o /dev/null -w '%{http_code}\n' http://127.0.0.1:8765/api/status

# 3. gateway :8090 on all interfaces (phone relay, eth1); --lan forces the token
[ -s "$TOKEN_FILE" ] || $VPY -c 'import secrets; print(secrets.token_urlsafe(12))' > "$TOKEN_FILE"
if ! alive gateway; then
  setsid nohup $VPY src/web/serve.py --console http://127.0.0.1:8765 --lan --port 8090 \
    --token "$(cat "$TOKEN_FILE")" --log data/web/log.jsonl --flow-python "$VPY" \
    > /tmp/gateway.log 2>&1 < /dev/null &
  echo $! > gateway.pid
  for i in 1 2 3 4 5 6; do sleep 1; curl -sf -m 1 -o /dev/null -H "X-Token: $(cat "$TOKEN_FILE")" http://127.0.0.1:8090/api/health && break; done
fi
printf 'gateway: '; curl -s -m 2 -H "X-Token: $(cat "$TOKEN_FILE")" http://127.0.0.1:8090/api/health; echo
echo "map ARM_MAP=$ARM_MAP; token in /root/arm-app/$TOKEN_FILE"
