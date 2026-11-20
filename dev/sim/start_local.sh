#!/bin/sh
# Mac / Linux counterpart of start_windows.cmd (doc 36): simulator console + gateway web page in the
# background, logs and pids under data/web/. Simulation only: not a statement about the real arm.
#
#   sh dev/sim/start_local.sh            start (A4 map with two bins: sim :8771, page :8093)
#   sh dev/sim/start_local.sh stop       stop the two started here
#   sh dev/sim/start_local.sh status     python3 dev/tools/check_links.py --only local
#   SIM_PORT=8770 WEB_PORT=8090 BINS= sh dev/sim/start_local.sh     the layout the phone App README uses
#                                         (adb reverse tcp:18090 tcp:8090, see dev/tools/phone.sh sim)
#
# Needs uv; the first run downloads opencv / numpy / onnxruntime (~70 MB).
set -u
cd "$(dirname "$0")/../.." || exit 1
SIM_PORT="${SIM_PORT:-8771}"
WEB_PORT="${WEB_PORT:-8093}"
BINS="${BINS-bins_acrylic_v2_pair}"
export ARM_MAP="${ARM_MAP:-map_a4_v4}"
RUN="data/web"
mkdir -p "$RUN"
WITH='--no-project --with opencv-python<5 --with numpy --with onnxruntime'

alive() { [ -f "$RUN/$1.pid" ] && kill -0 "$(cat "$RUN/$1.pid")" 2>/dev/null; }
listening() { nc -z 127.0.0.1 "$1" 2>/dev/null; }

case "${1:-start}" in
stop)
  for n in local_gw local_sim; do
    if alive "$n"; then kill "$(cat "$RUN/$n.pid")" && echo "stopped $n"; fi
    rm -f "$RUN/$n.pid"
  done
  exit 0 ;;
status)
  exec python3 dev/tools/check_links.py --only local --local-ports "$WEB_PORT,$SIM_PORT" ;;
start) ;;
*) echo "usage: $0 [start|stop|status]"; exit 2 ;;
esac

command -v uv >/dev/null || { echo "uv not found: brew install uv  (or python3 -m pip install uv)"; exit 1; }

# 1. simulator console
if alive local_sim; then
  echo "sim: already running (pid $(cat "$RUN/local_sim.pid"))"
elif listening "$SIM_PORT"; then
  echo "sim: :$SIM_PORT is taken by another program; set SIM_PORT"; exit 1
else
  # shellcheck disable=SC2086
  nohup uv run $WITH dev/sim/sim_server.py --port "$SIM_PORT" --speed 4 --map "$ARM_MAP" \
    ${BINS:+--bins "$BINS"} > "$RUN/local_sim.log" 2>&1 < /dev/null &
  echo $! > "$RUN/local_sim.pid"
  printf 'sim: waiting for :%s (first run downloads packages) ' "$SIM_PORT"
  i=0
  until curl -s -m 1 -o /dev/null "http://127.0.0.1:$SIM_PORT/api/status"; do
    i=$((i + 1))
    if [ "$i" -gt 300 ] || ! alive local_sim; then echo; echo "sim did not come up: $RUN/local_sim.log"; tail -5 "$RUN/local_sim.log"; exit 1; fi
    printf .; sleep 1
  done
  echo " up"
fi

# 2. gateway
if alive local_gw; then
  echo "gateway: already running (pid $(cat "$RUN/local_gw.pid"))"
elif listening "$WEB_PORT"; then
  echo "gateway: :$WEB_PORT is taken by another program; set WEB_PORT"; exit 1
else
  nohup python3 app/src/web/serve.py --console "http://127.0.0.1:$SIM_PORT" --log "$RUN/log.jsonl" \
    --port "$WEB_PORT" --flow-python "uv run $WITH python" ${BINS:+--pipeline-args "--bins $BINS"} \
    > "$RUN/local_gw.log" 2>&1 < /dev/null &
  echo $! > "$RUN/local_gw.pid"
  i=0
  until curl -s -m 1 -o /dev/null "http://127.0.0.1:$WEB_PORT/api/health"; do
    i=$((i + 1))
    if [ "$i" -gt 20 ] || ! alive local_gw; then echo "gateway did not come up: $RUN/local_gw.log"; tail -5 "$RUN/local_gw.log"; exit 1; fi
    sleep 1
  done
fi

echo "page http://127.0.0.1:$WEB_PORT/   simulator http://127.0.0.1:$SIM_PORT/sim/   logs $RUN/local_*.log"
python3 dev/tools/check_links.py --only local --local-ports "$WEB_PORT,$SIM_PORT"
